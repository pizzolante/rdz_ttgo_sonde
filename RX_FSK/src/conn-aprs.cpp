#include "../features.h"

#if FEATURE_APRS

#include "conn-aprs.h"
#include "aprs.h"
#include "pmu.h"
#include "posinfo.h"
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <sys/socket.h>
#include <lwip/dns.h>

#include <ESPAsyncWebServer.h>

// KISS over TCP for communicating with APRSdroid
static WiFiServer tncserver(14580);
static WiFiClient tncclient;

// APRS over TCP for radiosondy.info etc
// now we support up to two APRS connections (e.g. radiosondy.info, wettersonde.net)
#define N_APRS 2
#define APRS_PRIMARY_HOST "radiosondy.info:14580"
#define APRS_SECONDARY_DEFAULT_HOST "rotate.aprs.net:14580"
#define APRS_DEST_RADIOSONDY "APRRDZ"
#define APRS_DEST_APRSIS "APRDZ1,TCPIP*"
struct st_aprs {
    int tcpclient;
    ip_addr_t tcpclient_ipaddr;
    int port;
    unsigned long last_in;
    uint8_t tcpclient_state;
    uint32_t conn_ts;
} aprs[2]={0}; 
enum { TCS_DISCONNECTED, TCS_DNSLOOKUP, TCS_DNSRESOLVED, TCS_CONNECTING, TCS_LOGIN, TCS_CONNECTED };

char udphost[64];
int udpport;

extern const char *version_id;
extern PMU *pmu;

extern WiFiUDP udp;

void tcpclient_fsm();

static int aprs_index(const st_aprs *a) {
    return (a == aprs) ? 0 : 1;
}

static const char *aprs_signature() {
    return sonde.config.signature[0] ? sonde.config.signature : "RDZTTGO";
}

static const char *aprs_effective_host(const st_aprs *a) {
    if (aprs_index(a) == 0) {
        return APRS_PRIMARY_HOST;
    }
    if (sonde.config.tcpfeed.host2[0]) {
        return sonde.config.tcpfeed.host2;
    }
    return APRS_SECONDARY_DEFAULT_HOST;
}

static const char *aprs_dest_for(const st_aprs *a) {
    return (aprs_index(a) == 0) ? APRS_DEST_RADIOSONDY : APRS_DEST_APRSIS;
}

static const char *aprs_tail_for(const st_aprs *a) {
    return (aprs_index(a) == 0) ? aprs_signature() : NULL;
}

static bool aprs_feed_enabled(int idx) {
    if (idx == 0) return sonde.config.tcpfeed.radiosondy_active != 0;
    return sonde.config.tcpfeed.rotate_active != 0;
}

static bool aprs_any_feed_enabled() {
    return aprs_feed_enabled(0) || aprs_feed_enabled(1);
}

static float get_local_batt_voltage();
static void aprs_write_line(st_aprs *a, const char *line);

static float get_local_batt_voltage() {
    if(!pmu) {
        return getBattNoPMU();
    }
    return pmu->getBattVoltage() * 0.001f;
}

static void build_comment_with_batt(char *dst, size_t dst_len, const char *base_comment) {
    if (dst_len == 0) return;
    float batt = get_local_batt_voltage();
    if (isnan(batt) || batt <= 0.0f) {
        strlcpy(dst, base_comment ? base_comment : "", dst_len);
        return;
    }
    if (base_comment && base_comment[0]) {
        snprintf(dst, dst_len, "%s Batt: %.2fV", base_comment, batt);
    } else {
        snprintf(dst, dst_len, "Batt: %.2fV", batt);
    }
}

static void aprs_write_line(st_aprs *a, const char *line) {
    if (a->tcpclient_state != TCS_CONNECTED) return;
    char out[APRS_MAXLEN + 3];
    snprintf(out, sizeof(out), "%s\r\n", line);
    write(a->tcpclient, out, strlen(out));
}


void ConnAPRS::init() {
    aprs_gencrctab();

    strncpy(udphost, sonde.config.udpfeed.host, 63);
    char *colon = strchr(udphost, ':');
    if(colon) {
        *colon = 0;
        udpport = atoi(colon+1);
    } else {
        udpport = 9002;
    }
    Serial.printf("AXUDP: host=%s, port=%d\n", udphost, udpport);
    aprs[0].tcpclient = aprs[1].tcpclient = -1;
}

void ConnAPRS::netsetup() {
    // Setup for KISS TCP server
    if(sonde.config.kisstnc.active) {
        MDNS.addService("kiss-tnc", "tcp", 14580);
        tncserver.begin();
    }

    if(aprs_any_feed_enabled()) {
        // start the FSM
        tcpclient_fsm();
    }
}

void ConnAPRS::netshutdown() {
    tncserver.close();
}

void ConnAPRS::updateSonde( SondeInfo *si ) {
    time_last_sonde_rx = millis();
    // prepare data (for UDP and TCP output)
    char *str = aprs_senddata(si, sonde.config.call, sonde.config.objcall, sonde.config.tcpfeed.symbol, APRS_DEST_RADIOSONDY, NULL);

    Serial.printf("udpfedd active: %d  tcpfeed active: %d\n", sonde.config.udpfeed.active, aprs_any_feed_enabled());
    unsigned long now = millis();
    // Output via AXUDP
    if(sonde.config.udpfeed.active) {
	static unsigned long lastudp = 0;
	long tts = sonde.config.udpfeed.ratelimit * 1000L - (now - lastudp);
	Serial.printf("aprs-udp: now-last = %ld\n", (now - lastudp));
	if ( tts < 0 ) {
            char raw[201];
            int rawlen = aprsstr_mon2raw(str, raw, APRS_MAXLEN);
            Serial.println("Sending AXUDP");
            //Serial.println(raw);
            udp.beginPacket(udphost, udpport);
            udp.write((const uint8_t *)raw, rawlen);
            udp.endPacket();
	    lastudp = now;
	} else {
	    Serial.printf("Sending APRS-UDP in %d s\n", (int)(tts/1000));
	}
    }
    // KISS via TCP (incoming connection, e.g. from APRSdroid
    if (tncclient.connected()) {
        Serial.println("Sending position via TCP");
        char raw[201];
        int rawlen = aprsstr_mon2kiss(str, raw, APRS_MAXLEN);
        Serial.print("sending: "); Serial.println(raw);
        tncclient.write(raw, rawlen);
    }
    // APRS via TCP (outgoing connection to aprs-is, e.g. radiosonde.info or wettersonde.net
    if (aprs_any_feed_enabled()) {
        static unsigned long lasttcp_radiosondy = 0;
        static unsigned long lasttcp_rotate = 0;
        tcpclient_fsm();
        if (aprs_feed_enabled(0) && aprs[0].tcpclient_state == TCS_CONNECTED) {
            int radiosondy_rate = sonde.config.tcpfeed.highrate > 0 ? sonde.config.tcpfeed.highrate : 10;
            int radiosondy_fast_rate = sonde.config.tcpfeed.radiosondy_fast_rate;
            int radiosondy_fast_rate_height = sonde.config.tcpfeed.radiosondy_fast_rate_height;
            if (radiosondy_fast_rate > 0 && radiosondy_fast_rate_height > 0 && !isnan(si->d.alt) && si->d.alt < radiosondy_fast_rate_height) {
                radiosondy_rate = radiosondy_fast_rate;
            }
            unsigned long radiosondy_interval = (unsigned long)radiosondy_rate * 1000UL;
            unsigned long elapsed = now - lasttcp_radiosondy;
            if (elapsed >= radiosondy_interval) {
                sendSondeToRadiosondy(si);
                lasttcp_radiosondy = now;
            } else {
                unsigned long tts = radiosondy_interval - elapsed;
                Serial.printf("Sending APRS-radiosondy in %d s\n", (int)(tts / 1000UL));
            }
        }
        if (aprs_feed_enabled(1) && aprs[1].tcpclient_state == TCS_CONNECTED) {
            int rotate_rate = sonde.config.tcpfeed.rotate_highrate > 0 ? sonde.config.tcpfeed.rotate_highrate : 60;
            unsigned long rotate_interval = (unsigned long)rotate_rate * 1000UL;
            unsigned long elapsed = now - lasttcp_rotate;
            if (elapsed >= rotate_interval) {
                sendSondeToRotate(si);
                lasttcp_rotate = now;
            } else {
                unsigned long tts = rotate_interval - elapsed;
                Serial.printf("Sending APRS-rotate in %d s\n", (int)(tts / 1000UL));
            }
        }
    }
}

#define APRS_TIMEOUT 25000

static void check_timeout(st_aprs *a) {
    Serial.printf("Checking APRS timeout: last_in - new: %ld\n", millis() - a->last_in);
    if ( a->last_in && ( (millis() - a->last_in) > sonde.config.tcpfeed.timeout*1000 ) ) {
        Serial.println("APRS timeout - closing connection");
        if(a->tcpclient >= 0) {
            close(a->tcpclient);
            a->tcpclient = -1;
        }
        a->tcpclient_state = TCS_DISCONNECTED;
    }
}

void ConnAPRS::updateStation( PosInfo *pi ) {
    // This funciton is called peridocally.

    // We check for stalled connection and possibly close it
    if ( sonde.config.tcpfeed.timeout > 0) {
        check_timeout(aprs);
        check_timeout(aprs+1);
    }

    // If available, read data from tcpclient; then send update (if its time for that)
    tcpclient_fsm();
    if(aprs_any_feed_enabled()) {
        aprs_station_update();
    }

    // We check for new connections or new data (tnc port) 
    if (!tncclient.connected()) {
        tncclient = tncserver.accept();
        if (tncclient.connected()) {
           Serial.println("new TCP KISS connection");
        }
    }
    if (tncclient.available()) {
        Serial.print("TCP KISS socket: received ");
        while (tncclient.available()) {
           Serial.print(tncclient.read());  // Check if we receive anything from from APRSdroid
        }
        Serial.println("");
    }
}

static void aprs_beacon(char *bcn, st_aprs *aprs) {
  if(aprs->tcpclient_state == TCS_CONNECTED) {
    Serial.printf("APRS TCP BEACON: %s", bcn);
    aprs_write_line(aprs, bcn);
  }
}

void ConnAPRS::sendSondeToRadiosondy(SondeInfo *si) {
    if (!aprs_feed_enabled(0) || aprs[0].tcpclient_state != TCS_CONNECTED) return;
    char *line = aprs_senddata(
        si,
        sonde.config.call,
        sonde.config.objcall,
        sonde.config.tcpfeed.symbol,
        aprs_dest_for(aprs),
        aprs_tail_for(aprs)
    );
    Serial.printf("Sending APRS radiosondy: %s\n", line);
    aprs_write_line(aprs, line);
}

void ConnAPRS::sendSondeToRotate(SondeInfo *si) {
    if (!aprs_feed_enabled(1) || aprs[1].tcpclient_state != TCS_CONNECTED) return;
    char *line = aprs_senddata(
        si,
        sonde.config.call,
        sonde.config.objcall,
        sonde.config.tcpfeed.symbol,
        aprs_dest_for(aprs + 1),
        aprs_tail_for(aprs + 1)
    );
    Serial.printf("Sending APRS rotate: %s\n", line);
    aprs_write_line(aprs + 1, line);
}

void ConnAPRS::sendBeaconToRadiosondy(float lat, float lon, int chase) {
    if (!aprs_feed_enabled(0) || aprs[0].tcpclient_state != TCS_CONNECTED) return;
    char comment_with_batt[96];
    build_comment_with_batt(comment_with_batt, sizeof(comment_with_batt), sonde.config.comment);
    char *bcn = aprs_send_beacon(
        sonde.config.call,
        lat,
        lon,
        sonde.config.beaconsym + ((chase == SH_LOC_CHASE) ? 2 : 0),
        comment_with_batt,
        aprs_dest_for(aprs),
        aprs_tail_for(aprs)
    );
    aprs_beacon(bcn, aprs);
}

void ConnAPRS::sendBeaconToRotate(float lat, float lon, int chase) {
    if (!aprs_feed_enabled(1) || aprs[1].tcpclient_state != TCS_CONNECTED) return;
    // Keep rotate device metadata in a strict format recognized by aprs.fi:
    // "author: device (type)"
    char full_comment[96];
    const char *base_comment = sonde.config.rotate_comment[0] ? sonde.config.rotate_comment : sonde.config.comment;
    const char *author = sonde.config.rotate_author;
    const char *device = sonde.config.rotate_device;
    const char *dtype = sonde.config.rotate_type;
    char device_info[80];
    if (author[0] || device[0] || dtype[0]) {
        if (author[0] && device[0] && dtype[0]) {
            snprintf(device_info, sizeof(device_info), "%s: %s (%s)", author, device, dtype);
        } else if (author[0] && device[0]) {
            snprintf(device_info, sizeof(device_info), "%s: %s", author, device);
        } else if (author[0] && dtype[0]) {
            snprintf(device_info, sizeof(device_info), "%s (%s)", author, dtype);
        } else if (device[0] && dtype[0]) {
            snprintf(device_info, sizeof(device_info), "%s (%s)", device, dtype);
        } else if (author[0]) {
            strlcpy(device_info, author, sizeof(device_info));
        } else if (device[0]) {
            strlcpy(device_info, device, sizeof(device_info));
        } else {
            strlcpy(device_info, dtype, sizeof(device_info));
        }
    } else {
        device_info[0] = 0;
    }

    if (device_info[0]) {
        strlcpy(full_comment, device_info, sizeof(full_comment));
    } else if (base_comment[0]) {
        strlcpy(full_comment, base_comment, sizeof(full_comment));
    } else {
        full_comment[0] = 0;
    }

    // Do not append battery info on rotate beacon; keep device string parseable.
    char *bcn = aprs_send_beacon(
        sonde.config.call,
        lat,
        lon,
        sonde.config.beaconsym + ((chase == SH_LOC_CHASE) ? 2 : 0),
        full_comment,
        aprs_dest_for(aprs + 1),
        aprs_tail_for(aprs + 1)
    );
    aprs_beacon(bcn, aprs + 1);
}

void ConnAPRS::aprs_station_update() {
  int chase = sonde.config.chase;
  if (chase == SH_LOC_OFF) // do not send any location
    return;
  // automatically decided if CHASE or FIXED mode is used (for config AUTO)
  if (chase == SH_LOC_AUTO) {
    if (posInfo.chase) chase = SH_LOC_CHASE; else chase = SH_LOC_FIXED;
  }
  unsigned long time_now = millis();
  unsigned long time_delta = time_now - time_last_aprs_update;
  unsigned long update_time = (chase == SH_LOC_CHASE) ? APRS_MOBILE_STATION_UPDATE_TIME : APRS_STATION_UPDATE_TIME;
  long tts = update_time - time_delta;
  Serial.printf("aprs_station_update due in %d s", (int)(tts/1000));
  if (tts>0) return;

  float lat, lon;
  if (chase == SH_LOC_FIXED) {
    // fixed location
    lat = sonde.config.rxlat;
    lon = sonde.config.rxlon;
    if (isnan(lat) || isnan(lon)) return;
  } else {
    if (gpsPos.valid) {
      lat = gpsPos.lat;
      lon = gpsPos.lon;
    } else {
      return;
    }
  }
  tcpclient_fsm();
    // Beacon scheduling is independent from sonde RX traffic: this path is driven by updateStation().
    sendBeaconToRadiosondy(lat, lon, chase);
    unsigned long rotate_interval = (unsigned long)sonde.config.tcpfeed.rotate_beacon_interval * 60000UL;
    bool rotate_smart = sonde.config.tcpfeed.smart_beacon_rotate != 0;
    bool sonde_active = rotate_smart ? (time_last_sonde_rx > 0 && (time_now - time_last_sonde_rx) < rotate_interval) : true;
    if (sonde_active && (time_now - time_last_rotate_beacon) >= rotate_interval) {
      sendBeaconToRotate(lat, lon, chase);
      time_last_rotate_beacon = time_now;
    }
  time_last_aprs_update = time_now;
}

static void _tcp_dns_found(const char * name, const ip_addr_t *ipaddr, void * arg) {
    st_aprs *a = (st_aprs *)arg;
    if (ipaddr) {
        a->tcpclient_ipaddr = *ipaddr;
        a->tcpclient_state = TCS_DNSRESOLVED;    // DNS lookup success
    } else {
        memset(&a->tcpclient_ipaddr, 0, sizeof(a->tcpclient_ipaddr));
        a->tcpclient_state = TCS_DISCONNECTED;   // DNS lookup failed
    }
}

void tcpclient_sendlogin(st_aprs *a) {
    char buf[128];
    const char *agent = (aprs_index(a) == 0) ? aprs_signature() : "rdzttgo";
    a->conn_ts = esp_timer_get_time() / 1000000;
    snprintf(buf, 128, "user %s pass %d vers %s %s\r\n", sonde.config.call, sonde.config.passcode, agent, version_id);
    int res = write(a->tcpclient, buf, strlen(buf));
    Serial.printf("APRS login: %s, res=%d\n", buf, res);
    a->last_in = millis();
    if(res<=0) {
        if( a->tcpclient >= 0 ) close(a->tcpclient);
        a->tcpclient = -1;
        a->tcpclient_state = TCS_DISCONNECTED;
    }
}

static void tcpclient_fsm_single(st_aprs *a);

void tcpclient_fsm() {
    for(int i=0; i<N_APRS; i++) {
        tcpclient_fsm_single(aprs+i);
    }
}
 
static void tcpclient_fsm_single(st_aprs *a) {
    int idx = aprs_index(a);
    if(!aprs_feed_enabled(idx)) {
        if (a->tcpclient >= 0) {
            close(a->tcpclient);
        }
        a->tcpclient = -1;
        a->tcpclient_state = TCS_DISCONNECTED;
        return;
    }

    Serial.printf("TCS[%d]: %d\n", idx, a->tcpclient_state);

    struct timeval selto = {0};
    int res;

    switch(a->tcpclient_state) {
    case TCS_DISCONNECTED: 
        /* We are disconnected. Try to connect, starting with a DNS lookup */
      {
        // Restart timeout
        a->last_in = millis();
        char host[256];
                strlcpy(host, aprs_effective_host(a), sizeof(host));
        char *colon =strchr(host, ':');
        if(colon) {
            *colon = 0;
            a->port = atoi(colon+1);
        } else {
	    a->port = 14580;
        }
                if (idx == 1 && host[0] == 0) {
                        a->tcpclient_state = TCS_DISCONNECTED;
                        break;
                }
                Serial.printf("aprs %d: host is '%s', port %d\n", idx, host, a->port);
        err_t res = dns_gethostbyname( host, &a->tcpclient_ipaddr, /*(dns_found_callback)*/_tcp_dns_found, a );

        if(res == ERR_OK) {   // Returns immediately of host is IP or in cache
            a->tcpclient_state = TCS_DNSRESOLVED;
            /* fall through */
        } else if(res == ERR_INPROGRESS) {
            a->tcpclient_state = TCS_DNSLOOKUP;
            break;
        } else {  // failed
            a->tcpclient_state = TCS_DISCONNECTED;
            break;
        }
      }

    case TCS_DNSRESOLVED:
      {
        /* We have got the IP address, start the connection */
        a->tcpclient = socket(AF_INET, SOCK_STREAM, 0);
        int flags = fcntl(a->tcpclient, F_GETFL);
        if (fcntl(a->tcpclient, F_SETFL, flags | O_NONBLOCK) == -1) {
            Serial.println("Setting O_NONBLOCK failed");
        }

        struct sockaddr_in sock_info;
        memset(&sock_info, 0, sizeof(struct sockaddr_in));
        sock_info.sin_family = AF_INET;
        sock_info.sin_addr.s_addr = a->tcpclient_ipaddr.u_addr.ip4.addr;
        sock_info.sin_port = htons( a->port );
        err_t res = connect(a->tcpclient, (struct sockaddr *)&sock_info, sizeof(sock_info));
        if(res) {
            if (errno == EINPROGRESS) { // Should be the usual case, go to connecting state
                a->tcpclient_state = TCS_CONNECTING;
            } else {
                close(a->tcpclient);
                a->tcpclient = -1;
                a->tcpclient_state = TCS_DISCONNECTED;
            }
        } else {
            a->tcpclient_state = TCS_CONNECTED;
            tcpclient_sendlogin(a);
        }
      }
      break;
    case TCS_CONNECTING: 
      {
                fd_set fdset;
                FD_ZERO(&fdset);
                FD_SET(a->tcpclient, &fdset);
                fd_set fdeset;
                FD_ZERO(&fdeset);
                FD_SET(a->tcpclient, &fdeset);
        // Poll to see if we are now connected 
        res = select(a->tcpclient+1, NULL, &fdset, &fdeset, &selto);
        if(res<0) {
            Serial.println("TCS_CONNECTING: select error");
            goto error;
        } else if (res==0) { // still pending
            break;
        }
        // Socket has become ready (or something went wrong, check for error first)
        
        int sockerr;
        socklen_t len = (socklen_t)sizeof(int);
        if (getsockopt(a->tcpclient, SOL_SOCKET, SO_ERROR, (void*)(&sockerr), &len) < 0) {
            goto error;
        }
        Serial.printf("select returning %d. isset:%d iseset:%d sockerr:%d\n", res, FD_ISSET(a->tcpclient, &fdset), FD_ISSET(a->tcpclient, &fdeset), sockerr);
        if(sockerr) {
            Serial.printf("APRS connect error: %s\n", strerror(sockerr));
            goto error;
        }
        a->tcpclient_state = TCS_CONNECTED;
        tcpclient_sendlogin(a);
      }
      break;
        
    case TCS_CONNECTED:
      {
                fd_set fdset;
                FD_ZERO(&fdset);
                FD_SET(a->tcpclient, &fdset);
        res = select(a->tcpclient+1, &fdset, NULL, NULL, &selto);
        if(res<0) {
            Serial.println("TCS_CONNECTED: select error");
            goto error;
        } else if (res==0) { // still pending
            break;
        }
        // Read data
        char buf[512+1];
        res = read(a->tcpclient, buf, 512);
        if(res<=0) {
            close(a->tcpclient);
            a->tcpclient = -1;
            a->tcpclient_state = TCS_DISCONNECTED;
        } else {
            buf[res] = 0;
            Serial.printf("tcpclient data (len=%d):", res);
            Serial.write( (uint8_t *)buf, res );
            a->last_in = millis();    
        }
      }
      break;

    case TCS_DNSLOOKUP:
        Serial.println("DNS lookup in progress");
        break;   // DNS lookup in progress, do not do anything until callback is called, updating the state
    }
    return;

error:
    if(a->tcpclient >= 0) close(a->tcpclient);
    a->tcpclient = -1;
    a->tcpclient_state = TCS_DISCONNECTED;
    return;
}

const char *aprsstate2str(int state) {
  switch(state) {
  case TCS_DISCONNECTED: return "disconnected";
  case TCS_DNSLOOKUP: return "DNS lookup";
  case TCS_DNSRESOLVED: return "DNS resolved";
  case TCS_CONNECTING: return "connecting";
  case TCS_LOGIN: return "login";
  case TCS_CONNECTED: return "connected";
  default: return "??";
  }
}

String ConnAPRS::getStatus() {
    char buf[1024];
    // AXUDP: enabled or disabled
    strlcpy(buf, sonde.config.udpfeed.active ? "AXUDP enabled<br>":"AXUDP disabled<br>", 1024);
    // KISS TNC: disabled, enabled(idle), enabled(client connected)
    if(sonde.config.kisstnc.active==0) strlcat(buf, "KISS TNC: disabled<br>", 1024);
    else if (tncclient.connected()) strlcat(buf, "KISS TNC: server active, client connected<br>", 1024);
    else strlcat(buf, "KISS TNC: server active, idle<br>", 1024 );
    // APRS client
    if(!aprs_any_feed_enabled()) strlcat(buf, "APRS: disabled", 1024);
    else {
        snprintf( buf+strlen(buf), 1024-strlen(buf), "APRS: %s [%s] (%s)", aprsstate2str(aprs[0].tcpclient_state), aprs_effective_host(aprs), aprs_feed_enabled(0) ? "ON" : "OFF");
        uint32_t uptime = esp_timer_get_time() / 1000000;
        Serial.printf("up %d c1 %d c2%d\n", uptime, aprs[0].conn_ts, aprs[1].conn_ts);
        if(aprs[0].tcpclient_state == TCS_CONNECTED) {
            strlcat(buf, ", up: ", 1024);
            appendUptime(buf, 1024, uptime - aprs[0].conn_ts);
        }
        snprintf( buf+strlen(buf), 1024-strlen(buf), "<br>APRS2: %s [%s] (%s)", aprsstate2str(aprs[1].tcpclient_state), aprs_effective_host(aprs+1), aprs_feed_enabled(1) ? "ON" : "OFF");
        if(aprs[1].tcpclient_state == TCS_CONNECTED) {
            strlcat(buf, ", up: ", 1024);
            appendUptime(buf, 1024, uptime - aprs[1].conn_ts);
        }
    }
    return String(buf);
}

String ConnAPRS::getName() {
    return String("APRS");
}

ConnAPRS connAPRS;

#endif
