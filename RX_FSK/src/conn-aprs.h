#include "../features.h"
#if FEATURE_APRS

#ifndef conn_aprs_h
#define conn_aprs_h

#include "conn.h"
#include "aprs.h"

// Times in ms, i.e. station: 10 minutes, mobile: 20 seconds
#define APRS_STATION_UPDATE_TIME (10*60*1000)
#define APRS_MOBILE_STATION_UPDATE_TIME (20*1000)

static unsigned long time_last_aprs_update = -APRS_STATION_UPDATE_TIME;
static unsigned long time_last_rotate_beacon = -APRS_STATION_UPDATE_TIME;
static unsigned long time_last_sonde_rx = 0;
static char last_sonde_rx_id[10] = "";
static uint32_t last_sonde_rx_vframe = 0xffffffffUL;
static char last_rotate_beacon_sonde_id[10] = "";
static char last_rotate_data_sonde_id[10] = "";
static char last_rotate_detail_sonde_id[10] = "";


class ConnAPRS : public Conn
{
public:
        /* Called once on startup */
        void init();

        /* Called whenever the network becomes available */
        void netsetup();

	/* Close connections */
	void netshutdown();

        /* Called approx 1x / second (maybe only if good data is available) */
        void updateSonde( SondeInfo *si );

        /* Called approx 1x / second* */
        void updateStation( PosInfo *pi );

	String getStatus();

	String getName();

private:
	void aprs_station_update();
        void sendSondeToRadiosondy(SondeInfo *si);
        void sendSondeToRotate(SondeInfo *si);
        void sendDetailToRotate(SondeInfo *si);
        void sendBeaconToRadiosondy(float lat, float lon, int chase);
        void sendBeaconToRotate(float lat, float lon, int chase);
};

extern ConnAPRS connAPRS;
#endif

#endif
