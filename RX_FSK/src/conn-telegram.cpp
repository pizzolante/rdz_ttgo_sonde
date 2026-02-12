#include "../features.h"
#if FEATURE_TELEGRAM

#define TAG "conn-telegram"
#include "logger.h"

#include "../core.h"

#include <Arduino.h>
#include "conn-telegram.h"
#include <WiFi.h>
#include <esp_http_client.h>

extern const char *version_name;
extern const char *version_id;

ConnTelegram connTelegram;

// Telegram API endpoint
const char* TELEGRAM_API_HOST = "api.telegram.org";
const int TELEGRAM_API_PORT = 443;

/* Global initialization (on TTGO startup) */
void ConnTelegram::init() {
	num_tracked = 0;
	memset(tracked_sondes, 0, sizeof(tracked_sondes));
	LOG_I(TAG, "Telegram connector initialized");
}

/* Network initialization (as soon as network becomes available) */
void ConnTelegram::netsetup() {
	if (sonde.config.telegram.active == 0) {
		LOG_I(TAG, "Telegram disabled in config");
		return;
	}

	if (strlen(sonde.config.telegram.token) == 0) {
		LOG_W(TAG, "Telegram token not configured");
		return;
	}

	if (strlen(sonde.config.telegram.chat_id) == 0) {
		LOG_W(TAG, "Telegram chat_id not configured");
		return;
	}

	// Set default end_delay if not configured
	if (sonde.config.telegram.end_delay < 60) {
		sonde.config.telegram.end_delay = 300; // 5 minutes default
	}

	LOG_I(TAG, "Telegram connector ready - Bot notifications enabled");
}

void ConnTelegram::netshutdown() {
	LOG_I(TAG, "Telegram connector shutdown");
}

void ConnTelegram::updateSonde(SondeInfo *si) {
	if (sonde.config.telegram.active == 0) return;
	if (strlen(sonde.config.telegram.token) == 0) return;
	if (strlen(sonde.config.telegram.chat_id) == 0) return;

	SondeData *s = &(si->d);

	// Check if we have valid data
	if (isnan(s->lat) || isnan(s->lon)) return;

	// Get or create state for this sonde
	SondeState *state = getOrCreateSondeState(s->id);
	if (state == NULL) return;

	// Update last seen timestamp
	state->last_update = millis();

	// Check for new sonde notification
	if (sonde.config.telegram.notify_new && !state->notified_new) {
		String message = "🎈 *New Radiosonde Detected*\n\n";
		message += formatSondeMessage(si);
		
		if (sendTelegramMessage(message.c_str())) {
			state->notified_new = true;
			LOG_I(TAG, "New sonde notification sent: %s", s->id);
		}
	}

	// Check for burst detection
	if (sonde.config.telegram.notify_burst && state->notified_new) {
		checkBurst(si, state);
	}

	// Store vertical speed for burst detection
	state->last_vs = s->vs;
}

void ConnTelegram::updateStation(PosInfo *pi) {
	// Check for end notifications (sondes not seen for a while)
	if (sonde.config.telegram.notify_end) {
		checkEnd();
	}
}

// Helper function to send message via Telegram Bot API using esp_http_client
bool ConnTelegram::sendTelegramMessage(const char *message) {
	if (WiFi.status() != WL_CONNECTED) {
		LOG_W(TAG, "WiFi not connected");
		return false;
	}

	// Build API URL
	String url = "https://api.telegram.org/bot";
	url += sonde.config.telegram.token;
	url += "/sendMessage";

	// Escape special characters in message for JSON
	String escapedMessage = message;
	escapedMessage.replace("\\", "\\\\");
	escapedMessage.replace("\"", "\\\"");
	escapedMessage.replace("\n", "\\n");
	
	// Create JSON payload
	String payload = "{\"chat_id\":\"";
	payload += sonde.config.telegram.chat_id;
	payload += "\",\"text\":\"";
	payload += escapedMessage;
	payload += "\",\"parse_mode\":\"Markdown\"}";

	LOG_D(TAG, "Sending request to Telegram API");

	// Configure HTTP client
	esp_http_client_config_t config = {};
	config.url = url.c_str();
	config.method = HTTP_METHOD_POST;
	config.timeout_ms = 10000;
	config.skip_cert_common_name_check = true;
	
	esp_http_client_handle_t client = esp_http_client_init(&config);
	if (client == NULL) {
		LOG_E(TAG, "Failed to initialize HTTP client");
		return false;
	}

	// Set headers
	esp_http_client_set_header(client, "Content-Type", "application/json");
	esp_http_client_set_post_field(client, payload.c_str(), payload.length());

	// Perform HTTP request
	esp_err_t err = esp_http_client_perform(client);
	
	bool success = false;
	if (err == ESP_OK) {
		int status_code = esp_http_client_get_status_code(client);
		LOG_D(TAG, "HTTP Status Code: %d", status_code);
		
		if (status_code == 200) {
			int content_length = esp_http_client_get_content_length(client);
			if (content_length > 0 && content_length < 2048) {
				char *buffer = (char *)malloc(content_length + 1);
				if (buffer != NULL) {
					int read_len = esp_http_client_read(client, buffer, content_length);
					if (read_len > 0) {
						buffer[read_len] = 0;
						LOG_D(TAG, "Response: %s", buffer);
						
						if (strstr(buffer, "\"ok\":true") != NULL) {
							LOG_I(TAG, "Telegram message sent successfully");
							success = true;
						}
					}
					free(buffer);
				}
			}
		} else {
			LOG_E(TAG, "HTTP error code: %d", status_code);
		}
	} else {
		LOG_E(TAG, "HTTP request failed: %s", esp_err_to_name(err));
	}

	esp_http_client_cleanup(client);
	return success;
}

// Find existing sonde state by ID
ConnTelegram::SondeState* ConnTelegram::findSondeState(const char *id) {
	for (int i = 0; i < num_tracked; i++) {
		if (strcmp(tracked_sondes[i].id, id) == 0) {
			return &tracked_sondes[i];
		}
	}
	return NULL;
}

// Get or create sonde state
ConnTelegram::SondeState* ConnTelegram::getOrCreateSondeState(const char *id) {
	SondeState *state = findSondeState(id);
	if (state != NULL) return state;

	// Create new state if we have space
	if (num_tracked < MAX_TRACKED_SONDES) {
		state = &tracked_sondes[num_tracked];
		strncpy(state->id, id, sizeof(state->id) - 1);
		state->id[sizeof(state->id) - 1] = 0;
		state->notified_new = false;
		state->notified_burst = false;
		state->notified_end = false;
		state->last_update = millis();
		state->last_vs = 0;
		state->is_descending = false;
		num_tracked++;
		LOG_I(TAG, "Tracking new sonde: %s (total: %d)", id, num_tracked);
		return state;
	}

	// No space, find oldest entry and reuse it
	uint32_t oldest_time = tracked_sondes[0].last_update;
	int oldest_idx = 0;
	for (int i = 1; i < MAX_TRACKED_SONDES; i++) {
		if (tracked_sondes[i].last_update < oldest_time) {
			oldest_time = tracked_sondes[i].last_update;
			oldest_idx = i;
		}
	}

	LOG_W(TAG, "Replacing old sonde %s with %s", tracked_sondes[oldest_idx].id, id);
	state = &tracked_sondes[oldest_idx];
	strncpy(state->id, id, sizeof(state->id) - 1);
	state->id[sizeof(state->id) - 1] = 0;
	state->notified_new = false;
	state->notified_burst = false;
	state->notified_end = false;
	state->last_update = millis();
	state->last_vs = 0;
	state->is_descending = false;

	return state;
}

// Check for burst (balloon pop)
void ConnTelegram::checkBurst(SondeInfo *si, SondeState *state) {
	if (state->notified_burst) return;

	SondeData *s = &(si->d);

	// Detect burst: vertical speed changes from positive (ascending) to negative (descending)
	// Require at least 2 consecutive measurements descending
	if (s->vs < -1.0) {  // Descending faster than 1 m/s
		if (!state->is_descending && state->last_vs > 1.0) {
			// Was ascending, now descending - potential burst!
			state->is_descending = true;
		} else if (state->is_descending) {
			// Second measurement confirms burst
			String message = "💥 *Balloon Burst Detected!*\n\n";
			message += formatSondeMessage(si);
			message += "\n\n_Flight phase: Descent_";

			if (sendTelegramMessage(message.c_str())) {
				state->notified_burst = true;
				LOG_I(TAG, "Burst notification sent: %s", s->id);
			}
		}
	} else if (s->vs > 1.0) {
		state->is_descending = false;
	}
}

// Check for end notifications (sondes not seen for end_delay seconds)
void ConnTelegram::checkEnd() {
	uint32_t now = millis();
	uint32_t timeout = sonde.config.telegram.end_delay * 1000;

	for (int i = 0; i < num_tracked; i++) {
		SondeState *state = &tracked_sondes[i];
		
		// Skip if end notification already sent
		if (state->notified_end) continue;
		
		// Skip if new notification wasn't sent (probably just tracking)
		if (!state->notified_new) continue;

		// Check if timeout expired
		if ((now - state->last_update) > timeout) {
			String message = "🛬 *Radiosonde Flight Ended*\n\n";
			message += "*ID:* `";
			message += state->id;
			message += "`\n";
			message += "*Status:* Signal lost for ";
			message += String(sonde.config.telegram.end_delay);
			message += " seconds\n\n";
			message += "_Flight tracking ended_";

			if (sendTelegramMessage(message.c_str())) {
				state->notified_end = true;
				LOG_I(TAG, "End notification sent: %s", state->id);
			}
		}
	}
}

// Format sonde information message
String ConnTelegram::formatSondeMessage(SondeInfo *si) {
	SondeData *s = &(si->d);
	String message = "";

	// Sonde type and ID
	message += "*Type:* ";
	message += sondeTypeStr[si->type];
	message += "\n*ID:* `";
	message += s->id;
	message += "`\n";

	// Frequency
	message += "*Frequency:* ";
	message += String(si->freq, 3);
	message += " MHz\n\n";

	// Position
	message += "*Position:*\n";
	message += "├ Lat: ";
	message += String(s->lat, 6);
	message += "°\n";
	message += "├ Lon: ";
	message += String(s->lon, 6);
	message += "°\n";
	message += "└ Alt: ";
	message += String((int)s->alt);
	message += " m\n\n";

	// Velocity
	if (s->hs > 0.5) {
		message += "*Velocity:*\n";
		message += "├ Speed: ";
		message += String(s->hs, 1);
		message += " m/s (";
		message += String(s->hs * 3.6, 1);
		message += " km/h)\n";
		message += "├ Direction: ";
		message += String((int)s->dir);
		message += "°\n";
		message += "└ Vertical: ";
		message += String(s->vs, 1);
		message += " m/s\n\n";
	}

	// Meteorological data
	bool hasMet = false;
	message += "*Meteorology:*\n";
	
	if (!isnan(s->temperature)) {
		message += "├ Temp: ";
		message += String(s->temperature, 1);
		message += " °C\n";
		hasMet = true;
	}
	
	if (!isnan(s->pressure)) {
		message += "├ Press: ";
		message += String(s->pressure, 1);
		message += " hPa\n";
		hasMet = true;
	}
	
	if (!isnan(s->relativeHumidity)) {
		message += "└ Humid: ";
		message += String(s->relativeHumidity, 1);
		message += " %\n";
		hasMet = true;
	}
	
	if (!hasMet) {
		message += "└ _No data_\n";
	}
	message += "\n";

	// Google Maps link
	message += "[📍 View on Google Maps](https://maps.google.com/?q=";
	message += String(s->lat, 6);
	message += ",";
	message += String(s->lon, 6);
	message += ")";

	return message;
}

String ConnTelegram::getStatus() {
	if (sonde.config.telegram.active == 0) {
		return "Telegram: disabled";
	}

	if (strlen(sonde.config.telegram.token) == 0 || strlen(sonde.config.telegram.chat_id) == 0) {
		return "Telegram: not configured";
	}

	String status = "Telegram: active (tracking ";
	status += String(num_tracked);
	status += " sonde";
	if (num_tracked != 1) status += "s";
	status += ")";

	return status;
}

String ConnTelegram::getName() {
	return "Telegram";
}

static bool hasValidFrame(const SondeInfo *si) {
	const SondeData *s = &(si->d);
	if (s->id[0] == 0) return false;
	if (isnan(s->lat) || isnan(s->lon)) return false;
	return true;
}

bool ConnTelegram::sendLastFrameTest() {
	if (sonde.config.telegram.active == 0) {
		LOG_W(TAG, "Telegram disabled in config");
		return false;
	}
	if (strlen(sonde.config.telegram.token) == 0 || strlen(sonde.config.telegram.chat_id) == 0) {
		LOG_W(TAG, "Telegram not configured");
		return false;
	}

	SondeInfo *best = NULL;
	uint32_t best_rxstart = 0;

	if (rxtask.receiveSonde >= 0 && rxtask.receiveSonde < sonde.config.maxsonde) {
		SondeInfo *candidate = &sonde.sondeList[rxtask.receiveSonde];
		if (hasValidFrame(candidate)) {
			best = candidate;
			best_rxstart = candidate->rxStart;
		}
	}

	for (int i = 0; i < sonde.config.maxsonde; i++) {
		SondeInfo *candidate = &sonde.sondeList[i];
		if (!hasValidFrame(candidate)) continue;
		if (best == NULL || candidate->rxStart > best_rxstart) {
			best = candidate;
			best_rxstart = candidate->rxStart;
		}
	}

	if (best == NULL) {
		return sendTelegramMessage("Telegram test: no sonde frames received yet.");
	}

	String message = "Telegram test - last received frame\n\n";
	message += formatSondeMessage(best);
	return sendTelegramMessage(message.c_str());
}

#endif
