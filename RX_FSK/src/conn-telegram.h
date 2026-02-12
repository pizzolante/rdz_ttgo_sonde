#include "../features.h"
#if FEATURE_TELEGRAM

#ifndef conn_telegram_h
#define conn_telegram_h

#include "conn.h"
#include "Sonde.h"

class ConnTelegram : public Conn
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

	bool sendLastFrameTest();

private:
	// Track sonde states for notifications
	struct SondeState {
		char id[16];
		bool notified_new;
		bool notified_burst;
		bool notified_end;
		uint32_t last_update;
		float last_vs;
		bool is_descending;
	};

	static const int MAX_TRACKED_SONDES = 10;
	SondeState tracked_sondes[MAX_TRACKED_SONDES];
	int num_tracked = 0;

	// Helper functions
	bool sendTelegramMessage(const char *message);
	SondeState* findSondeState(const char *id);
	SondeState* getOrCreateSondeState(const char *id);
	void checkBurst(SondeInfo *si, SondeState *state);
	void checkEnd();
	String formatSondeMessage(SondeInfo *si);
};

extern ConnTelegram connTelegram;
#endif

#endif
