/*
 * modem_simple.c - the EC200U path from all_hw_test, verbatim.
 *
 * WHY THIS FILE EXISTS
 *
 * all_hw_test brings this modem up on this board, attaches to AIRTEL, opens the
 * broker and publishes - repeatedly and reliably. ec200.c, on the same board and
 * the same pins, has never received a single byte from the module. Many attempts
 * to find the difference by inspection failed, and several "fixes" derived from
 * that reasoning turned out to be regressions.
 *
 * So this stops paraphrasing the working program and copies it: same polled I/O,
 * same pin order, same delays, same AT strings, same retry counts. Where the two
 * environments genuinely differ, that difference is handled explicitly rather
 * than assumed away:
 *
 *   RX INTERRUPT. all_hw_test sets CONFIG_UART_INTERRUPT_DRIVEN=n and polls.
 *   This application enables the ISR, which drains the FIFO into a ring buffer
 *   the moment a byte lands - so a polled reader copied out of the test finds
 *   the FIFO permanently empty. Every routine here therefore runs with the RX
 *   interrupt DISABLED and restores it on the way out.
 *
 *   WATCHDOG. all_hw_test has none. This application has a 120 s one, and the
 *   long waits below (20 s boot, 90 s attach) will trip it unless fed. Every
 *   loop calls gw_wdt_alive().
 *
 * ec200.c is left in place for OTA/HTTP and the downlink URC handling. This file
 * owns only bring-up, attach and publish - the part that has never worked.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdio.h>

#include "modem_simple.h"
#include "app_config.h"
#include "config.h"
#include "telemetry.h"   /* telem_epoch() */
#include "power.h"
#include "watchdog.h"
#include "board_pins.h"

#define MS_UART DT_NODELABEL(uart22)

static const struct device *const uart = DEVICE_DT_GET(MS_UART);

/* Set once the broker session is up, so later reports skip straight to publish.
 * Cleared whenever any step fails, which forces a full re-attach next time. */
static bool s_connected;

/*
 * Last measured signal and serving cell.
 *
 * Sampled once per session in ms_connect(), which is the only moment the modem
 * is guaranteed attached and awake. The heartbeat reads them through the
 * getters below - previously it printed the literals "0" and -71, which looked
 * exactly like a healthy link no matter what the radio was doing.
 *
 * s_rssi_dbm holds 0 for "unknown", which is not a legal RSSI so it cannot be
 * confused with a reading. The cell string is empty when unknown for the same
 * reason: "0" was indistinguishable from a real cell id of zero.
 */
static int  s_rssi_dbm;
static char s_cell[24];

/* ---------------------------------------------------------------- polled I/O */

/*
 * Read for `ms`, returning whatever arrived. Polled, no ring buffer - valid only
 * while the RX interrupt is disabled, which every caller here guarantees.
 */
static int ms_collect(char *b, size_t n, int ms)
{
	int64_t end = k_uptime_get() + ms;
	size_t i = 0;
	unsigned char c;

	while (k_uptime_get() < end && i < n - 1) {
		if (uart_poll_in(uart, &c) == 0) {
			b[i++] = (char)c;
		}
	}
	b[i] = '\0';
	return (int)i;
}

static void ms_send(const char *s)
{
	for (const char *p = s; *p; p++) {
		uart_poll_out(uart, (unsigned char)*p);
	}
	uart_poll_out(uart, '\r');
	uart_poll_out(uart, '\n');
}

/*
 * Send one command and wait for `want` to appear. Returns true on match.
 *
 * Bails early on "ERROR" so a rejected command does not burn its whole timeout -
 * the attach loop issues this dozens of times and the difference is minutes.
 */
static bool ms_at(const char *cmd, const char *want, int ms)
{
	static char buf[512];
	size_t n = 0;
	int64_t end;

	(void)ms_collect(buf, sizeof(buf), 50);      /* flush */
	printk("MS> %s\n", cmd);
	ms_send(cmd);

	end = k_uptime_get() + ms;
	while (k_uptime_get() < end && n < sizeof(buf) - 1) {
		unsigned char c;

		if (uart_poll_in(uart, &c) == 0) {
			buf[n++] = (char)c;
			buf[n] = '\0';
			if (strstr(buf, want)) {
				return true;
			}
			if (strstr(buf, "ERROR")) {
				break;
			}
		}
		if ((n & 0x3F) == 0) {
			gw_wdt_alive();
		}
	}

	for (size_t i = 0; i < n; i++) {
		if (buf[i] == '\r' || buf[i] == '\n') {
			buf[i] = ' ';
		}
	}
	buf[n] = '\0';
	printk("MS< %s\n", n ? buf : "(silence)");
	gw_wdt_alive();
	return false;
}

/*
 * Send a command and print whatever comes back, matching nothing.
 *
 * ms_at() answers "did this token appear", which is the wrong question for a
 * diagnostic: the VALUE is the whole point. +CSQ: 99,99 (no signal) and
 * +CSQ: 18,99 (good signal) both fail a match on "+CGATT: 1" identically, and
 * they mean completely different things.
 */
static void ms_show(const char *cmd, int ms)
{
	static char buf[256];
	int n;

	(void)ms_collect(buf, sizeof(buf), 50);      /* flush */
	ms_send(cmd);
	n = ms_collect(buf, sizeof(buf), ms);

	for (int i = 0; i < n; i++) {
		if (buf[i] == '\r' || buf[i] == '\n') {
			buf[i] = ' ';
		}
	}
	buf[n > 0 ? n : 0] = '\0';
	printk("MS?  %-12s -> %s\n", cmd, n ? buf : "(silence)");
	gw_wdt_alive();
}

/* all_hw_test's at_ping(): flush, send AT, look for OK within a second. */
static bool ms_ping(int tries)
{
	static char buf[256];

	for (int i = 0; i < tries; i++) {
		(void)ms_collect(buf, sizeof(buf), 50);
		ms_send("AT");
		if (ms_collect(buf, sizeof(buf), 1000) > 0 && strstr(buf, "OK")) {
			printk("MS< AT -> OK\n");
			return true;
		}
		gw_wdt_alive();
	}
	return false;
}

/* ------------------------------------------------------------------ sequence */

/*
 * Bring the module up. Pulses PWRKEY only when it is genuinely silent - PWRKEY
 * is a TOGGLE, and pulsing a running module powers it down.
 */
static bool ms_bringup(void)
{
	static char buf[512];

	if (ms_ping(2)) {
		printk("MS: modem already up\n");
		return true;
	}

	printk("MS: silent - pulsing PWRKEY\n");
	modem_pwrkey_pulse();                        /* 600 ms + fed 20 s wait */

	/* Drain the boot URCs (RDY, +CPIN: READY ...) so the first AT starts on a
	 * clean line rather than mid-URC. This is the test's 5 s collect. */
	int n = ms_collect(buf, sizeof(buf), 5000);

	if (n > 0) {
		printk("MS: boot text, %d bytes\n", n);
	}
	gw_wdt_alive();

	return ms_ping(8);
}

/* Attach, activate a bearer, connect to the broker. Leaves s_connected set. */
static bool ms_connect(void)
{
	static char cmd[192];

	/*
	 * PERSISTENT SESSION - Rev.1 s7.3, and the mechanism behind Class A.
	 *
	 * clean_session = false is what lets the broker hold queued QoS-1
	 * commands on dn/cmd while the gateway sleeps. Without it every wake
	 * opened a CLEAN session and the broker discarded anything queued since
	 * the last one - so a command sent while the modem was powered down was
	 * simply lost, with no error anywhere. The gateway is asleep ~99 % of the
	 * time at the production cadence, so that was almost every command.
	 *
	 * ec200.c has always set this; modem_simple, which is the path that
	 * actually connects on this board, did not.
	 */
	if (!ms_at("AT+QMTCFG=\"session\",0,0", "OK", 3000)) {
		printk("MS: WARNING - persistent session not set; the broker will "
		       "DROP commands queued while asleep\n");
	}

	/*
	 * Last Will, configured BEFORE QMTCONN because the broker records it at
	 * connect time. Without it an ungraceful loss - brownout, watchdog,
	 * cellular drop - leaves the retained status reading "online" forever for
	 * a gateway that is gone. The graceful path publishes "offline" itself in
	 * link_down(); this covers every other way a session can end.
	 *
	 * The payload is the s7.3 object, not a bare string. Its `ts` is the
	 * CONNECT time, because a will is composed now and delivered by the broker
	 * later - the gateway cannot stamp a message it never sends. Read it as
	 * "last known good", not as the moment of failure.
	 *
	 * Quoting: the AT argument is double-quoted and so is the JSON, so the
	 * inner quotes are escaped as \" for the module. Not every Quectel build
	 * accepts that, and a REJECTED will leaves no will at all - which is worse
	 * than a non-conformant one - so a bare-string will is used as a fallback
	 * and the outcome is logged.
	 */
	{
		static char will[256];
		static char topic[128];
		unsigned ts = (unsigned)telem_epoch();
		bool ok = false;

		snprintf(topic, sizeof(topic), "%s/%s/%s/status",
			 TOPIC_ROOT, cfg_wagon(), cfg_gw_id());

		/*
		 * Try each quoting style the Quectel AT parser might accept, most
		 * conformant first. The module is the authority on which works, and
		 * it answers in one command - far better than picking one from a
		 * datasheet and hoping.
		 *
		 *   1. backslash-escaped  \"  - the common Quectel convention
		 *   2. doubled quotes     ""  - the other widespread AT convention
		 *   3. unquoted payload       - some builds accept a bare argument
		 *
		 * This matters more than it looks. The graceful path publishes its
		 * own conformant JSON, so the will only ever reaches the broker when
		 * the gateway dies UNGRACEFULLY - a brownout, a watchdog bite, a lost
		 * signal. Those are precisely the events an operator most needs to
		 * parse, and a bare string is the one payload their parser will choke
		 * on.
		 */
		snprintf(will, sizeof(will),
			 "AT+QMTCFG=\"will\",0,1,1,1,\"%s\","
			 "\"{\\\"st\\\":\\\"offline\\\",\\\"ts\\\":%u}\"",
			 topic, ts);
		ok = ms_at(will, "OK", 3000);

		if (!ok) {
			snprintf(will, sizeof(will),
				 "AT+QMTCFG=\"will\",0,1,1,1,\"%s\","
				 "\"{\"\"st\"\":\"\"offline\"\",\"\"ts\"\":%u}\"",
				 topic, ts);
			ok = ms_at(will, "OK", 3000);
			if (ok) {
				printk("MS: will accepted with \"\" quoting\n");
			}
		}
		if (!ok) {
			snprintf(will, sizeof(will),
				 "AT+QMTCFG=\"will\",0,1,1,1,\"%s\","
				 "{\"st\":\"offline\",\"ts\":%u}",
				 topic, ts);
			ok = ms_at(will, "OK", 3000);
			if (ok) {
				printk("MS: will accepted unquoted\n");
			}
		}
		if (!ok) {
			/*
			 * Last resort. A bare string is a s7.3 deviation, but NO
			 * will at all is worse: a dead gateway would leave the
			 * retained status reading "online" indefinitely.
			 */
			printk("MS: no JSON will form accepted - using a bare "
			       "string (s7.3 deviation on the LWT only)\n");
			snprintf(will, sizeof(will),
				 "AT+QMTCFG=\"will\",0,1,1,1,\"%s\",\"offline\"",
				 topic);
			(void)ms_at(will, "OK", 2000);
		}
	}

	if (!ms_at("AT+CPIN?", "READY", 5000)) {
		printk("MS: SIM not READY\n");
		return false;
	}

	/*
	 * Attach takes tens of seconds from cold - scan, authenticate, attach.
	 * Asking once and giving up is how a healthy module gets reported dead.
	 */
	bool att = false;

	/*
	 * 60 s, not 30. A cold LTE registration in marginal signal regularly
	 * takes longer than half a minute - band scan, cell selection, EPS
	 * attach - and giving up early turns a slow attach into a failed cycle.
	 */
	for (int i = 0; i < 60 && !att; i++) {
		att = ms_at("AT+CGATT?", "+CGATT: 1", 2000);
		if (!att) {
			k_msleep(1000);
			gw_wdt_alive();
		}
	}
	if (!att) {
		/*
		 * Dump the state that separates the four reasons CGATT stays 0.
		 * Read them together:
		 *
		 *   CSQ 99,99          no signal at all - antenna, or no coverage
		 *   CSQ <= 5           signal too weak to register
		 *   CSQ good, CEREG 0  not even searching - usually SIM or band
		 *   CEREG 2            searching, so it just needs longer
		 *   CEREG 3            registration DENIED - the network refused
		 *                      this SIM: no data plan, barred, wrong APN
		 *   CEREG 1 or 5       registered; CGATT should follow shortly
		 *   COPS empty         no operator selected yet
		 *
		 * A supply that sags shows up as CSQ swinging wildly between
		 * attempts, or the module resetting mid-attach - registration is
		 * the highest-current phase the modem ever runs.
		 */
		printk("MS: no network attach after 60 s - state follows\n");
		ms_show("AT+CSQ", 2000);        /* signal:  <rssi>,<ber>       */
		ms_show("AT+CPIN?", 2000);      /* SIM:     READY / not ready  */
		ms_show("AT+CEREG?", 2000);     /* LTE reg: <n>,<stat>         */
		ms_show("AT+CREG?", 2000);      /* 2G/3G reg fallback          */
		ms_show("AT+COPS?", 3000);      /* operator + access tech      */
		return false;
	}
	/*
	 * Sample the signal for telemetry.
	 *
	 * +CSQ: <rssi>,<ber> where rssi is 0..31 in 2 dBm steps from -113 dBm,
	 * and 99 means "not known or not detectable" - which must NOT be mapped
	 * onto the scale, or an unmeasurable link reports a strong +85 dBm.
	 */
	{
		static char b[128];
		int raw = 99;

		(void)ms_collect(b, sizeof(b), 50);
		ms_send("AT+CSQ");
		(void)ms_collect(b, sizeof(b), 2000);

		char *p = strstr(b, "+CSQ:");

		if (p && sscanf(p, "+CSQ: %d", &raw) == 1 && raw >= 0 && raw <= 31) {
			s_rssi_dbm = -113 + 2 * raw;
		} else {
			s_rssi_dbm = 0;          /* unknown */
		}
		printk("MS: signal %d dBm (CSQ %d)\n", s_rssi_dbm, raw);
	}

	/*
	 * Serving cell id, via CEREG=2 which adds <tac>,<ci> to the reply.
	 * Best-effort: an empty string means "not reported", never a fake id.
	 */
	{
		static char b[160];

		s_cell[0] = '\0';
		(void)ms_at("AT+CEREG=2", "OK", 2000);
		(void)ms_collect(b, sizeof(b), 50);
		ms_send("AT+CEREG?");
		(void)ms_collect(b, sizeof(b), 2000);

		char *p = strstr(b, "+CEREG:");

		if (p) {
			/* +CEREG: 2,<stat>,"<tac>","<ci>",<AcT> - take <ci>. */
			char *q1 = strchr(p, '"');
			char *q2 = q1 ? strchr(q1 + 1, '"') : NULL;
			char *q3 = q2 ? strchr(q2 + 1, '"') : NULL;
			char *q4 = q3 ? strchr(q3 + 1, '"') : NULL;

			if (q3 && q4 && (q4 - q3 - 1) > 0 &&
			    (size_t)(q4 - q3 - 1) < sizeof(s_cell)) {
				memcpy(s_cell, q3 + 1, q4 - q3 - 1);
				s_cell[q4 - q3 - 1] = '\0';
			}
		}
	}

	/*
	 * An ALREADY-ACTIVE context cannot be reconfigured - QICSGP answers a bare
	 * ERROR. Skipping both commands in that case is what makes a second run
	 * work; treating the ERROR as fatal is what made the test publish once and
	 * then fail every time afterwards.
	 */
	if (!ms_at("AT+QIACT?", "+QIACT: 1,1", 3000)) {
		if (CELL_APN[0]) {
			snprintf(cmd, sizeof(cmd),
				 "AT+QICSGP=1,1,\"%s\",\"\",\"\",1", CELL_APN);
			if (!ms_at(cmd, "OK", 3000)) {
				printk("MS: APN \"%s\" rejected\n", CELL_APN);
				return false;
			}
		}
		if (!ms_at("AT+QIACT=1", "OK", 60000)) {
			printk("MS: PDP activation failed\n");
			return false;
		}
	}

	snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=0,\"%s\",%d", MQTT_HOST, MQTT_PORT);
	if (!ms_at(cmd, "+QMTOPEN: 0,0", 30000)) {
		/* Already open from a previous report is a success, not a failure. */
		if (!ms_at("AT+QMTOPEN?", "+QMTOPEN: 0", 3000)) {
			printk("MS: broker socket failed\n");
			return false;
		}
	}

	if (MQTT_USERNAME[0]) {
		snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
			 cfg_gw_id(), MQTT_USERNAME, MQTT_PASSWORD);
	} else {
		snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\"", cfg_gw_id());
	}
	if (!ms_at(cmd, "+QMTCONN: 0,0,0", 30000)) {
		printk("MS: MQTT connect refused\n");
		return false;
	}

	printk("MS: CONNECTED to %s:%d as %s\n", MQTT_HOST, MQTT_PORT,
	       cfg_gw_id());
	return true;
}

/* ---------------------------------------------------------------- public API */

/*
 * Establish or re-validate the broker session. Caller must already hold the
 * FIFO (uart_irq_rx_disable), because every reply below is read by polling.
 *
 * Split out of modem_simple_publish() so link_up() can ask for a session
 * WITHOUT having something to publish. That is what lets the gateway's `online`
 * flag mean "modem_simple is connected" rather than "ec200 answered AT" - two
 * things that disagreed on this board, with the second permanently false.
 */
static bool ms_session(void)
{
	if (!s_connected) {
		modem_power_on();
		if (!ms_bringup()) {
			printk("MS: modem did not answer AT\n");
			return false;
		}
		(void)ms_at("ATE0", "OK", 2000);      /* echo off, link proven */
		(void)ms_at("AT+CMEE=2", "OK", 1000); /* verbose errors        */
		if (!ms_connect()) {
			return false;
		}
		s_connected = true;
		return true;
	}

	if (!ms_ping(2)) {
		/*
		 * Stale session - rebuild NOW, not on the next call.
		 *
		 * Returning false here cost one publish every single cycle.
		 * link_down() powers the modem off but nothing cleared
		 * s_connected, so each cycle opened believing it still had a
		 * session, discovered otherwise, and threw the message away. The
		 * heartbeat is the first publish of a cycle, so the heartbeat was
		 * always the casualty - while alarms raised later in the same
		 * cycle found the flag already cleared and published normally.
		 */
		printk("MS: session stale - rebuilding\n");
		s_connected = false;

		if (!ms_bringup()) {
			printk("MS: modem did not answer AT\n");
			return false;
		}
		(void)ms_at("ATE0", "OK", 2000);
		(void)ms_at("AT+CMEE=2", "OK", 1000);
		if (!ms_connect()) {
			return false;
		}
		s_connected = true;
	}
	return true;
}

/*
 * Tell this module the session is gone.
 *
 * Called from link_down() right after AT+QPOWD, so the next cycle starts from a
 * cold, honest state instead of pinging a module that has no power. Without it
 * the stale-session path above runs every cycle - harmless now that it rebuilds,
 * but it still wastes two AT timeouts before discovering the obvious.
 */
void modem_simple_session_closed(void)
{
	s_connected = false;
}

/*
 * Bring the session up with nothing to send.
 *
 * link_up() calls this to decide whether the gateway is online. Returns 0 when
 * the broker session is good.
 */
int modem_simple_up(void)
{
	int rc;

	if (!device_is_ready(uart)) {
		return -1;
	}
	uart_irq_rx_disable(uart);
	rc = ms_session() ? 0 : -1;
	uart_irq_rx_enable(uart);
	gw_wdt_alive();
	return rc;
}

/*
 * Publish with an explicit RETAIN flag.
 *
 * The status topic must be retained: a subscriber that connects between the
 * gateway's brief online windows would otherwise see nothing at all, which is
 * indistinguishable from a gateway that has never existed.
 */
static int ms_pub(const char *topic, const char *payload, int retain);

int modem_simple_publish(const char *topic, const char *payload)
{
	return ms_pub(topic, payload, 0);
}

int modem_simple_publish_retain(const char *topic, const char *payload)
{
	return ms_pub(topic, payload, 1);
}

static int ms_pub(const char *topic, const char *payload, int retain)
{
	static char cmd[192];
	static char buf[256];
	int rc = -1;

	if (!device_is_ready(uart) || !topic || !payload) {
		return -1;
	}

	/*
	 * Own the FIFO for the whole exchange. With the ISR running, every
	 * uart_poll_in() below would find an empty register because ec_isr() had
	 * already moved the byte into its ring buffer - which is exactly why
	 * polled code copied out of the test could never see a reply here.
	 */
	uart_irq_rx_disable(uart);

	if (!ms_session()) {
		goto out;
	}

	/*
	 * QMTPUB replies with a bare '>' prompt, then the payload is terminated by
	 * Ctrl-Z (0x1A) - NOT by a newline, which is the usual way this goes wrong.
	 */
	/*
	 * QoS 1 with a real message id.
	 *
	 * This was QoS 0 (msgid 0), inherited from the all_hw_test bench path.
	 * At QoS 0 the module answers +QMTPUB the moment it has handed the bytes
	 * to TCP - it does not and cannot tell you the broker received them. A
	 * stale socket after a PDP drop returns success and discards the packet,
	 * which is precisely the "publish OK, nothing on the broker" case.
	 *
	 * At QoS 1 the module waits for the broker PUBACK before answering, so a
	 * result of 0 genuinely means delivered. That is also what
	 * telem_flush_backlog() has always assumed when it pops a record only on
	 * success - an assumption that was false until now.
	 *
	 * msgid must be non-zero and unique in flight; it wraps at 65535.
	 */
	static uint16_t s_msgid;

	if (++s_msgid == 0) {
		s_msgid = 1;
	}
	snprintf(cmd, sizeof(cmd), "AT+QMTPUB=0,%u,1,%d,\"%s\"",
		 s_msgid, retain, topic);
	if (!ms_at(cmd, ">", 5000)) {
		printk("MS: no '>' prompt\n");
		s_connected = false;
		goto out;
	}
	for (const char *p = payload; *p; p++) {
		uart_poll_out(uart, (unsigned char)*p);
	}
	uart_poll_out(uart, 0x1A);

	/*
	 * Wait for the WHOLE +QMTPUB line, then judge it.
	 *
	 * This used to break out as soon as "+QMTPUB:" appeared and then test the
	 * buffer for the complete string "+QMTPUB: 0,0,0". At the instant the
	 * prefix arrives the rest of the line has not been received yet, so that
	 * test looked at a buffer that could not contain it and reported FAILED on
	 * publishes the broker had accepted. Every one of those was then buffered
	 * to the NOR ring as if it had never been sent - so a working link
	 * quietly produced duplicates.
	 *
	 * It also demanded three fields. Quectel documents
	 * "+QMTPUB: <id>,<msgid>,<result>", but this module's firmware answers a
	 * QoS-0 publish with two, so the third is parsed only if present.
	 */
	int64_t end = k_uptime_get() + 15000;
	size_t n = 0;

	while (k_uptime_get() < end && n < sizeof(buf) - 1) {
		unsigned char c;

		if (uart_poll_in(uart, &c) == 0) {
			buf[n++] = (char)c;
			buf[n] = '\0';

			char *p = strstr(buf, "+QMTPUB:");

			/* Only judge once the line is terminated - otherwise we
			 * are parsing a fragment. */
			if (p && strchr(p, '\n')) {
				int id = 0, mid = 0, res = 0;
				int got = sscanf(p, "+QMTPUB: %d,%d,%d",
						 &id, &mid, &res);

				/*
				 * At QoS 1 the RESULT field is the whole point:
				 * 0 = the broker acknowledged. A reply without
				 * it is accepted only because some firmware
				 * omits the field entirely - but a PRESENT
				 * non-zero result is a real failure and must
				 * not be reported as success.
				 */
				rc = (got >= 2 && (got < 3 || res == 0)) ? 0 : -1;
				if (rc != 0) {
					printk("MS: broker rejected publish "
					       "(result %d)\n", res);
				}
				break;
			}
			if (strstr(buf, "ERROR")) {
				break;          /* definite refusal, stop early */
			}
		}
		gw_wdt_alive();
	}
	if (rc != 0 && n == 0) {
		printk("MS: no response to QMTPUB in 15 s\n");
	}
	printk("MS: publish %s (%s)\n", rc == 0 ? "OK" : "FAILED", topic);

out:
	uart_irq_rx_enable(uart);
	gw_wdt_alive();
	return rc;
}

int modem_simple_rssi_dbm(void)
{
	return s_rssi_dbm;               /* 0 = not measured */
}

const char *modem_simple_cell(void)
{
	return s_cell;                   /* "" = not reported */
}

bool modem_simple_is_up(void)
{
	return s_connected;
}
