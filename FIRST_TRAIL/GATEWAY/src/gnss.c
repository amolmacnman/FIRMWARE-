/*
 * GNSS module: read NMEA from the LC29H over uart30, parse the RMC sentence,
 * and keep the latest fix available to the rest of the app.
 *
 * Per the gateway netlist:
 *   LC29H TXD (pin 20) -> nRF pin 24 = P0.06  (uart30 RX)  [net UART_TX_LC]
 *   LC29H RXD (pin 21) <- nRF pin 25 = P0.00  (uart30 TX)  [net UART_RX_LC]
 *   LC29H GND -> GND,  VCC (pin 23) -> switched GNSS rail
 *
 * Note which way the net names run: UART_TX_LC is the LC29H's TRANSMITTER, so
 * it arrives at OUR receiver. The nets are named from the module's side, not
 * ours. This comment previously had the two module pins the other way round
 * and the pinctrl was built to match it, which put our TX on the module's TX -
 * two outputs shouting at each other while both receivers sat unconnected.
 *
 * The identical error on the modem is now proven: correcting it there took the
 * EC200U from silent to answering AT. The same correction is applied here by
 * the same rule, but is NOT yet confirmed on hardware - the LC29H has not
 * powered up, so nothing has been received either way. If it powers up and is
 * still silent, re-read the netlist before trusting this block.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/ring_buffer.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "gnss.h"
#include "app_config.h"

#define GNSS_UART DT_NODELABEL(uart30)   /* schematic: LC29H on P0.00/P0.06 */

/* Hysteresis so GPS speed noise near the threshold doesn't flap the state:
 * we switch to "moving" above MOVING_SPEED_KMH and back to "stopped" only
 * once speed drops below half that. */
#define MOVING_ON_KMH   (MOVING_SPEED_KMH)
#define MOVING_OFF_KMH  (MOVING_SPEED_KMH * 0.5)

static const struct device *const uart = DEVICE_DT_GET(GNSS_UART);

RING_BUF_DECLARE(gnss_rb, 1024);
static K_SEM_DEFINE(rx_sem, 0, 1);      /* ISR -> thread wakeup       */
static K_SEM_DEFINE(fix_sem, 0, 1);     /* parser -> "fresh valid fix" */

static struct gnss_fix latest;
static struct k_mutex  latest_lock;

static bool moving_state;
static void (*motion_cb)(bool moving);

/* ---- NMEA line assembler ---- */
#define LINE_MAX 128
static char line[LINE_MAX];
static int  line_len = -1;

static bool checksum_ok(const char *s)
{
	if (s[0] != '$') {
		return false;
	}
	uint8_t sum = 0;
	int i = 1;
	while (s[i] && s[i] != '*') {
		sum ^= (uint8_t)s[i++];
	}
	if (s[i] != '*') {
		return true;
	}
	unsigned int ref = (unsigned int)strtol(&s[i + 1], NULL, 16);
	return sum == (uint8_t)ref;
}

static int split_fields(char *s, char *f[], int max)
{
	int n = 0;
	f[n++] = s;
	for (char *p = s; *p && n < max; p++) {
		if (*p == ',') {
			*p = '\0';
			f[n++] = p + 1;
		} else if (*p == '*') {
			*p = '\0';
		}
	}
	return n;
}

static double nmea_to_deg(const char *val, const char *hemi)
{
	if (!val || val[0] == '\0') {
		return 0.0;
	}
	double raw = atof(val);
	int    deg = (int)(raw / 100.0);
	double min = raw - deg * 100.0;
	double dec = deg + min / 60.0;
	if (hemi && (hemi[0] == 'S' || hemi[0] == 'W')) {
		dec = -dec;
	}
	return dec;
}

/*
 * Which constellations are actually contributing, learned from NMEA talker IDs
 * on GSV sentences.
 *
 * Protocol Rev.1 s2.3 defines loc.sys as "the constellation(s) that produced the
 * fix, so the cloud knows the provenance of every position". It was previously
 * hardcoded to ["navic","gps"] on every message, which is an assertion rather
 * than a measurement - and specifically a wrong one while gnss_set_constellation()
 * never enables NavIC.
 *
 * The mask decays: a constellation must have been seen recently to be reported,
 * otherwise a single GSV at power-up would keep claiming a system that has since
 * dropped out.
 */
#define SYS_GPS      BIT(0)
#define SYS_GLONASS  BIT(1)
#define SYS_GALILEO  BIT(2)
#define SYS_BEIDOU   BIT(3)
#define SYS_QZSS     BIT(4)
#define SYS_NAVIC    BIT(5)

#define SYS_STALE_MS 30000

static uint8_t s_sys_mask;
static int64_t s_sys_seen[6];

static void note_talker(char a, char b)
{
	uint8_t bit = 0;
	int idx = -1;

	if (a == 'G' && b == 'P')                  { bit = SYS_GPS;     idx = 0; }
	else if (a == 'G' && b == 'L')             { bit = SYS_GLONASS; idx = 1; }
	else if (a == 'G' && b == 'A')             { bit = SYS_GALILEO; idx = 2; }
	else if ((a == 'G' && b == 'B') ||
		 (a == 'B' && b == 'D'))           { bit = SYS_BEIDOU;  idx = 3; }
	else if (a == 'G' && b == 'Q')             { bit = SYS_QZSS;   idx = 4; }
	else if ((a == 'G' && b == 'I') ||
		 (a == 'I' && b == 'R'))           { bit = SYS_NAVIC;   idx = 5; }
	else                                       { return; }   /* $GN etc. */

	s_sys_mask |= bit;
	s_sys_seen[idx] = k_uptime_get();
}

int gnss_sys_json(char *out, size_t n)
{
	/*
	 * No fix -> empty list.
	 *
	 * The mask below is built from GSV talker IDs, which report satellites IN
	 * VIEW. That is not what loc.sys means: Rev.1 s2.3 defines it as the
	 * constellations that PRODUCED the fix. Indoors the receiver sees five
	 * systems and fixes on none, so every message was going out asserting
	 * five constellations of provenance for lat 0 / lon 0 / fix "none".
	 *
	 * The satellites-in-view information is not lost - nsat and hdop already
	 * carry it. What this stops is a positional claim with no position
	 * behind it.
	 */
	{
		struct gnss_fix f;

		gnss_get(&f);
		if (!f.valid) {
			return snprintk(out, n, "[]");
		}
	}

	static const char *const name[6] = {
		"gps", "glonass", "galileo", "beidou", "qzss", "navic"
	};
	int64_t now = k_uptime_get();
	int len = 0;
	bool first = true;

	len += snprintk(out + len, n - len, "[");
	for (int i = 0; i < 6; i++) {
		if (!(s_sys_mask & BIT(i))) {
			continue;
		}
		if ((now - s_sys_seen[i]) > SYS_STALE_MS) {
			s_sys_mask &= ~BIT(i);      /* aged out */
			continue;
		}
		len += snprintk(out + len, n - len, "%s\"%s\"",
				first ? "" : ",", name[i]);
		first = false;
	}
	len += snprintk(out + len, n - len, "]");
	return len;
}

/*
 * Parse GGA for fix quality, satellite count and HDOP.
 *
 *   $xxGGA,time,lat,N,lon,E,qual,nsat,hdop,alt,M,...
 *     f[0]  f[1] f[2] f3 f[4] f5 f[6] f[7] f[8]
 *
 * RMC carries position and speed but NOT accuracy, so without GGA the gateway
 * has no basis for the cep it publishes - which RDSO s7.17 requires to be a real
 * measurement. Only these three fields are taken; position still comes from RMC
 * so the two cannot disagree.
 */
static void parse_gga(char *sentence)
{
	char *f[16];
	int   n = split_fields(sentence, f, 16);

	if (n < 9) {
		return;
	}

	k_mutex_lock(&latest_lock, K_FOREVER);
	latest.fix_q = (uint8_t)atoi(f[6]);
	latest.nsat  = (uint8_t)atoi(f[7]);
	latest.hdop  = (float)atof(f[8]);
	k_mutex_unlock(&latest_lock);
}

uint8_t gnss_cep_m(const struct gnss_fix *f)
{
	if (!f || !f->valid || f->fix_q == 0 || f->hdop <= 0.0f) {
		return 99;      /* no usable fix - do not claim an accuracy */
	}

	/* CEP50 ~ HDOP x UERE. 2.5 m UERE is representative of a modern
	 * multi-constellation receiver with a clear sky view; it is the term to
	 * revisit if field data shows the reported cep is optimistic. */
	int cep = (int)(f->hdop * 2.5f + 0.5f);

	if (cep < 1)  { cep = 1; }
	if (cep > 99) { cep = 99; }
	return (uint8_t)cep;
}

/* Parse one RMC sentence and publish it into `latest`. */
static void parse_rmc(char *sentence)
{
	char *f[24];
	int   n = split_fields(sentence, f, 24);
	if (n < 10) {
		return;
	}

	struct gnss_fix fix = {0};
	fix.valid      = (f[2][0] == 'A');
	fix.updated_ms = k_uptime_get();

	if (fix.valid) {
		fix.lat_deg    = nmea_to_deg(f[3], f[4]);
		fix.lon_deg    = nmea_to_deg(f[5], f[6]);
		fix.speed_kmh  = (f[7][0] ? atof(f[7]) : 0.0) * 1.852; /* kn->km/h */
		fix.course_deg = (f[8][0] ? atof(f[8]) : 0.0);
	}

	/* Build ISO-8601 UTC "YYYY-MM-DDThh:mm:ssZ" from RMC time f[1] + date f[9] */
	fix.utc_iso[0] = '\0';
	if (strlen(f[1]) >= 6 && strlen(f[9]) >= 6) {
		int hh = (f[1][0]-'0')*10 + (f[1][1]-'0');
		int mm = (f[1][2]-'0')*10 + (f[1][3]-'0');
		int ss = (f[1][4]-'0')*10 + (f[1][5]-'0');
		int dd = (f[9][0]-'0')*10 + (f[9][1]-'0');
		int mo = (f[9][2]-'0')*10 + (f[9][3]-'0');
		int yy = (f[9][4]-'0')*10 + (f[9][5]-'0');
		snprintf(fix.utc_iso, sizeof(fix.utc_iso),
			 "20%02d-%02d-%02dT%02d:%02d:%02dZ",
			 yy, mo, dd, hh, mm, ss);
	}

	/* Update the shared fix and recompute motion state (with hysteresis). */
	double sp = fix.valid ? fix.speed_kmh : 0.0;
	bool changed = false;
	bool nm;

	k_mutex_lock(&latest_lock, K_FOREVER);
	latest = fix;
	nm = moving_state;
	if (!moving_state && sp > MOVING_ON_KMH) {
		nm = true;
	} else if (moving_state && sp < MOVING_OFF_KMH) {
		nm = false;
	}
	if (nm != moving_state) {
		moving_state = nm;
		changed = true;
	}
	k_mutex_unlock(&latest_lock);

	/* Fire the motion-change event OUTSIDE the lock. */
	if (changed && motion_cb) {
		motion_cb(nm);
	}

	/* Signal any thread waiting for a fresh valid fix. */
	if (fix.valid) {
		k_sem_give(&fix_sem);
	}
}

int gnss_wait_fix(int timeout_ms)
{
	k_sem_reset(&fix_sem);                 /* require a FRESH fix */
	return k_sem_take(&fix_sem, K_MSEC(timeout_ms)) == 0 ? 0 : -EAGAIN;
}

static void feed_byte(char c)
{
	if (c == '$') {
		line_len = 0;
		line[line_len++] = c;
	} else if (line_len >= 0) {
		if (c == '\r' || c == '\n') {
			line[line_len] = '\0';
			if (line_len >= 6 && line[3] == 'R' &&
			    line[4] == 'M' && line[5] == 'C' &&
			    checksum_ok(line)) {
				parse_rmc(line);
			} else if (line_len >= 6 && line[3] == 'G' &&
				   line[4] == 'G' && line[5] == 'A' &&
				   checksum_ok(line)) {
				parse_gga(line);
			} else if (line_len >= 6 && line[3] == 'G' &&
				   line[4] == 'S' && line[5] == 'V') {
				/* Talker ID identifies the constellation, so GSV
				 * tells us which systems are actually contributing.
				 * No checksum gate: we only read the 2-byte talker,
				 * and losing one GSV just delays the mask slightly. */
				note_talker(line[1], line[2]);
			}
			line_len = -1;
		} else if (line_len < LINE_MAX - 1) {
			line[line_len++] = c;
		} else {
			line_len = -1;
		}
	}
}

static void gnss_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);
	uint8_t buf[64];

	if (!uart_irq_update(dev)) {
		return;
	}
	while (uart_irq_rx_ready(dev)) {
		int r = uart_fifo_read(dev, buf, sizeof(buf));
		if (r <= 0) {
			break;
		}
		ring_buf_put(&gnss_rb, buf, r);
	}
	k_sem_give(&rx_sem);            /* wake the parser thread */
}

/* Sleeps until the ISR signals data, then drains + parses. No polling. */
static void gnss_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	uint8_t byte;
	while (1) {
		k_sem_take(&rx_sem, K_FOREVER);
		while (ring_buf_get(&gnss_rb, &byte, 1) == 1) {
			feed_byte((char)byte);
		}
	}
}

K_THREAD_STACK_DEFINE(gnss_stack, 2048);
static struct k_thread gnss_tcb;

void gnss_init(void)
{
	k_mutex_init(&latest_lock);

	if (!device_is_ready(uart)) {
		printk("GNSS: uart30 not ready\n");
		return;
	}
	uart_irq_callback_user_data_set(uart, gnss_isr, NULL);
	/*
	 * RESUME the UART before arming its receiver.
	 *
	 * CONFIG_PM_DEVICE is enabled in this application (the NOR uses it for
	 * deep-power-down) and is absent from all_hw_test. With PM_DEVICE the nRF
	 * UARTE can sit SUSPENDED, and uart_irq_rx_enable() on a suspended uartice
	 * does not arm the receiver - while uart_poll_out keeps working, because
	 * it writes the peripheral registers directly.
	 *
	 * The result is a UART that transmits perfectly and never receives a byte:
	 * every AT command reached the modem, the module acted on them, and not one
	 * reply was ever delivered. Not a single "EC<" line appeared in any
	 * production log, while all_hw_test - which sets
	 * CONFIG_UART_INTERRUPT_DRIVEN=n and polls - talked to the same modem on
	 * the same pins without trouble.
	 *
	 * Resuming explicitly costs nothing when the uartice is already active.
	 */
#ifdef CONFIG_PM_DEVICE
	(void)pm_device_action_run(uart, PM_DEVICE_ACTION_RESUME);
#endif

	uart_irq_rx_enable(uart);

	k_thread_create(&gnss_tcb, gnss_stack, K_THREAD_STACK_SIZEOF(gnss_stack),
			gnss_thread, NULL, NULL, NULL,
			5 /* prio */, 0, K_NO_WAIT);
	k_thread_name_set(&gnss_tcb, "gnss");
	printk("GNSS: listening on uart30 (RX=P0.00)\n");
}

/* Send one PAIR/NMEA command to the LC29H (adds the *CS checksum + CRLF).
 * `body` is the text between '$' and '*', e.g. "PAIR066,1,1,1,1,0,0". The GNSS
 * must be powered (switched rail on) for this to take effect. */
static void gnss_send_cmd(const char *body)
{
	uint8_t cs = 0;
	for (const char *p = body; *p; p++) {
		cs ^= (uint8_t)*p;
	}
	char cmd[64];
	int n = snprintf(cmd, sizeof(cmd), "$%s*%02X\r\n", body, cs);
	for (int i = 0; i < n && i < (int)sizeof(cmd); i++) {
		uart_poll_out(uart, cmd[i]);
	}
}

/*
 * Apply the configured constellation set to the LC29H via PAIR066, then PAIR513
 * to persist it to the module's flash (so it survives GNSS power cycles - we
 * only need to send it when the selection CHANGES). Call while the GNSS rail is
 * powered (e.g. from the set_gnss handler, which runs mid-report after a fix).
 *
 * PAIR066 fields: GPS,GLONASS,Galileo,BDS,QZSS,NavIC (1=enable).
 *
 * FIELD 6 IS NAVIC, and it is now REQUESTED rather than assumed impossible.
 * The plain LC29H does not track NavIC - its protocol spec lists that position
 * as Reserved - but the LC29H(AI) variant does, and this board is expected to
 * move to one. Asking for it means that hardware swap needs no firmware change.
 *
 * Safe on both parts, which is why it is unconditional rather than hidden
 * behind a build flag:
 *
 *   - AI variant: NavIC is enabled and its $GI/$IR sentences start arriving.
 *   - Plain part: the module either ignores the reserved field or rejects the
 *     sentence. A rejected PAIR066 leaves the PREVIOUS flash-saved selection
 *     in place - the multi-GNSS default, which is exactly what this code used
 *     to send. The worst case is the behaviour we already had.
 *
 * NOTHING HERE ASSERTS NAVIC IS WORKING. loc.sys is built from the talker IDs
 * that actually arrive (note_talker), so the uplink reports NavIC only once the
 * module genuinely produces it. Checking `sys` for "navic" after a real fix is
 * the test for whether the swap took - do not trust this command succeeding.
 */
void gnss_set_constellation(uint8_t constel)
{
	const char *sys;
	switch (constel) {
	case 2:  sys = "1,0,0,0,0,0"; break;   /* gps only                       */
	/*
	 * NavIC ONLY. Meaningful on an LC29H(AI); on a plain part it leaves no
	 * usable constellation at all, so it is an explicit operator choice and
	 * never a default.
	 */
	case 1:  sys = "0,0,0,0,0,1"; break;   /* navic only  (AI variant)       */
	/*
	 * navic_gps and auto both request EVERYTHING, NavIC included.
	 *
	 * navic_gps deliberately does NOT mean "GPS and NavIC only". Dropping
	 * GLONASS, Galileo and BeiDou to honour the name would make fixes slower
	 * and less accurate on the plain modules fitted today, for no gain. The
	 * setting exists to say "include NavIC", and that is what it does.
	 */
	case 3:
	case 0:  /* auto */
	default: sys = "1,1,1,1,1,1"; break;   /* all, incl. QZSS + NavIC        */
	}
	char body[32];
	snprintf(body, sizeof(body), "PAIR066,%s", sys);
	gnss_send_cmd(body);
	k_msleep(120);
	gnss_send_cmd("PAIR513");               /* save configuration to flash    */
	printk("GNSS: constellation cfg %u applied (PAIR066,%s)\n", constel, sys);
}

void gnss_get(struct gnss_fix *out)
{
	k_mutex_lock(&latest_lock, K_FOREVER);
	*out = latest;
	k_mutex_unlock(&latest_lock);
}

bool gnss_is_moving(void)
{
	bool m;
	k_mutex_lock(&latest_lock, K_FOREVER);
	m = moving_state;
	k_mutex_unlock(&latest_lock);
	return m;
}

void gnss_register_motion_cb(void (*cb)(bool moving))
{
	motion_cb = cb;
}
