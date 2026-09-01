/*
 * EC200 cellular modem driver (AT commands over uart22), with just enough
 * MQTT support to publish telemetry to the broker.
 *
 *   nRF P1.06 (uart22 TX) -> EC200 RXD
 *   nRF P1.07 (uart22 RX) <- EC200 TXD
 *   common GND;  EC200 powered + SIM inserted + antenna attached
 *
 * MQTT uses the Quectel AT flow:
 *   QICSGP -> QIACT -> QMTOPEN -> QMTCONN -> QMTPUB
 *
 * NOTE: this is best-effort and cannot be tested here without the modem.
 * Set EC_ECHO to 1 to print the full AT dialog to the console for debugging.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include <hal/nrf_gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

#include "ec200.h"
#include "config.h"
#include "app_config.h"
#include "modem_cmd.h"
#include "power.h"
#include "watchdog.h"   /* modem_pwrkey_pulse() */

#define EC_ECHO 1                        /* 1 = print AT traffic to console */
#define EC200_UART DT_NODELABEL(uart22)

static const struct device *const uart = DEVICE_DT_GET(EC200_UART);

RING_BUF_DECLARE(ec_rb, 2048);

/* ---------------- low level ---------------- */

static void ec_isr(const struct device *dev, void *user_data)
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
		ring_buf_put(&ec_rb, buf, r);
	}
}

/*
 * Console echo, LINE buffered.
 *
 * This used to be printk("%c") per character. At a few kB of AT dialog per
 * cycle that is thousands of printk calls into a queue that cannot drain that
 * fast, so the logger dropped whole blocks - "--- 1720 messages dropped ---" -
 * and destroyed exactly the values worth reading (+CSQ, +QMTCONN, +QMTPUB
 * results). Buffering to a newline cuts it to a few dozen calls per cycle, so
 * the dialog actually survives to the console.
 */
#if EC_ECHO
static void ec_echo(char c)
{
	static char ln[192];
	static int n;

	if (c == '\r' || c == '\n') {
		if (n) { ln[n] = '\0'; printk("EC< %s\n", ln); n = 0; }
		return;
	}
	if (n >= (int)sizeof(ln) - 1) {
		ln[n] = '\0'; printk("EC< %s\n", ln); n = 0;
	}
	ln[n++] = c;
}
#else
static void ec_echo(char c) { ARG_UNUSED(c); }
#endif

/* ---- downlink URC sniffer ------------------------------------------------
 * A "+QMTRECV:" URC can land in the middle of ANY wait. The broker pushes
 * queued dn/cmd messages the instant it sends the SUBACK, and the modem emits
 * the URC BEFORE its own "+QMTSUB: 0,1,0,1" result - so ec_wait() scanning for
 * the SUBACK consumed the command and threw it away. (Observed on hardware:
 * the +QMTRECV line appears in the echo, which only ec_wait() produces.) The
 * same race can swallow a command while we are blocked on a PUBACK.
 *
 * So every byte taken off the ring passes through here first, no matter who
 * consumed it. A "+QMTRECV:" line is copied into the pending queue and
 * ec200_mqtt_poll_cmd() serves it later, so nothing depends on WHICH wait
 * happened to be running when the broker pushed.
 */
#define EC_PEND_SLOTS 3
#define EC_PEND_LEN   512

static char s_pend[EC_PEND_SLOTS][EC_PEND_LEN];
static uint8_t s_pend_head, s_pend_tail;   /* head==tail => empty */

static bool ec_pend_ready(void) { return s_pend_head != s_pend_tail; }

static bool ec_pend_get(char *out, size_t n)
{
	if (!ec_pend_ready()) {
		return false;
	}
	strncpy(out, s_pend[s_pend_tail], n - 1);
	out[n - 1] = '\0';
	s_pend_tail = (s_pend_tail + 1) % EC_PEND_SLOTS;
	return true;
}

static void ec_sniff(char c)
{
	static char win[16];
	static int wl;
	static bool cap;
	static int ol;

	if (cap) {
		if (c == '\r' || c == '\n') {
			s_pend[s_pend_head][ol] = '\0';
			cap = false;
			uint8_t next = (s_pend_head + 1) % EC_PEND_SLOTS;
			if (next == s_pend_tail) {
				printk("EC200: downlink queue full - command dropped\n");
			} else {
				s_pend_head = next;
				printk("EC200: downlink URC captured\n");
			}
		} else if (ol < EC_PEND_LEN - 1) {
			s_pend[s_pend_head][ol++] = c;
		}
		return;
	}

	if (wl < (int)sizeof(win) - 1) {
		win[wl++] = c;
	} else {
		memmove(win, win + 1, sizeof(win) - 2);
		win[sizeof(win) - 2] = c;
	}
	win[wl < (int)sizeof(win) - 1 ? wl : (int)sizeof(win) - 1] = '\0';

	if (strstr(win, "+QMTRECV:")) {
		cap = true;
		ol = 0;
		wl = 0;
		win[0] = '\0';
	}
}

/* Pull one byte off the ring, sniffing it. 1 = got a byte. */
static int ec_getc(uint8_t *b)
{
	if (ring_buf_get(&ec_rb, b, 1) != 1) {
		/*
		 * FALL BACK TO POLLING when the ring is empty.
		 *
		 * Every reply this driver has ever received arrives through
		 * ec_isr(). On this board it has received NOTHING: across every
		 * production log there is not one "EC<" line, while "EC>" shows
		 * commands going out continuously and the modem visibly acts on
		 * them. all_hw_test talks to the same modem on the same pins with
		 * CONFIG_UART_INTERRUPT_DRIVEN=n and pure uart_poll_in, and works
		 * every time.
		 *
		 * Rather than keep hunting for why the interrupt never fires,
		 * read the FIFO directly when the ring has nothing. If the ISR is
		 * healthy this branch is almost never taken - the ring is already
		 * full of bytes. If it is not, the driver still receives.
		 *
		 * The two cannot race for the same byte: uart_fifo_read() in the
		 * ISR and uart_poll_in() here both drain the same hardware
		 * register, so whichever gets it puts it through ec_sniff() below
		 * and into the same stream.
		 */
		if (uart_poll_in(uart, b) != 0) {
			return 0;
		}
	}
	ec_sniff((char)*b);
	return 1;
}

static void ec_flush_rx(void)
{
	uint8_t b;
	while (ec_getc(&b) == 1) {
	}
}

static void ec_send(const char *s)
{
#if EC_ECHO
	if (s[0] != '\r' && s[0] != '\n') { printk("EC> %s\n", s); }
#endif
	for (const char *p = s; *p; p++) {
		uart_poll_out(uart, (unsigned char)*p);
	}
}

/*
 * Wait until `tok` appears in the incoming stream, or `err` appears, or we
 * time out. Returns 0 (tok), -1 (err seen), -2 (timeout).
 */
static int ec_wait(const char *tok, const char *err, int timeout_ms)
{
	static char win[513];
	int len = 0;
	win[0] = '\0';

	int64_t deadline = k_uptime_get() + timeout_ms;
	uint8_t b;

	while (k_uptime_get() < deadline) {
		while (ec_getc(&b) == 1) {
			ec_echo((char)b);
			if (len < (int)sizeof(win) - 1) {
				win[len++] = (char)b;
			} else {
				memmove(win, win + 1, sizeof(win) - 2);
				win[sizeof(win) - 2] = (char)b;
			}
			win[len < (int)sizeof(win) - 1 ? len : (int)sizeof(win) - 1] = '\0';

			if (tok && strstr(win, tok)) {
				return 0;
			}
			if (err && strstr(win, err)) {
				return -1;
			}
		}
		k_msleep(2);
	}
	return -2;
}

/* Send "cmd\r\n", wait for `resp` (or ERROR / timeout). */
static int at(const char *cmd, const char *resp, int timeout_ms)
{
	ec_flush_rx();
	ec_send(cmd);
	ec_send("\r\n");
	return ec_wait(resp, "ERROR", timeout_ms);
}

/* Read up to `want` RAW bytes from the RX ring (no echo). Returns bytes read
 * (may be < want on timeout). Used for the binary body of an HTTP download. */
static int ec_read_raw(uint8_t *buf, int want, int timeout_ms)
{
	int got = 0;
	int64_t deadline = k_uptime_get() + timeout_ms;
	while (got < want && k_uptime_get() < deadline) {
		uint32_t r = ring_buf_get(&ec_rb, buf + got, want - got);
		if (r > 0) {
			got += (int)r;
		} else {
			k_msleep(2);
		}
	}
	return got;
}

/* Capture the line that contains `prefix` (from prefix start to end-of-line)
 * into `out`, e.g. to read "+QHTTPGET: 0,200,262144". 0 on success, -2 timeout. */
static int ec_capture(const char *prefix, char *out, int n, int timeout_ms)
{
	char win[96];
	int wl = 0, ol = 0;
	bool found = false;
	int64_t deadline = k_uptime_get() + timeout_ms;
	uint8_t b;

	while (k_uptime_get() < deadline) {
		while (ec_getc(&b) == 1) {
			char c = (char)b;
			if (!found) {
				if (wl < (int)sizeof(win) - 1) {
					win[wl++] = c;
				} else {
					memmove(win, win + 1, sizeof(win) - 2);
					win[sizeof(win) - 2] = c;
				}
				win[wl < (int)sizeof(win) - 1 ? wl : (int)sizeof(win) - 1] = '\0';
				char *p = strstr(win, prefix);
				if (p) {
					found = true;
					for (char *q = p; *q && ol < n - 1; q++) {
						out[ol++] = *q;
					}
				}
			} else if (c == '\r' || c == '\n') {
				out[ol] = '\0';
				return 0;
			} else if (ol < n - 1) {
				out[ol++] = c;
			}
		}
		k_msleep(2);
	}
	if (found) { out[ol] = '\0'; return 0; }
	return -2;
}

/*
 * Read AT+CSQ and print the value on ONE line of its own.
 *
 * The raw echo of +CSQ was unreadable in every field log - the number itself
 * always landed inside a "messages dropped" gap - and it is the one figure
 * that separates "weak signal" from "protocol bug" when publishes start
 * failing. So capture it and translate it.
 *
 * rssi: 0..31 maps to -113..-51 dBm in 2 dB steps; 99 = not detectable.
 * Below about 10 (-93 dBm) TCP keeps stalling and QoS 1 PUBACKs start timing
 * out, which is exactly the +QMTPUB: 0,N,1 / +QMTSTAT: 0,3 pattern.
 */
static void ec_report_signal(void)
{
	char line[48];

	ec_flush_rx();
	ec_send("AT+CSQ");
	ec_send("\r\n");
	if (ec_capture("+CSQ:", line, sizeof(line), 2000) != 0) {
		printk("EC200: signal UNKNOWN (no +CSQ response)\n");
		return;
	}

	/* Parse the first integer by hand - no need to drag sscanf() in here. */
	int rssi = 99;
	const char *p = strchr(line, ':');
	if (p) {
		while (*p && (*p < '0' || *p > '9')) { p++; }
		if (*p) {
			rssi = 0;
			while (*p >= '0' && *p <= '9') { rssi = rssi * 10 + (*p++ - '0'); }
		}
	}
	if (rssi == 99) {
		printk("EC200: signal NONE (+CSQ 99 - check antenna/coverage)\n");
	} else {
		int dbm = -113 + 2 * rssi;
		printk("EC200: signal %d/31 (%d dBm) %s\n", rssi, dbm,
		       rssi >= 15 ? "good" : rssi >= 10 ? "marginal" : "POOR");
	}
	(void)ec_wait("OK", "ERROR", 1000);
}

/* ---------------- APN discovery ---------------- */

/*
 * Extract the Nth double-quoted field from an AT response line.
 * +CGDCONT: 1,"IP","airtelgprs.com","10.1.2.3",0,0
 *              ^1   ^2               ^3
 * Returns 0 on success. Used instead of strtok so the source line is not
 * modified and an unterminated quote cannot run off the end.
 */
static int quoted_field(const char *line, int want, char *out, size_t n)
{
	const char *p = line;
	int found = 0;

	while (*p) {
		if (*p != '"') {
			p++;
			continue;
		}
		p++;                       /* opening quote */
		const char *start = p;

		while (*p && *p != '"') {
			p++;
		}
		if (*p != '"') {
			return -1;         /* unterminated -> malformed line */
		}
		if (++found == want) {
			size_t len = (size_t)(p - start);

			if (len >= n) {
				len = n - 1;
			}
			memcpy(out, start, len);
			out[len] = '\0';
			return 0;
		}
		p++;                       /* closing quote */
	}
	return -1;
}

int ec200_query_apn(char *out, size_t n)
{
	char line[192];

	if (!out || n == 0) {
		return -EINVAL;
	}
	out[0] = '\0';

	/* 1. modem alive */
	at("ATE0", "OK", 2000);

	int ok = -1;

	for (int i = 0; i < 15; i++) {
		if (at("AT", "OK", 1000) == 0) {
			ok = 0;
			break;
		}
		k_msleep(1000);
	}
	if (ok != 0) {
		printk("APN: modem not responding to AT\n");
		return -1;
	}
	at("AT+CMEE=2", "OK", 1000);        /* verbose errors while diagnosing */

	/* 2. SIM present */
	if (at("AT+CPIN?", "READY", 5000) != 0) {
		printk("APN: SIM not READY (check SIM seating / PIN)\n");
		return -2;
	}

	/* 3. SIM identifiers - quote these to the operator if you need to be told
	 *    the APN for this account. ICCID should match the number printed on
	 *    the card. */
	ec_flush_rx();
	ec_send("AT+QCCID\r\n");
	if (ec_capture("+QCCID:", line, sizeof(line), 3000) == 0) {
		printk("APN: %s\n", line);
	}
	ec_flush_rx();
	ec_send("AT+CIMI\r\n");
	(void)ec_wait("OK", "ERROR", 3000);   /* IMSI echoes via EC_ECHO */

	/* 4. Attach. Until the network attaches there is no assigned APN to read;
	 *    this is the step that actually takes time on a cold SIM. */
	ec_report_signal();
	at("AT+CGATT=1", "OK", 10000);

	bool attached = false;
	int64_t deadline = k_uptime_get() + 120000;

	while (k_uptime_get() < deadline && !attached) {
		ec_flush_rx();
		ec_send("AT+CGATT?\r\n");
		if (ec_wait("+CGATT: 1", "+CGATT: 0", 3000) == 0) {
			attached = true;
			break;
		}
		at("AT+CEREG?", "+CEREG:", 2000);
		printk("APN: waiting for network attach...\n");
		k_msleep(3000);
	}
	if (!attached) {
		printk("APN: not attached - no APN assigned yet\n");
		return -3;
	}

	/* 5. Ask what the network/SIM actually gave us for context 1. */
	ec_flush_rx();
	ec_send("AT+CGDCONT?\r\n");
	if (ec_capture("+CGDCONT: 1", line, sizeof(line), 5000) != 0) {
		printk("APN: no +CGDCONT response\n");
		return -4;
	}

	/* Field 1 is the PDP type ("IP"/"IPV4V6"); field 2 is the APN. */
	if (quoted_field(line, 2, out, n) != 0 || out[0] == '\0') {
		printk("APN: could not parse '%s'\n", line);
		return -5;
	}

	/* 6. Is that context already active (network-assigned default bearer)?
	 *    If so the APN above is live and CELL_APN need not be set at all. */
	ec_flush_rx();
	ec_send("AT+QIACT?\r\n");
	if (ec_capture("+QIACT:", line, sizeof(line), 3000) == 0) {
		printk("APN: context state: %s\n", line);
	}

	printk("APN: ===> SIM is using APN \"%s\" <===\n", out);
	printk("APN: put this in CELL_APN (app_config.h)\n");
	return 0;
}

/* ---------------- public API ---------------- */


/*
 * POLLED AT probe - decides whether the RX INTERRUPT path is the problem.
 *
 * TX and RX do not use the same mechanism here. ec_send() writes with
 * uart_poll_out, but replies arrive through ec_isr() into a ring buffer. So a
 * dead RX interrupt looks exactly like a dead modem: commands go out, the
 * module acts on them, and nothing ever comes back.
 *
 * all_hw_test polls BOTH directions and brings this same modem up on this same
 * board every time, which makes the interrupt path the one part of the
 * transport that has never been proven here.
 *
 * This reads with uart_poll_in instead, bypassing the ISR and the ring buffer
 * entirely. RX interrupts are disabled around it so the two cannot race for the
 * same bytes, and restored afterwards.
 *
 *   polled works, interrupt does not -> the ISR is the fault
 *   neither works                    -> the fault is below the UART
 */
int ec200_at_polled(void)
{
	char buf[256];
	size_t n = 0;

	uart_irq_rx_disable(uart);

	/* Drain anything the ISR left behind, then anything still in the FIFO. */
	ec_flush_rx();
	for (int i = 0; i < 512; i++) {
		unsigned char c;

		if (uart_poll_in(uart, &c) != 0) {
			break;
		}
	}

	for (const char *p = "AT\r\n"; *p; p++) {
		uart_poll_out(uart, (unsigned char)*p);
	}

	int64_t end = k_uptime_get() + 2000;

	while (k_uptime_get() < end && n < sizeof(buf) - 1) {
		unsigned char c;

		if (uart_poll_in(uart, &c) == 0) {
			buf[n++] = (char)c;
			buf[n] = '\0';
			if (strstr(buf, "OK")) {
				uart_irq_rx_enable(uart);
				printk("POLLED AT: OK - the modem answers. The "
				       "RX INTERRUPT path is the fault.\n");
				return 0;
			}
		}
	}
	uart_irq_rx_enable(uart);

	for (size_t i = 0; i < n; i++) {
		if (buf[i] == '\r' || buf[i] == '\n') {
			buf[i] = ' ';
		}
	}
	buf[n] = '\0';
	printk("POLLED AT: no OK either (%u bytes: %s) - the fault is below "
	       "the UART, not in the ISR.\n", (unsigned)n, n ? buf : "silence");
	return -1;
}

void ec200_init(void)
{
	if (!device_is_ready(uart)) {
		printk("EC200: uart22 not ready\n");
		return;
	}
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

	uart_irq_callback_user_data_set(uart, ec_isr, NULL);
	uart_irq_rx_enable(uart);
	printk("EC200: uart22 ready (TX=P1.06, RX=P1.07)\n");
}


/*
 * ---- MODEM BRING-UP, COPIED FROM all_hw_test ----------------------------
 *
 * Byte for byte the sequence in test_modem(), which starts this module on this
 * board every time. The application's own version never has, and comparing the
 * two line by line found no difference that explained it - so the difference is
 * something the comparison missed, and the way to stop missing it is to stop
 * paraphrasing.
 *
 * Specifically this uses uart_poll_in throughout. The test sets
 * CONFIG_UART_INTERRUPT_DRIVEN=n and polls; this driver reads through ec_isr()
 * into a ring buffer and has never delivered a single byte on this hardware -
 * there is not one "EC<" line in any production log.
 */

/*
 * Raw polled read - ONLY valid while the RX interrupt is disabled.
 *
 * Deliberately does NOT go through ec_getc(). Inside the bring-up window the
 * ISR is off, so the FIFO is ours and this is precisely what all_hw_test does.
 */
static int ec_poll_raw(char *b, size_t n, int ms)
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

/* all_hw_test's at_ping(): flush 50 ms, send AT, look for OK within 1 s. */
static bool ec_ping_raw(int tries)
{
	static char buf[256];

	for (int i = 0; i < tries; i++) {
		(void)ec_poll_raw(buf, sizeof(buf), 50);
		for (const char *p = "AT\r\n"; *p; p++) {
			uart_poll_out(uart, (unsigned char)*p);
		}
		gw_wdt_alive();
		if (ec_poll_raw(buf, sizeof(buf), 1000) > 0 && strstr(buf, "OK")) {
			printk("EC< %s\n", buf);
			return true;
		}
	}
	return false;
}

/*
 * Polled AT check for use with the interrupt RUNNING - reads through ec_getc()
 * so it sees whatever the ISR has already moved into the ring.
 */
bool ec200_at_ok(int tries)
{
	static char buf[256];

	for (int i = 0; i < tries; i++) {
		ec_flush_rx();
		ec_send("AT");
		ec_send("\r\n");
		if (ec_wait("OK", NULL, 1500) == 0) {
			return true;
		}
	}
	return false;
}

/*
 * Bring the modem up, with the RX interrupt DISABLED throughout.
 *
 * Called ONCE per cycle from link_up(), never from inside a retry loop: PWRKEY
 * is a toggle, and pulsing it repeatedly walks the module on, off, on.
 */
bool ec200_bringup(void)
{
	static char buf[512];
	bool ok;

	uart_irq_rx_disable(uart);

	/* Drop anything the ISR banked before we took over, and anything still
	 * sitting in the FIFO, so the first AT starts from silence. */
	ec_flush_rx();
	(void)ec_poll_raw(buf, sizeof(buf), 50);

	if (ec_ping_raw(2)) {
		printk("EC200: already up (no PWRKEY needed)\n");
		uart_irq_rx_enable(uart);
		return true;
	}

	printk("EC200: silent - pulsing PWRKEY\n");
	modem_pwrkey_pulse();            /* 600 ms + boot settle */

	gw_wdt_alive();
	int n = ec_poll_raw(buf, sizeof(buf), 5000);
	gw_wdt_alive();

	if (n > 0) {
		printk("EC200: boot text, %d bytes\n", n);
	}

	ok = ec_ping_raw(8);
	printk("EC200: %s after PWRKEY\n", ok ? "ANSWERED" : "still silent");

	if (!ok) {
		/*
		 * Read the modem's TX line (nRF P1.07) as a LEVEL.
		 *
		 * The module is demonstrably running - its NETLIGHT blinks after
		 * the pulse - yet nothing arrives by any path. Two causes remain
		 * and they need different fixes:
		 *
		 *   DRIVEN high  the TXS0102 is powered and passing an idle UART
		 *                line; the module is up but not transmitting, or
		 *                is transmitting at another baud.
		 *   LOW          the translator is not passing at all. Its A-side
		 *                supply is EC200U VDDEXT (module pin 7) into
		 *                TXS0102 pin 3; without it no byte can cross,
		 *                however healthy the module.
		 *
		 * all_hw_test prints this same reading and gets DRIVEN high on a
		 * working board, so the two runs compare directly.
		 *
		 * Read through the HAL: pinctrl owns P1.07 for uart22 and
		 * gpio_pin_configure() would take it back. The IN register still
		 * reflects the pad, so this observes without disturbing anything.
		 */
		const uint32_t pin = NRF_GPIO_PIN_MAP(1, 7);
		int hi = 0, lo = 0, edges = 0;
		int prev = (int)nrf_gpio_pin_read(pin);
		int64_t stop = k_uptime_get() + 200;

		while (k_uptime_get() < stop) {
			int v = (int)nrf_gpio_pin_read(pin);

			v ? hi++ : lo++;
			if (v != prev) {
				edges++;
				prev = v;
			}
		}

		if (edges) {
			printk("EC200: RX P1.07 - %d edges, module IS sending "
			       "(suspect BAUD)\n", edges);
		} else if (hi > lo) {
			printk("EC200: RX P1.07 - idle HIGH, translator passing, "
			       "module not sending\n");
		} else {
			/*
			 * A LOW RX line here usually just means the module is
			 * not powered YET, not that the translator is broken.
			 *
			 * The TXS0102's VCCA comes from the EC200U's own VDDEXT
			 * (module pin 7), so until the module has finished
			 * booting the translator has no A-side supply and its
			 * B-side output sits low. That is the normal state
			 * before the module answers - and this line used to
			 * announce it as a hardware fault, sending bench time
			 * after a translator that works perfectly a few seconds
			 * later once the module is up.
			 *
			 * Treat it as a hardware fault only if it is STILL low
			 * after the module has answered AT elsewhere.
			 */
			printk("EC200: RX P1.07 - low (module not up yet; the "
			       "TXS0102 VCCA comes from EC200U VDDEXT, so this "
			       "is expected before boot completes)\n");
		}
	}

	uart_irq_rx_enable(uart);
	return ok;
}

int ec200_modem_up(void)
{
	/*
	 * Establish contact with a BARE "AT" before anything else.
	 *
	 * This used to open with ATE0 and expect OK. A module that has just been
	 * started is still emitting its boot URCs - RDY, +CFUN: 1, +CPIN: READY -
	 * and that reply stream lands in the ring buffer around the ATE0
	 * response, so the matcher can miss the OK it is waiting for and burn its
	 * 2 s on a module that is answering perfectly well.
	 *
	 * all_hw_test opens with "AT" and only turns echo off once the link is
	 * proven, which is the safer order: AT is the one command whose reply
	 * cannot be confused with anything else. Flushing first discards whatever
	 * boot text is already queued, so the first exchange starts clean.
	 */
	ec_flush_rx();

	/*
	 * CHECK ONLY - never pulse PWRKEY here.
	 *
	 * link_up() calls this function in a retry loop, so anything that
	 * toggles PWRKEY inside it toggles the modem on, off, on across the
	 * retries. That is exactly what happened when the bring-up was called
	 * from here: two "pulsing PWRKEY" lines per cycle and a module that
	 * ended up switched off.
	 *
	 * ec200_bringup() is called ONCE per cycle by link_up(), before this
	 * loop. By the time we get here the module should already be running, so
	 * a polled AT check is all that is needed.
	 */
	if (!ec200_at_ok(3)) {
		printk("EC200: no response to AT\n");
		return -1;
	}

	/* Link proven - now it is safe to turn the echo off. Doing this before
	 * AT succeeded was how the old order wasted its first attempt. */
	at("ATE0", "OK", 2000);
	at("AT+CMEE=2", "OK", 1000);       /* verbose errors */

	if (at("AT+CPIN?", "READY", 5000) != 0) {
		printk("EC200: SIM not READY (check SIM/PIN)\n");
		return -2;
	}

	ec_report_signal();

	/* Ask the modem to attach to the packet-domain, then WAIT until it is
	 * actually attached (+CGATT: 1). QIACT will keep returning ERROR until
	 * this succeeds. This is the step that was missing. */
	at("AT+CGATT=1", "OK", 10000);

	bool attached = false;
	int64_t deadline = k_uptime_get() + 120000;   /* up to 2 min */
	int n = 0;
	while (k_uptime_get() < deadline && !attached) {
		ec_flush_rx();
		ec_send("AT+CGATT?\r\n");
		if (ec_wait("+CGATT: 1", "+CGATT: 0", 3000) == 0) {
			attached = true;
			break;
		}
		/* also report registration state for visibility */
		at("AT+CEREG?", "+CEREG:", 2000);
		printk("EC200: waiting for network attach... (%d)\n", ++n);
		k_msleep(3000);
	}
	if (!attached) {
		printk("EC200: not attached - check antenna/coverage, SIM "
		       "data plan, and APN (\"%s\")\n", CELL_APN);
		return -3;
	}

	/*
	 * Configure the PDP context, but ONLY if an APN is actually set.
	 *
	 * An empty CELL_APN is not "no APN" to this command - QICSGP answers a
	 * bare ERROR when handed "", and the activation that follows then fails
	 * for a reason the log never explains. all_hw_test hit exactly this on a
	 * modem that was registered on AIRTEL with good signal: everything up to
	 * the bearer worked, and the only visible clue was
	 *
	 *     AT+QICSGP=1,1,"","","",1 ->   ERROR
	 *
	 * Skipping the command instead asks the module to use the bearer the
	 * network assigned, which is what an empty APN was always meant to mean.
	 */
	if (CELL_APN[0]) {
		char cmd[96];

		snprintf(cmd, sizeof(cmd),
			 "AT+QICSGP=1,1,\"%s\",\"\",\"\",1", CELL_APN);
		if (at(cmd, "OK", 3000) != 0) {
			printk("EC200: APN \"%s\" REJECTED by QICSGP - check "
			       "spelling\n", CELL_APN);
		}
	} else {
		printk("EC200: no APN set - relying on the network-assigned "
		       "bearer\n");
	}

	/*
	 * Is context 1 ALREADY active? It usually is on every cycle after the
	 * first whenever the modem is not powered down between wakes, because
	 * link_down() never deactivates the PDP context, so it survives.
	 *
	 * AT+QIACT=1 on an already-active context returns ERROR. Without this
	 * check the retry loop burns all six attempts and returns -4 blaming the
	 * APN - badly misleading, because the APN was fine and had worked on the
	 * previous cycle. Symptom: cycle 1 works, every cycle after it fails at
	 * PDP activation.
	 *
	 * "+QIACT: 1,1" = context 1, state 1 (activated).
	 */
	bool active = (at("AT+QIACT?", "+QIACT: 1,1", 3000) == 0);

	if (active) {
		printk("EC200: PDP context already active - reusing it\n");
	}

	for (int i = 0; i < 6 && !active; i++) {
		if (at("AT+QIACT=1", "OK", 30000) == 0) {
			active = true;
		} else {
			printk("EC200: QIACT retry %d (APN \"%s\")\n",
			       i + 1, CELL_APN);
			/* Clear a half-open/stale context before retrying, else
			 * every attempt hits the same ERROR. Harmless if nothing
			 * was active. */
			at("AT+QIDEACT=1", "OK", 40000);
			k_msleep(3000);
		}
	}
	if (!active) {
		printk("EC200: PDP activation failed after retries. Check, in "
		       "order: SIM data plan active, coverage (AT+CSQ), then the "
		       "APN (\"%s\"). If a PREVIOUS cycle worked, the APN is not "
		       "the problem.\n", CELL_APN);
		return -4;
	}

	at("AT+QIACT?", "+QIACT:", 3000);   /* show the assigned IP */
	printk("EC200: data connection up\n");
	return 0;
}

bool ec200_mqtt_is_up(void)
{
	/*
	 * Ask the modem, do not trust a cached flag.
	 *
	 * The session can drop underneath us at any time - lost registration, a
	 * broker keepalive timeout, the carrier tearing down the PDP context -
	 * and none of those notify the application. Believing a stale "connected"
	 * would send every publish into a dead socket, and QMTPUB would report
	 * success for data that went nowhere.
	 *
	 * AT+QMTCONN? reports state 3 ("connected") for client 0. Anything else,
	 * including no response at all, means the caller must do a full attach.
	 * The timeout is short because this runs on the fast bench path and a
	 * modem that cannot answer a status query in two seconds is not usable.
	 */
	return at("AT+QMTCONN?", "+QMTCONN: 0,3", 2000) == 0;
}

int ec200_mqtt_up(void)
{
	char cmd[160];

	/* In case a previous session is half-open, close it (ignore result). */
	at("AT+QMTCLOSE=0", "OK", 3000);

	/* --- session config (MUST be set before QMTOPEN/QMTCONN) --- */
	/* Persistent session (clean_session=0): the broker QUEUES downlink
	 * commands while the gateway sleeps, and delivers them on reconnect. */
	at("AT+QMTCFG=\"session\",0,0", "OK", 3000);
	/* Deliver received messages inside the +QMTRECV URC (recv/mode=0), which
	 * is what ec200_mqtt_poll_cmd() parses. VERIFY the arg order on your fw. */
	at("AT+QMTCFG=\"recv/mode\",0,0,0", "OK", 3000);
	/* Last-will: broker publishes an offline marker if we drop unexpectedly. */
	snprintf(cmd, sizeof(cmd),
		 /* Bare string here is DEAD CODE on this board - modem_simple
		  * owns the session and sets its own s7.3-conformant will. Left
		  * consistent with the old behaviour rather than diverging. */
		 "AT+QMTCFG=\"will\",0,1,1,1,\"%s/%s/%s/status\",\"offline\"",
		 TOPIC_ROOT, cfg_wagon(), cfg_gw_id());
	at(cmd, "OK", 3000);

#if MQTT_TLS
	/*
	 * TLS (RDSO s3.8: data "must be encrypted by the latest security
	 * standards available"). Without this the whole uplink - telemetry,
	 * alarms and every downlink command - crosses the cellular network in
	 * clear text. The BLE hop to the nodes was already AES-CCM protected;
	 * this closes the far larger hop.
	 *
	 * SSL context 2 is used so it cannot collide with the HTTP(S) OTA
	 * download, which binds context 1 (see ec200_http_download).
	 */
	at("AT+QSSLCFG=\"sslversion\",2,4", "OK", 3000);   /* 4 = TLS 1.2 only  */
	at("AT+QSSLCFG=\"ciphersuite\",2,0xFFFF", "OK", 3000); /* let the server pick */
	at("AT+QSSLCFG=\"seclevel\",2," MQTT_TLS_SECLEVEL_STR, "OK", 3000);
	at("AT+QSSLCFG=\"ignorelocaltime\",2,1", "OK", 3000);

#if MQTT_TLS_SECLEVEL >= 1
	/*
	 * Server authentication needs the CA certificate present in the modem's
	 * filesystem under MQTT_TLS_CA_FILE. It is uploaded once, out of band,
	 * with AT+QFUPL - not on every connect, which would burn modem flash.
	 * If the file is missing the handshake fails and QMTOPEN reports an
	 * error, which is the correct outcome: better a refused connection than
	 * a silently unauthenticated one.
	 */
	at("AT+QSSLCFG=\"cacert\",2,\"" MQTT_TLS_CA_FILE "\"", "OK", 3000);
#endif

	/* Bind SSL context 2 to MQTT client 0 and enable TLS for it. */
	at("AT+QMTCFG=\"ssl\",0,1,2", "OK", 3000);
#endif  /* MQTT_TLS */

	/*
	 * Open the socket, with escalating recovery for a WEDGED MQTT client.
	 *
	 * "+QMTOPEN: 0,2" means the client identifier is already occupied - index
	 * 0 is still held from a previous cycle that ended badly (QMTSTAT 0,8).
	 * That stale state survives any cycle that does not power-cycle the
	 * modem: once wedged, every later cycle fails at QMTOPEN and nothing is
	 * ever published again. Observed on hardware.
	 *
	 * Escalation, cheapest first:
	 *   1. plain open
	 *   2. DISC + CLOSE, waiting for the +QMTCLOSE URC (not just "OK" - the
	 *      close is asynchronous, and not waiting is why a naive retry hits
	 *      the same 0,2), then reopen
	 *   3. AT+CFUN=1,1 - a full modem restart over the UART. Unlike QPOWD
	 *      the modem comes back BY ITSELF, so this is the one hard reset
	 *      available without asserting PWRKEY. Costs a re-attach, hence last.
	 */
	int rc = -1;

	for (int attempt = 0; attempt < 3 && rc != 0; attempt++) {
		if (attempt == 1) {
			printk("EC200: client busy - releasing socket 0\n");
			at("AT+QMTDISC=0", "OK", 5000);
			ec_flush_rx();
			ec_send("AT+QMTCLOSE=0\r\n");
			/* Wait for the CLOSE to actually complete. */
			ec_wait("+QMTCLOSE: 0,0", "ERROR", 10000);
			k_msleep(2000);
		} else if (attempt == 2) {
			printk("EC200: still busy - restarting modem (CFUN=1,1)\n");
			at("AT+CFUN=1,1", "OK", 15000);
			k_msleep(15000);          /* modem reboots */
			for (int i = 0; i < 20; i++) {   /* wait for AT to answer */
				if (at("AT", "OK", 1000) == 0) { break; }
				k_msleep(1000);
			}
			at("ATE0", "OK", 2000);
			/* The restart dropped the PDP context too; bring it back. */
			at("AT+CGATT=1", "OK", 10000);
			snprintf(cmd, sizeof(cmd),
				 "AT+QICSGP=1,1,\"%s\",\"\",\"\",1", CELL_APN);
			at(cmd, "OK", 3000);
			at("AT+QIACT=1", "OK", 30000);
		}

		snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=0,\"%s\",%d",
			 MQTT_HOST, MQTT_PORT);
		ec_flush_rx();
		ec_send(cmd);
		ec_send("\r\n");
		rc = ec_wait("+QMTOPEN: 0,0", "ERROR", 75000);
	}

	if (rc != 0) {
		printk("EC200: QMTOPEN failed after recovery attempts "
		       "(result 2 = client busy, 4 = DNS, 3 = PDP down)\n");
		at("AT+QMTCLOSE=0", "OK", 5000);
		return -1;
	}

	if (MQTT_USERNAME[0] != '\0') {
		snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
			 cfg_gw_id(), MQTT_USERNAME, MQTT_PASSWORD);
	} else {
		snprintf(cmd, sizeof(cmd), "AT+QMTCONN=0,\"%s\"", cfg_gw_id());
	}
	ec_flush_rx();
	ec_send(cmd);
	ec_send("\r\n");
	if (ec_wait("+QMTCONN: 0,0,0", "ERROR", 30000) != 0) {
		printk("EC200: QMTCONN failed\n");
		return -2;
	}

	/*
	 * Subscribe to THE downlink command topic - one, exactly as Protocol
	 * Rev.1 s7.1 defines it:
	 *
	 *   smartwagon/v1/{wgn}/{gw}/dn/cmd
	 *
	 * An earlier revision also subscribed to "smartwagon/v1/grp/<group>/dn/cmd"
	 * and "smartwagon/v1/all/dn/cmd" to carry RDSO s5.1.13's "group wise or
	 * bulk upgrade commands". Those topics are NOT in the spec, and they break
	 * its base pattern: s7.2's documented cloud wildcards
	 * (smartwagon/v1/+/+/up/#) assume position 3 is a wagon number, so a
	 * literal "grp" there mis-parses for every subscriber built to Rev.1.
	 *
	 * Group and bulk are instead carried in the PAYLOAD as "scope", and the
	 * CLOUD fans the command out with one publish per wagon on each wagon's
	 * own topic. The gateway still fully accepts group and bulk commands, so
	 * s5.1.13 is satisfied - the addressing just stays inside the topic tree
	 * the railway specified. It also gives per-wagon delivery confirmation,
	 * which a broadcast to a shared topic can never provide.
	 */
	/*
	 * INDIVIDUAL downlink: this wagon only. Protocol Rev.1 s7.1.
	 */
	snprintf(cmd, sizeof(cmd), "AT+QMTSUB=0,1,\"%s/%s/%s/dn/cmd\",1",
		 TOPIC_ROOT, cfg_wagon(), cfg_gw_id());
	ec_flush_rx();
	ec_send(cmd);
	ec_send("\r\n");
	if (ec_wait("+QMTSUB: 0,1", "ERROR", 10000) != 0) {
		printk("EC200: QMTSUB dn/cmd failed\n");
	}

	/*
	 * BULK downlink: EVERY gateway in the fleet, one publish.
	 *
	 * Without this, "update all wagons" means the back office publishing once
	 * per wagon - 2500 publishes to say one thing, and no way to reach a
	 * wagon whose number the operator does not have to hand.
	 *
	 * VENDOR EXTENSION, not Protocol Rev.1. Rev.1 s7.1 defines only the
	 * per-wagon topic. It is safe alongside the documented cloud wildcard
	 * smartwagon/v1/+/+/up/# because that pattern matches six levels ending
	 * in "up"; this topic has five and ends in "dn/cmd", so an uplink
	 * subscriber built strictly to Rev.1 never sees it. Declare it in the
	 * interface document so the extension is explicit rather than discovered.
	 *
	 * RESPONSES STILL GO TO THIS WAGON'S OWN up/resp. That is the whole point
	 * of a fan-in: the back office can tell which wagons acted on a broadcast
	 * and which never answered.
	 */
	snprintf(cmd, sizeof(cmd), "AT+QMTSUB=0,2,\"%s/all/dn/cmd\",1",
		 TOPIC_ROOT);
	ec_flush_rx();
	ec_send(cmd);
	ec_send("\r\n");
	if (ec_wait("+QMTSUB: 0,2", "ERROR", 10000) != 0) {
		printk("EC200: QMTSUB all/dn/cmd failed\n");
	}
	/* The broker pushes any queued command the instant a SUBACK is sent, and
	 * the URC can arrive BEFORE it. ec_sniff() catches it whichever wait
	 * consumes the bytes, so nothing is lost here. */

	/*
	 * Give any command the broker pushed on the back of the SUBACK a moment
	 * to land, so it can be answered in THIS cycle rather than the next one.
	 * The bytes go through ec_getc() -> ec_sniff(), so they reach the pending
	 * queue wherever they are consumed - including inside the ec_wait() above.
	 */
	ec200_stash_pending_cmd();

	/* Retained birth so the back-office sees us online (clears the will). */
	snprintf(cmd, sizeof(cmd), "AT+QMTPUB=0,0,0,1,\"%s/%s/%s/status\"",
		 TOPIC_ROOT, cfg_wagon(), cfg_gw_id());
	ec_flush_rx();
	ec_send(cmd);
	ec_send("\r\n");
	if (ec_wait(">", "ERROR", 5000) == 0) {
		ec_send("online");
		uart_poll_out(uart, 0x1A);
		ec_wait("+QMTPUB: 0,0", "ERROR", 10000);
	}

	printk("EC200: MQTT connected to %s:%d (subscribed dn/cmd)\n",
	       MQTT_HOST, MQTT_PORT);
	return 0;
}

/*
 * Poll for one queued downlink command. Parses the +QMTRECV URC and returns the
 * JSON command object (from the first '{' to the last '}', which is robust to
 * how the modem quotes the payload). Returns 1 + payload in `out`, or 0 if no
 * message arrived within timeout_ms. Called repeatedly by drain_commands().
 */
void ec200_stash_pending_cmd(void)
{
	int64_t deadline = k_uptime_get() + 500;
	uint8_t b;

	while (k_uptime_get() < deadline && !ec_pend_ready()) {
		while (ec_getc(&b) == 1) {
			ec_echo((char)b);
		}
		k_msleep(2);
	}
}

/* Set by ec200_mqtt_poll_cmd() from the topic in the URC; read immediately
 * afterwards by the command handler. */
static bool s_last_bulk;

bool ec200_last_cmd_was_bulk(void)
{
	return s_last_bulk;
}

int ec200_mqtt_poll_cmd(char *out, size_t n, int timeout_ms)
{
	static char line[EC_PEND_LEN];

	if (!ec_pend_get(line, sizeof(line))) {
		/* Nothing stashed yet - watch the stream for one more window. */
		int64_t deadline = k_uptime_get() + timeout_ms;
		uint8_t b;

		while (k_uptime_get() < deadline && !ec_pend_ready()) {
			while (ec_getc(&b) == 1) {
				ec_echo((char)b);
			}
			k_msleep(2);
		}
		if (!ec_pend_get(line, sizeof(line))) {
			return 0;   /* nothing delivered in this window */
		}
	}
	char *b = strchr(line, '{');
	char *e = strrchr(line, '}');
	if (!b || !e || e < b) {
		return 0;   /* not a JSON command payload */
	}

	/*
	 * Record whether this arrived on the fleet-wide topic. The +QMTRECV URC
	 * carries the topic ahead of the payload, so the information is right
	 * here - and it is the only reliable signal, because an operator
	 * publishing a broadcast can forget "scope" in the JSON. Getting that
	 * wrong would have every wagon in the fleet start the same download in
	 * the same second, so it must not depend on the sender remembering.
	 */
	s_last_bulk = (strstr(line, "/all/dn/cmd") != NULL);
	size_t len = (size_t)(e - b + 1);
	if (len >= n) {
		len = n - 1;
	}
	memcpy(out, b, len);
	out[len] = '\0';
	return 1;
}

int ec200_mqtt_publish(const char *topic, const char *payload)
{
	static uint16_t msgid;
	char cmd[160], ack[24];

	if (++msgid == 0) {
		msgid = 1;               /* 1..65535; QoS1 msgid must be non-zero */
	}

	/* QoS 1 (3rd arg = 1), retain 0. At QoS 1 the +QMTPUB result URC arrives
	 * only AFTER the broker PUBACK, so a 0 return here means DELIVERED - which
	 * is exactly what store-and-forward needs before dropping a buffered
	 * record. (QoS 0 returned "sent", not "delivered" - the earlier bug.) */
	snprintf(cmd, sizeof(cmd), "AT+QMTPUB=0,%u,1,0,\"%s\"", msgid, topic);
	ec_flush_rx();
	ec_send(cmd);
	ec_send("\r\n");

	/* Modem replies with "> " to ask for the payload. */
	if (ec_wait(">", "ERROR", 5000) != 0) {
		printk("EC200: no '>' prompt for publish\n");
		return -1;
	}

	ec_send(payload);
	uart_poll_out(uart, 0x1A);           /* Ctrl-Z ends the message */
#if EC_ECHO
	printk("<Ctrl-Z>\n");
#endif

	/* "+QMTPUB: 0,<msgid>,0" -> result 0 = PUBACK received (QoS1 delivered). */
	snprintf(ack, sizeof(ack), "+QMTPUB: 0,%u,0", msgid);
	if (ec_wait(ack, "ERROR", 20000) != 0) {
		printk("EC200: publish not acknowledged (QoS1)\n");
		return -2;
	}
	return 0;
}

void ec200_disconnect(void)
{
	/*
	 * The "offline" publish that used to live here is GONE.
	 *
	 * It made `status` mean "the modem is powered right now", which for a
	 * device that is deliberately asleep ~99 % of the time reads "offline"
	 * almost always - indistinguishable from a gateway that has failed.
	 *
	 * `status` now means HEALTHY: "online" is published retained on every
	 * session establishment (see link_bringup) and only the broker's Last
	 * Will moves it to "offline", on a session that ends without a
	 * DISCONNECT. A clean cycle therefore leaves it reading "online", which
	 * is what an operator wants to know.
	 */

	/* Graceful MQTT close + modem power-down so the modem draws ~µA between
	 * wakes (the modem is the largest consumer). The persistent session lets
	 * the broker hold queued commands until the next wake reconnect. */
	at("AT+QMTDISC=0", "+QMTDISC: 0,0", 10000);
	at("AT+QMTCLOSE=0", "OK", 5000);
	at("AT+QPOWD=1", "POWERED DOWN", 15000);
}

/* ---------------- HTTP(S) firmware download ---------------- */
int ec200_http_download(const char *url, ec200_sink_fn sink, void *ctx)
{
	char line[80];
	int err = -1, http = 0, clen = 0;

	/* Use the activated PDP context; no response headers in the body. */
	at("AT+QHTTPCFG=\"contextid\",1", "OK", 3000);
	at("AT+QHTTPCFG=\"responseheader\",0", "OK", 3000);
	at("AT+QHTTPCFG=\"sslctxid\",1", "OK", 3000);

	/*
	 * Configure SSL context 1 before any https:// URL is attempted.
	 *
	 * Binding the context above is not enough - an unconfigured context has
	 * no TLS version and no security level, so the handshake fails and the
	 * download reports a transport error that looks exactly like a bad URL.
	 * That cost a bring-up session: the URL was right and the file was
	 * there, but nothing here had ever set the context up.
	 *
	 * Context 1 for HTTP, context 2 for MQTT, deliberately separate: the
	 * broker and the firmware host are different servers with different
	 * CAs, and sharing one context would make changing either break the
	 * other.
	 *
	 * ignorelocaltime matters more than it looks. Certificates carry
	 * validity dates; the modem's clock is not set until it attaches to the
	 * network, so a cold boot would reject a perfectly good certificate for
	 * being "not yet valid". Ignoring local time avoids failing an update
	 * for the one reason the operator can do nothing about.
	 *
	 * These are harmless for a plain http:// URL - the context is simply
	 * never used.
	 */
	at("AT+QSSLCFG=\"sslversion\",1,4", "OK", 3000);       /* 4 = TLS 1.2   */
	at("AT+QSSLCFG=\"ciphersuite\",1,0xFFFF", "OK", 3000); /* server picks  */
	at("AT+QSSLCFG=\"seclevel\",1," OTA_TLS_SECLEVEL_STR, "OK", 3000);
	at("AT+QSSLCFG=\"ignorelocaltime\",1,1", "OK", 3000);
#if OTA_TLS_SECLEVEL >= 1
	/*
	 * Server authentication needs the host's CA in modem flash, uploaded
	 * once out of band with AT+QFUPL. For raw.githubusercontent.com that is
	 * the DigiCert root GitHub currently chains to - verify it before
	 * relying on it, because GitHub has rotated CAs before and a changed
	 * chain fails every update at once.
	 */
	at("AT+QSSLCFG=\"cacert\",1,\"" OTA_TLS_CA_FILE "\"", "OK", 3000);
#endif

	/* Push the URL: QHTTPURL replies CONNECT, then we send the URL bytes. */
	{
		char cmd[48];
		snprintf(cmd, sizeof(cmd), "AT+QHTTPURL=%d,30", (int)strlen(url));
		ec_flush_rx();
		ec_send(cmd);
		ec_send("\r\n");
		if (ec_wait("CONNECT", "ERROR", 5000) != 0) {
			return -1;
		}
		ec_send(url);
		if (ec_wait("OK", "ERROR", 5000) != 0) {
			return -2;
		}
	}

	/* GET -> "+QHTTPGET: <err>,<http_status>,<content_len>" */
	ec_flush_rx();
	ec_send("AT+QHTTPGET=60\r\n");
	if (ec_capture("+QHTTPGET:", line, sizeof(line), 60000) != 0) {
		return -3;
	}
	if (sscanf(line, "+QHTTPGET: %d,%d,%d", &err, &http, &clen) < 3 ||
	    err != 0 || clen <= 0) {
		printk("EC200: QHTTPGET failed (%s)\n", line);
		return -4;
	}

	/*
	 * Insist on 200. This used to check only the Quectel error code and the
	 * content length, which are both happy for a response that is not the
	 * firmware at all:
	 *
	 *   302  a redirect. github.com/<user>/<repo>/raw/... redirects to
	 *        raw.githubusercontent.com, and QHTTPGET does NOT follow it - it
	 *        returns the redirect, whose short HTML body has clen > 0.
	 *   404  a "not found" page. Also HTML, also clen > 0.
	 *
	 * Either would have been streamed into the MCUboot secondary slot and a
	 * swap requested. The signature check would reject it at boot, so no
	 * brick - but the campaign burns a download, the slot is left holding
	 * rubbish, and the failure surfaces one reboot away from its cause.
	 * Refusing here reports the real reason while the URL is still in hand.
	 */
	if (http != 200) {
		printk("EC200: HTTP %d, not 200 - wrong URL, a redirect, or "
		       "auth required. Use the FINAL url (for GitHub that is "
		       "raw.githubusercontent.com, not github.com)\n", http);
		return -5;
	}
	printk("EC200: downloading %d bytes (http %d)\n", clen, http);

	/* READ body: replies CONNECT, then `clen` raw bytes, then OK/+QHTTPREAD. */
	ec_flush_rx();
	ec_send("AT+QHTTPREAD=60\r\n");
	if (ec_wait("CONNECT", "ERROR", 10000) != 0) {
		return -5;
	}

	int remaining = clen;
	uint8_t buf[256];
	while (remaining > 0) {
		int want = remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf);
		int r = ec_read_raw(buf, want, 15000);
		if (r <= 0) {
			printk("EC200: download stalled, %d left\n", remaining);
			return -6;
		}
		if (sink(ctx, buf, r) != 0) {
			return -7;   /* flash write failed -> abort */
		}
		remaining -= r;
	}
	ec_wait("+QHTTPREAD: 0", "ERROR", 5000);
	return clen;
}
