/*
 * Store-and-forward ring buffer on NOR flash + FRAM pointers.
 *
 * Layout: the "archive" fixed-partition is divided into SLOT-sized (1 KB)
 * slots, four per 4 KB NOR erase sector. A record SPANS as many slots as it
 * needs and starts with [magic:1][len:2][kind:1]. head =
 * next slot to write, tail = oldest unsent slot; both live in FRAM so they
 * survive resets and wear the flash evenly. On overflow the oldest record is
 * overwritten (ring), EXCEPT a buffered alarm/event (critical) is never
 * overwritten by a heartbeat - a full ring drops the incoming heartbeat
 * instead, so alarms survive a long outage.
 *
 * Requires in app.overlay: a jedec,spi-nor node with an 'archive' partition,
 * and an 'fram' eeprom node. See CONFIG_ flags in prj.conf.
 */
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/eeprom.h>
#include <string.h>
#include "storage.h"

/*
 * SLOT is the per-record stride; SECTOR is the NOR's erase granularity.
 *
 * These were both 4096, i.e. one record per erase sector. A record is at most
 * ~640 B (the largest telemetry message buffer), so ~84 % of every sector was
 * wasted and the 15 MB archive held only 3840 records - about 26 days at the
 * RDSO s7.4 10-minute cadence, SHORT OF THE ONE MONTH s7.5 REQUIRES.
 *
 * NOR flash must be ERASED a whole sector at a time, but can be WRITTEN in
 * smaller pieces anywhere already erased. So the slot can shrink to 1 KB while
 * erasing stays sector-sized: erase once when head crosses into a new sector,
 * then write its four slots individually. That gives 15360 records - about
 * 106 days - with no change to the head/tail slot-index model.
 */
#define SECTOR        4096u
#define SLOT          1024u
#define SLOTS_PER_SEC (SECTOR / SLOT)
/* Bumped when the on-flash record format changed (3-byte header -> 4-byte with
 * REC_MAGIC). A mismatch makes storage_init() reset the ring, so a gateway
 * carrying records in the OLD layout starts clean instead of misparsing them. */
/* rev 3 adds a 4-byte monotonic sequence to each record header, which is what
 * lets the ring be rebuilt from the flash alone when there is no FRAM. */
#define MAGIC         0x57414750u   /* WAGP, format rev 3 */

static const struct flash_area *fa;
static const struct device *const fram = DEVICE_DT_GET(DT_NODELABEL(fram));
static uint32_t num_slots;

struct meta { uint32_t magic, head, tail, count, seq_next; };
static struct meta m;

/*
 * Is the FRAM actually fitted?
 *
 * IC3 is a DNP on the current boards and gets soldered later, so this is not a
 * fault path - it is a supported configuration. With FRAM the metadata is read
 * back directly at boot; without it the same values are RECONSTRUCTED by
 * scanning the NOR (see scan_rebuild). Nothing else in this file cares which
 * happened, and no code changes when the part is finally fitted.
 */
static bool have_fram;

#define FRAM_META_OFF 0

static int meta_save(void)
{
	if (!have_fram) {
		/*
		 * Nowhere to persist to, and nothing is lost by that: every field
		 * in `m` is recoverable from the record headers themselves. This
		 * is a no-op, not a failure - returning an error here would make
		 * storage_append() report a perfectly good write as failed.
		 */
		return 0;
	}
	return eeprom_write(fram, FRAM_META_OFF, &m, sizeof(m));
}

/* Forward declarations - the scan needs the record helpers defined below. */
static int len_at(uint32_t s);
static uint32_t seq_at(uint32_t s);
static uint32_t slots_for(int len);

/*
 * Count the live records between tail and head by walking forward.
 *
 * Cannot just be "records found by the scan": storage_append() PADS to slot 0
 * when a record will not fit in the remaining slots, and the skipped region may
 * still hold older records that are logically behind the tail. Walking the
 * actual [tail, head) chain is what the rest of this file means by count, so it
 * is what gets reconstructed.
 *
 * Invalid slots inside the walk are pad, not records - stepped over without
 * counting. The guard bounds the loop at one lap so a damaged chain cannot spin
 * forever at boot.
 */
static uint32_t chain_count(uint32_t tail, uint32_t head)
{
	uint32_t n = 0, s = tail, guard = 0;

	while (s != head && guard++ < num_slots) {
		int len = len_at(s);

		if (len < 0) {
			s = (s + 1u) % num_slots;   /* pad */
			continue;
		}
		n++;
		s = (s + slots_for(len)) % num_slots;
	}
	return n;
}

/*
 * Rebuild head / tail / count / seq_next from the flash contents alone.
 *
 * This is what lets the archive work with no FRAM fitted. Each record header
 * carries a monotonic sequence, so the OLDEST live record is the one with the
 * lowest sequence (that is the tail) and the NEWEST is the highest (head sits
 * just past it). Nothing else is needed - the ordering is in the flash.
 *
 * Cost is one 8-byte header read per slot in the worst case. An erased archive
 * reads 0xFF everywhere, so no header validates and all ~14800 slots are
 * probed: roughly half a second at 8 MHz, once, at boot. Records advance the
 * cursor by their whole span, so a full archive scans far faster than an empty
 * one.
 */
static void scan_rebuild(void)
{
	uint32_t s = 0, found = 0;
	uint32_t min_seq = 0, max_seq = 0;
	uint32_t tail = 0, head = 0;

	while (s < num_slots) {
		int len = len_at(s);

		if (len < 0) {
			s++;
			continue;
		}

		uint32_t seq = seq_at(s);
		uint32_t span = slots_for(len);

		if (found == 0 || seq < min_seq) {
			min_seq = seq;
			tail = s;
		}
		if (found == 0 || seq > max_seq) {
			max_seq = seq;
			head = (s + span) % num_slots;
		}
		found++;
		s += span;
	}

	if (found == 0) {
		m.head = m.tail = m.count = 0;
		m.seq_next = 0;
		printk("storage: no records on flash - starting empty\n");
		return;
	}

	m.tail     = tail;
	m.head     = head;
	m.count    = chain_count(tail, head);
	m.seq_next = max_seq + 1u;

	printk("storage: rebuilt from flash - %u record(s), "
	       "tail %u head %u seq %u\n",
	       m.count, m.tail, m.head, m.seq_next);
}

int storage_init(void)
{
	int rc = flash_area_open(FIXED_PARTITION_ID(archive_partition), &fa);
	if (rc) {
		printk("storage: flash_area_open failed (%d)\n", rc);
		return rc;
	}
	num_slots = fa->fa_size / SLOT;
	m.magic = MAGIC;

	have_fram = device_is_ready(fram);

	/*
	 * FRAM present: read the metadata back directly - one small read instead
	 * of a whole-partition scan.
	 *
	 * Anything inconsistent falls through to the scan rather than resetting
	 * the ring. That is a real improvement over resetting: FRAM metadata can
	 * be STALE if power dropped between the NOR write and the FRAM write, and
	 * the flash is the ground truth in that case. Discarding a valid backlog
	 * because its bookkeeping was one write behind is exactly the failure
	 * store-and-forward exists to prevent.
	 */
	if (have_fram) {
		struct meta f;

		if (eeprom_read(fram, FRAM_META_OFF, &f, sizeof(f)) == 0 &&
		    f.magic == MAGIC && f.head < num_slots && f.tail < num_slots) {
			m = f;
			printk("storage: %u records buffered (FRAM)\n", m.count);
			return 0;
		}
		printk("storage: FRAM metadata unusable - rebuilding from flash\n");
	} else {
		/*
		 * IC3 not fitted. This is a SUPPORTED configuration, not a fault:
		 * the ring is fully functional and survives reboots, because the
		 * sequence numbers in the record headers carry the ordering that
		 * would otherwise live in FRAM. Fitting the part later changes
		 * nothing except skipping the scan below.
		 */
		printk("storage: no FRAM - ring state comes from the flash scan\n");
	}

	scan_rebuild();
	meta_save();          /* no-op without FRAM; seeds it when present */
	return 0;
}

/*
 * Record header: [magic:1][len:2][kind:1][seq:4] then the payload.
 *
 * The 4-byte SEQ is what makes the archive self-describing. Without it the
 * flash held no notion of which record was oldest, so head/tail had to come
 * from FRAM and an unfitted FRAM meant no store-and-forward at all. With it, a
 * boot-time scan finds the lowest sequence (the tail) and the highest (the
 * head), so the ring reconstructs itself from the NOR alone.
 *
 * It wraps after 2^32 records - at the RDSO s7.4 ten-minute cadence, about
 * 81,000 years - so wrap handling would be dead code.
 *
 * REC_MAGIC exists because records now SPAN slots. With one record per slot,
 * tail could never land mid-record. Now it can - and a single bad length field
 * would desynchronise tail from every following record, silently turning the
 * whole backlog into garbage rather than costing one record. The magic lets a
 * misaligned tail be DETECTED and resynchronised.
 *
 * 0xA6 is deliberately not 0xFF (erased flash) and not 0x00.
 *
 * IT CHANGED FROM 0xA5 WITH THE 8-BYTE HEADER, and it had to. The struct MAGIC
 * guards the FRAM metadata, not the records, so an old 4-byte-header record
 * still carried 0xA5 and a plausible length - it validated, and seq_at() then
 * read the first four bytes of its JSON PAYLOAD as a sequence number. A bench
 * boot reported "seq 1953309308", which is ASCII text from the record body, and
 * a bogus seq that large would have made every genuinely new record look older
 * than the stale one for the rest of the ring's life.
 *
 * Bumping this makes records written by any earlier build fail len_at() and be
 * skipped as pad, which is the correct outcome: one stale record is worth far
 * less than a coherent ring.
 */
#define REC_MAGIC  0xA6u
#define REC_HDR    8u

/* Slots a record of `len` payload bytes occupies, including its header. */
static uint32_t slots_for(int len)
{
	return ((uint32_t)len + REC_HDR + SLOT - 1u) / SLOT;
}

/* Payload length of the record starting at slot `s`, or -1 if that slot does
 * not begin a valid record. */
static int len_at(uint32_t s)
{
	uint8_t hdr[REC_HDR];

	if (flash_area_read(fa, (off_t)s * SLOT, hdr, REC_HDR) != 0) {
		return -1;
	}
	if (hdr[0] != REC_MAGIC) {
		return -1;
	}

	int len = hdr[1] | (hdr[2] << 8);

	if (len <= 0 || (uint32_t)len + REC_HDR > num_slots * SLOT) {
		return -1;
	}
	return len;
}

/* Sequence number of the record starting at slot `s`. Only meaningful when
 * len_at(s) has already validated that slot. */
static uint32_t seq_at(uint32_t s)
{
	uint8_t b[4];

	if (flash_area_read(fa, (off_t)s * SLOT + 4, b, 4) != 0) {
		return 0;
	}
	return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
	       ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

/* Span in slots of the record at `s`; 1 if the header is not valid, so callers
 * still advance and cannot spin. */
static uint32_t span_at(uint32_t s)
{
	int len = len_at(s);

	return (len < 0) ? 1u : slots_for(len);
}

/*
 * Re-align tail onto a real record boundary. Called when the header at tail
 * does not validate - scans forward for the next intact record so ONE damaged
 * record is lost instead of the entire remaining backlog. Returns true if a
 * valid start was found.
 */
static bool resync_tail(void)
{
	for (uint32_t k = 0; k < num_slots; k++) {
		uint32_t s = (m.tail + k) % num_slots;

		if (len_at(s) >= 0) {
			if (k > 0) {
				printk("storage: resync - skipped %u slot(s) of "
				       "damaged record\n", k);
				m.tail = s;
				if (m.count > 0) {
					m.count--;
				}
				meta_save();
			}
			return true;
		}
	}
	printk("storage: no valid record found - ring reset\n");
	m.tail = m.head;
	m.count = 0;
	meta_save();
	return false;
}

/* Do slot ranges [a, a+na) and [b, b+nb) overlap? Records never wrap (see the
 * pad in storage_append), so plain linear comparison is correct. */
static bool overlaps(uint32_t a, uint32_t na, uint32_t b, uint32_t nb)
{
	return (a < b + nb) && (b < a + na);
}

int storage_append(const char *rec, int len, int critical)
{
	/*
	 * Records SPAN as many 1 KB slots as they need. This is what lets a full
	 * ~2.4 KB heartbeat be buffered at all: with one-record-per-slot it was
	 * silently rejected by the length check, so on a fully-fitted wagon every
	 * heartbeat was lost during an outage while alarms survived.
	 */
	if (!fa) {
		/*
		 * No archive partition open - there is NO store-and-forward, so
		 * anything that fails to publish is gone. Report it as -ENODEV,
		 * not -EINVAL: the old code made every such loss look like an
		 * oversized record in the log, which sent a bench investigation
		 * chasing record sizes when the NOR simply was not there.
		 */
		return -ENODEV;
	}
	if (len <= 0) {
		return -EINVAL;
	}

	uint32_t n = slots_for(len);

	if (n > num_slots) {
		return -EINVAL;          /* larger than the entire ring */
	}

	/*
	 * Keep every record contiguous - a spanning record must not wrap the end
	 * of the partition, or peek/pop would have to stitch two reads together.
	 * If it will not fit in the remaining slots, skip them and restart at 0.
	 * The skipped slots still hold older records; they stay readable and are
	 * consumed normally, so nothing is lost by padding.
	 */
	if (m.head + n > num_slots) {
		m.head = 0;
		/* An EMPTY ring has tail == head; leaving tail stranded near the end
		 * would make the next peek read a slot that was never written. When
		 * records ARE live they sit in [tail, old_head), so the skipped
		 * region holds nothing and tail is already correct. */
		if (m.count == 0) {
			m.tail = 0;
		}
	}

	/*
	 * Erase every sector this record will touch. Erasing destroys the WHOLE
	 * 4 KB sector, so any live record overlapping it must be retired first -
	 * and if one of them is critical while this record is only a heartbeat,
	 * refuse instead, so a long outage cannot flush alarms out of the ring.
	 */
	for (uint32_t k = 0; k < n; k++) {
		uint32_t s = m.head + k;

		if ((s % SLOTS_PER_SEC) != 0) {
			continue;        /* already erased with its sector */
		}

		uint32_t sec0 = s;

		if (!critical) {
			uint32_t t = m.tail, remaining = m.count;

			while (remaining > 0) {
				uint32_t tspan = span_at(t);

				if (!overlaps(t, tspan, sec0, SLOTS_PER_SEC)) {
					break;
				}

				uint8_t kind = 0;

				if (flash_area_read(fa, (off_t)t * SLOT + 3,
						    &kind, 1) == 0 && (kind & 1)) {
					return -ENOSPC;   /* keep the alarm */
				}
				t = (t + tspan) % num_slots;
				remaining--;
			}
		}

		/* Retire the records this erase is about to destroy. */
		while (m.count > 0) {
			uint32_t tspan = span_at(m.tail);

			if (!overlaps(m.tail, tspan, sec0, SLOTS_PER_SEC)) {
				break;
			}
			m.tail = (m.tail + tspan) % num_slots;
			m.count--;
		}

		if (flash_area_erase(fa, (off_t)sec0 * SLOT, SECTOR) != 0) {
			return -EIO;
		}
	}

	off_t off = (off_t)m.head * SLOT;
	uint32_t seq = m.seq_next++;
	uint8_t hdr[REC_HDR] = { REC_MAGIC,
				 (uint8_t)(len & 0xFF), (uint8_t)(len >> 8),
				 (uint8_t)(critical ? 1 : 0),
				 (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8),
				 (uint8_t)(seq >> 16), (uint8_t)(seq >> 24) };

	if (flash_area_write(fa, off, hdr, REC_HDR) != 0 ||
	    flash_area_write(fa, off + REC_HDR, rec, len) != 0) {
		return -EIO;
	}

	m.head = (m.head + n) % num_slots;
	m.count++;
	meta_save();
	return 0;
}

int storage_count(void) { return (int)m.count; }

void storage_seq_span(uint32_t *oldest, uint32_t *newest)
{
	/*
	 * seq_next is the NEXT id to be issued, so the newest record in the ring
	 * carries seq_next-1. The oldest is that minus the count. Both are exact
	 * because every append allocates exactly one id, and neither needs a
	 * flash read.
	 */
	uint32_t newest_seq = m.seq_next ? m.seq_next - 1u : 0u;

	if (oldest) {
		*oldest = (m.count && newest_seq >= m.count - 1u)
			      ? newest_seq - (m.count - 1u) : 0u;
	}
	if (newest) {
		*newest = m.count ? newest_seq : 0u;
	}
}

int storage_peek(char *buf, int maxlen)
{
	if (m.count == 0) {
		return -1;
	}
	/* Validate the header before trusting its length. If tail has drifted into
	 * the middle of a record, resync to the next intact one rather than
	 * reading payload bytes as a length and corrupting everything after. */
	int len = len_at(m.tail);

	if (len < 0) {
		if (!resync_tail()) {
			return -1;
		}
		len = len_at(m.tail);
		if (len < 0) {
			return -EIO;
		}
	}
	if (len >= maxlen) {
		/* Caller's buffer is too small - report rather than truncate, since
		 * a partial record would be published as if complete. */
		printk("storage: record %d B exceeds caller buffer %d B\n",
		       len, maxlen);
		return -EIO;
	}
	if (flash_area_read(fa, (off_t)m.tail * SLOT + REC_HDR, buf, len) != 0) {
		return -EIO;
	}
	buf[len] = '\0';
	return len;
}

void storage_pop(void)
{
	if (m.count == 0) {
		return;
	}
	/* Advance by the record's OWN span - a heartbeat occupies several slots,
	 * so a fixed +1 would leave tail pointing into the middle of it and the
	 * next peek would read garbage as a length. */
	m.tail = (m.tail + span_at(m.tail)) % num_slots;
	m.count--;
	meta_save();
}
