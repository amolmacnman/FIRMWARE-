#ifndef STORAGE_H
#define STORAGE_H
#include <stdint.h>

/*
 * Store-and-forward archive: a ring buffer of records on the NOR flash
 * "archive" partition, with head/tail/count persisted in FRAM.
 *
 * The archive spans the whole usable NOR (15 MB) since sub-node firmware
 * staging was removed - see partitions in app.overlay.
 */
int  storage_init(void);
/* Buffer one record. critical!=0 (alarms/events) marks it protected: when the
 * ring is full it will NOT be overwritten by a non-critical record. */
int  storage_append(const char *rec, int len, int critical);
int  storage_count(void);

/*
 * Sequence numbers of the oldest and newest buffered records, or 0/0 when the
 * ring is empty. Reported as buf.old / buf.new in the heartbeat, which used to
 * emit hardcoded zeros - indistinguishable from an empty backlog.
 */
void storage_seq_span(uint32_t *oldest, uint32_t *newest);                          /* records waiting         */
int  storage_peek(char *buf, int maxlen);          /* copy oldest; len or -1  */
void storage_pop(void);                            /* drop oldest (after ACK) */

#endif
