/* SPDX-License-Identifier: MIT */
#ifndef AR_PWR_AUTO_H
#define AR_PWR_AUTO_H
#include <stdint.h>

/* Automatic TX power, the pure part. The decision is the power model
 * (AR_PWR_AUTO_CLEAN_DBM, below); the rules described next are its history,
 * and only the overload step-down is still one of them.
 *
 * Automatic TX power: A receiver judges how it hears its
 * peer and asks the peer for its TX power (stock's pwr_auto2 messages);
 * nobody moves its own power from its own measurement. Each received frame
 * of the peer's is one record: whether it was errored, its SNR (the
 * receiver's raw scale: linear, 36 = 0 dB) and the AGC gains of both paths
 * (a code: higher is a weaker signal).
 *
 * Ours, not stock's step-by-step votes (a walk took 4 s from 13 to 26 dBm):
 * TX power and SNR are both dB and move together, so the climb is the
 * shortfall. The SNR, filtered (s + 3 snr) / 4, in dB:
 *  - under target_db10 with the weaker path's gain at weak_gain or more (a
 *    weak signal - more power raises its SNR; with a strong one the SNR is
 *    the receiver's, ~21-24 dB close by, and more power only overloads it):
 *    ask for the power now + the shortfall, rounded up, in one request
 *    (up_records apart: the next measurement has to see it);
 *  - errored records: `burst` in a row asks for the maximum, `window` in
 *    the last 16 for +3 dB;
 *  - over target + band_db10 (and nothing above): 1 dB down once it has held
 *    down_ms, down_records apart;
 *  - the weaker path's gain at or under overload_gain is a signal too
 *    strong for the receiver: never up then, and with the SNR under the
 *    target too (overloaded: its SNR falls with more power, 17-18 dB at 27
 *    dBm a metre away) 1 dB down after overload_ms. A low gain with a good
 *    SNR is only a strong link: it holds in the band, descends over it.
 * The gain is the weaker path's, filtered over ~16 records (g + (new - g) /
 * 16): one record's swings (26-34 at 26 dBm close by) crossed overload_gain
 * every few records, each restarted the overload hold, and the power sat at
 * the maximum, overloaded, for good (walk, 2026-10-26).
 * Always inside [min, max]. */
struct ar_pwr_auto_policy {
	int16_t target_db10, band_db10;
	uint8_t overload_gain, weak_gain, burst, window;
	/* The peer transmitter's own SNR ceiling (the power model, below). */
	uint8_t clean_dbm;           /* the ceiling is flat up to this power */
	int16_t ceiling_db10;        /* the ceiling there, dB x 10 */
	uint8_t ceiling_slope_db10;  /* how much lower per dB above it, dB x 10 */
	uint8_t knee_dbm;            /* past this power the ceiling falls faster */
	uint8_t knee_slope_db10;     /* how much lower per dB above the knee, dB x 10 */
	uint8_t down_step;           /* the most a request takes off, dB */
	uint8_t up_step;             /* the most a request adds, dB */
	uint16_t down_ms, overload_ms;
	uint16_t up_hold_ms;         /* no model down this long after an up */
	uint8_t up_records, down_records;
};
struct ar_pwr_auto {
	struct ar_pwr_auto_policy policy;
	uint8_t enabled, max_dbm, min_dbm;
	/* The peer's TX power, as announced or as we asked it (0: not known;
	 * taken as max). */
	uint8_t peer_dbm;
	uint16_t window;          /* errored records, the newest in bit 0 */
	uint8_t run, filtered;
	uint16_t snr;             /* filtered, raw scale */
	uint16_t gain_x16;        /* the weaker path's gain, filtered, x16 */
	int16_t snr_db10;         /* the same in dB x 10 */
	int8_t verdict;           /* -1 down held (since verdict_ms), 0 none */
	uint32_t verdict_ms, since_request;
	/* The power model: the SNR over ~300 ms (raw scale, x16), the power it
	 * picks and since when a lower one has been picked. */
	uint32_t slow_x16;
	uint8_t best_dbm, up_votes;
	uint32_t lower_since;
	uint32_t up_at;           /* when the last up was asked (0: none since a reset) */
	uint32_t records, requests_up, requests_down;
};
/* The receiver's raw SNR in dB x 10: 100 log10(raw / 36); 0 or less for 0. */
int ar_pwr_auto_db10(uint32_t raw);
/* The policy's defaults (ar_pwr_auto_init).
 *
 * CHANGE TOGETHER: the SNR target is set against the ground's MCS table -
 * its radio configuration (the profile's bb_config_gnd_open8030.json,
 * baseband/basic/dev/mcs/slot/mcs_tab, raw linear SNR, dB = 10 log10(raw /
 * 36)). In that table today (20 MHz):
 *   MCS 11 (the top): up above 3682 = 20.1 dB, down under 2925 = 19.1 dB
 *   MCS 10:           up above 1846 = 17.1 dB, down under 1466 = 16.1 dB
 *   MCS 8:            up above 1192 = 15.2 dB, down under  947 = 14.2 dB
 *   MCS 6:            up above  598 = 12.2 dB, down under  475 = 11.2 dB
 * A target above a row's "up" keeps the link able to climb to that row; the
 * band above it is where the power steps down. 22.0 dB is the top row with
 * ~2 dB to spare: in flight (2026-10-26) 500 mW gave 19-20 dB from ~30 m out,
 * so the power stayed at the top all flight. 18.0 would aim for MCS 10.
 * Change the table (another profile, 40 MHz rows, the MCS 13 patch) and
 * these follow it. */
#define AR_PWR_AUTO_TARGET_DB10   220   /* the SNR kept, dB x 10 (see above) */
#define AR_PWR_AUTO_BAND_DB10      15   /* over target + band, and clean: 1 dB down */
/* The weaker path's AGC gain, filtered (a code: higher is a weaker signal).
 * Bench and walks, Lite+ at 13-27 dBm: 25-29 overloaded at 26-27 dBm a metre
 * away (their average 26-29, swings to 34), 30-31 at 19-20 dBm with a good
 * SNR, 35-50 a normal signal close by, 57-93 at 100-170 m. CHANGE TOGETHER
 * with the receiver (another ground radio, another front end). */
#define AR_PWR_AUTO_OVERLOAD_GAIN  30   /* at or under, with the SNR short: overloaded - down, never up */
/* Weak from 50 (was 35): armed 1-5 m off, a Lite at full power read gains
 * 35-46 with the SNR stuck at ~19 dB, 24.5 dB disarmed (10 mW, gain 51) at
 * the same spot (flight 2026-10-29) - under 50 the power is probed down. */
#define AR_PWR_AUTO_WEAK_GAIN      50   /* at or over: a weak signal - an SNR shortfall climbs */
/* Timing: errored records in a row for the maximum (burst), in the last 16
 * for +3 dB (window); a down held down_ms (overload_ms when overloaded);
 * requests up_records / down_records records apart (a record per frame,
 * ~210 a second at 4.65 ms). */
#define AR_PWR_AUTO_BURST           2
#define AR_PWR_AUTO_WINDOW          3
#define AR_PWR_AUTO_DOWN_MS      2000
#define AR_PWR_AUTO_OVERLOAD_MS   300
#define AR_PWR_AUTO_UP_RECORDS      8
#define AR_PWR_AUTO_DOWN_RECORDS   16
/* Overload protection only (2026-10-27): the power is the host's maximum
 * unless less is better. With a strong signal (gain under weak_gain) and the
 * SNR short of the target, 1 dB down is tried and judged after PROBE_MS: kept
 * (and the next tried) if the SNR rose by PROBE_GAIN_DB10, else taken back and
 * nothing tried for HOLD_MS. A Lite 30 cm from the goggle at 26 dBm read
 * 21 dB with gain 31 (24 dB at 13 dBm): a gain floor alone missed it. A weak
 * signal (gain at or over weak_gain) gets the maximum at once. */
/* The power model (2026-10-29, replaces the probes above and weak_gain):
 * what the ground hears is the distance's SNR (up 1 dB per dB of power) and
 * the air transmitter's own noise (its SNR ceiling, which falls once the
 * power is past clean_dbm) added: 1/snr = 1/distance + 1/ceiling. From the
 * SNR measured at the power now, the distance's part is what is left after
 * the ceiling's; every power in the range is then predicted, and the best
 * is asked for (ties to the lower; a move only for 1 dB or more). Up on
 * the fast SNR once it has said so for up_records records in a row (a fade),
 * at most up_step dB a request (a deep fade climbs again up_records later),
 * down on the ~300 ms SNR once it has been the pick for down_ms, counted
 * from up_hold_ms after the last up, at most down_step dB a request (drive
 * 2026-10-29 on a buggy: without the hold every fade went 22 -> 26 and the
 * model walked it back 26 -> 24 -> 22 within ~10 s, 13 times in 4 minutes,
 * and 24 <-> 26 every few seconds far out). The receiver overloaded (gain at its floor
 * with the SNR short) still steps down on its own, before the model.
 * Measured on a Lite+ (single TX antenna) 15-30 m off, receiver not
 * overloaded (gain 59-75): the SNR peaked at 20-23 dBm and was 2 dB lower at
 * 26; fitted ceiling ~24 dB to 22 dBm, ~1 dB less per dB above (flight
 * 2026-10-29: 19.5 dB at 26 dBm, 23 at 22, at the same received level).
 * CHANGE TOGETHER with the air unit's front end (another board, another
 * amplifier): re-measure with kestrel-air's pwr= bench knob. */
/* Flight 2026-11-01 (Lite+, armed, 10-40 m, gain 45-75): 19.1 dB at 26 dBm
 * on 20 MHz and 20.6 on 40 against 23.2-23.4 at 22 and 22.8-24 at 24, the
 * flights of 2026-10-30 alike - the ceiling holds to ~24 dBm and drops past
 * it. On the old line (1 dB a dB from 22, 20 dB at 26) a reading under 19.5
 * at 26 looked like distance: the power stayed at 26 close by. Now 24 dB to
 * 22 dBm, 0.5 dB less a dB to 24 (23 dB), 2.5 a dB above (18 dB at 26). */
#define AR_PWR_AUTO_CLEAN_DBM        22
#define AR_PWR_AUTO_CEILING_DB10    240
#define AR_PWR_AUTO_CEILING_SLOPE     5   /* dB x 10 per dB over CLEAN_DBM */
#define AR_PWR_AUTO_KNEE_DBM         24
#define AR_PWR_AUTO_KNEE_SLOPE       25   /* dB x 10 per dB over KNEE_DBM */
#define AR_PWR_AUTO_DOWN_STEP         2
#define AR_PWR_AUTO_UP_STEP           2
#define AR_PWR_AUTO_UP_HOLD_MS    10000
#define AR_PWR_AUTO_MOVE_X1000     1259   /* 1 dB: the least gain a move is made for (0.3 flipped 24 <-> 26 every 10-15 s far out, drive 2026-10-29) */
/* The model's prediction at `dbm` (dB x 10) for a measured SNR (dB x 10) at
 * `now_dbm`, and the power it picks in [min, max] (exposed for tests and
 * the log). */
int ar_pwr_auto_predict_db10(const struct ar_pwr_auto_policy *p, int measured_db10, int now_dbm, int dbm);
uint8_t ar_pwr_auto_best(const struct ar_pwr_auto_policy *p, int measured_db10, int now_dbm,
	int min_dbm, int max_dbm);
/* Off until ar_pwr_auto_set_range. */
void ar_pwr_auto_init(struct ar_pwr_auto *ctx);
/* SET 9 {mode, max, min} on the requesting side: mode 1 on, 0 off. -22
 * for a range outside 9..39 dBm or min above max. */
int ar_pwr_auto_set_range(struct ar_pwr_auto *ctx, unsigned on, unsigned max_dbm, unsigned min_dbm);
/* A link ends or starts: the record history and the peer's power go (a
 * peer goes back to its own reset power when the link drops). */
void ar_pwr_auto_reset(struct ar_pwr_auto *ctx);
/* One record. Returns 1 with *target_dbm and *direction (1 up, 2 down)
 * when a request is due; the caller sends it and reports it with
 * ar_pwr_auto_sent (a busy control channel: the next record tries again). */
unsigned ar_pwr_auto_record(struct ar_pwr_auto *ctx, uint32_t now_ms, unsigned errored,
	uint16_t snr, uint8_t gain_a, uint8_t gain_b, uint8_t *target_dbm, uint8_t *direction);
void ar_pwr_auto_sent(struct ar_pwr_auto *ctx, uint8_t target_dbm, uint8_t direction);
/* The peer said what it transmits at (its announcement): requests step from
 * that, not from the last one sent. */
void ar_pwr_auto_peer(struct ar_pwr_auto *ctx, uint8_t dbm);

/* The request on the air: PHY command 8 (the sync message), sub-type 6, TTL
 * 2, four bytes LE - [3:0] 6, [11:4] the target dBm, [15:12] 1 up / 2 down,
 * [19:16] the slot, [20] broadcast (0 for one ground). */
#define AR_PWR_AUTO_SUBTYPE 6
uint32_t ar_pwr_auto_word(uint8_t target_dbm, uint8_t direction, uint8_t slot);
/* 0 and the fields for a sub-type 6 word, -22 otherwise. */
int ar_pwr_auto_parse(uint32_t word, uint8_t *target_dbm, uint8_t *direction, uint8_t *slot);
/* Ours (Open8030 at both ends; stock sends no such command): an AP
 * announces the TX power it runs at, at link-up and at each change - PHY
 * command 9, its word sub-type 13, [11:4] dBm, other bits 0. A command of
 * its own, not command 8: a 2026.10.beta4 ground takes every command 8 as
 * the frame mode ((byte >> 4) & 7 without looking at the sub-type), and at
 * 26 dBm switched alone to the short frame - the link dropped and re-formed
 * at each announcement. Command 9 it counts as unsupported and leaves.
 * A ground still takes the announcement on command 8 (an AP built in
 * between sends it there). */
#define AR_PHY_PWR_ANNOUNCE 9u
#define AR_PWR_AUTO_ANNOUNCE 13
uint32_t ar_pwr_auto_announce_word(uint8_t dbm);
/* 0 and the power for an announcement word, -22 otherwise. */
int ar_pwr_auto_parse_announce(uint32_t word, uint8_t *dbm);
/* The receiving side: the power to set for a request, given its current
 * power and its host's range; 0 for none. Clamped into [min, max]; "up" to
 * a target not above the current one is taken as current + 1 (stock's
 * out-of-sync rule). */
uint8_t ar_pwr_auto_apply(uint8_t current_dbm, uint8_t target_dbm, uint8_t direction,
	uint8_t min_dbm, uint8_t max_dbm);
#endif
