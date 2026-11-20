/* SPDX-License-Identifier: MIT */
/* Independently implemented Ascent VRX RF peripheral and DEV receiver. Board
 * profiles are data, not executable vendor instructions. Native video and
 * explicit host restarts are verified; full firmware parity remains open. */
#include "ar_ap_time.h"
#include "ar_cache.h"
#include "ar_rf_device.h"
#include "ar_mac.h"
#include "ar_dev.h"
#include "ar_binding.h"
#include "ar_dev_tx.h"
#include "ar_dev_mcs.h"
#include "ar_fch_monitor.h"
#include "ar_channel_monitor.h"
#include "ar_ap_time.h"
#include "ar_phy_receive.h"
#include "ar_command.h"
#include "ar_scan.h"
#include "ar_scan_scores.h"
#include "ar_pwr_auto.h"
#include "ar_tune.h"
#include "ar_power_control.h"
#include "ar_tx_mcs.h"
#include "ar_tx_irq.h"
#include "ar_power.h"
#include "ar_ap.h"
#include "ar_config.h"
#include "ar_dev_width.h"
#include "ar_frame_stamp.h"
#include <string.h>
#ifndef AR_RF_SIMULATED
#include "ar_sdio_device.h"
#define progress ar_sdio_record_progress
#else
static void progress(uint32_t stage, uint32_t a, uint32_t b, uint32_t c) { (void)stage; (void)a; (void)b; (void)c; }
#endif

#define BB UINT32_C(0x0a110000)
#define CLOCK UINT32_C(0x0a104000)
#define OTP UINT32_C(0x00140000)
#define TIMER UINT32_C(0xe000bff8)
static struct ar_rf_info information;
static uint8_t calibration[5];
static unsigned pins_configured;
static struct ar_rf_rx_info receiver;
/* 48 KiB each: one MAC packet is a block or two (about 10-12 KiB at 20 MHz,
 * twice that at 40); RAM ran out at 64 KiB once upstream's modules joined
 * the USB buffers. A longer packet is refused in receive_event(). */
#ifdef AR_ROLE_AP
/* An air unit receives only the ground's slot (a report and short streams). */
static uint8_t receive_buffers[4][4096] __attribute__((aligned(512)));
#else
static uint8_t receive_buffers[4][49152] __attribute__((aligned(512)));
#endif
/* Receive DMAs in flight, oldest at receive_head: up to four, as stock
 * queues them. With one, a block whose event came before the last block's
 * DMA was taken (a busy poll loop) was dropped. */
static unsigned receive_head, receive_pending, receive_length, receive_user;
static unsigned dev_transmit_channel;
static uint32_t tuned_frequency[12][2];
static uint32_t receive_metadata;
static struct { uint32_t length, user, metadata; } receive_queue[4];
static uint64_t receive_started;
static struct ar_phy_control phy_control;
static uint32_t receive_milliseconds, receive_iterations;
static unsigned receive_operation;
static unsigned listener_capture, listener_follow;
static struct {
    uint64_t due;
    unsigned command, value, hops, widths, rejected;
} listener_change;
static void listener_control(const uint8_t control[7]);
static void listener_poll(void);

static ar_rf_event_callback event_callback;
static void *event_context;
void ar_rf_set_event_callback(ar_rf_event_callback callback, void *context)
{
	event_callback = callback; event_context = context;
}
static void publish_event(unsigned id, const uint8_t *data, size_t length)
{
	if (receive_operation >= 75 && event_callback) { event_callback(id, data, length, event_context); }
}
static void link_event(unsigned current, unsigned previous)
{
	/* DEV 0x22870c publishes host slot0,current,previous. */
	if (current == previous) { return; }
	const uint8_t data[] = {0, current, previous};
	publish_event(0, data, sizeof data);
}
static void mcs_event(unsigned direction, unsigned current, unsigned previous)
{
	/* 0x21f8be: DEV host slot0, direction0=TX/1=RX. */
	const uint8_t data[] = {0, direction, current, previous};
	publish_event(1, data, sizeof data);
}
static uint32_t search_started;
static unsigned normal_search, search_candidate_seen;
static uint32_t search_candidate_started;
static uint64_t search_pll;
static uint64_t channel_plls[AR_RF_MAX_CHANNELS];
static struct ar_command_program command_program;
static struct ar_command_state command_state = {.cached_bank = 3, .board_band = -1};
static unsigned commands_ready;
static struct ar_power_control tx_power;
#ifdef AR_RF_SIMULATED
const struct ar_power_control *ar_rf_power_information(void) { return &tx_power; }
#endif
static struct ar_rf_scan_info scan_info;
static uint32_t scan_rows[AR_RF_MAX_CHANNELS];
static struct ar_scan_scores scan_scores;
static struct ar_scan_history scan_history[AR_RF_MAX_CHANNELS];
static uint64_t scan_plls[AR_RF_MAX_CHANNELS];
static void tune_measurement(void);
const struct ar_rf_scan_info *ar_rf_scan_information(void) { return &scan_info; }
#ifdef AR_RF_SIMULATED
const struct ar_command_program *ar_rf_command_information(void) { return &command_program; }
#endif
/* Distance (stock bb_phy_distc_proc), see distance_sample(). */
static struct { uint32_t raw, ms, irqs, accepted, dropped; uint8_t rejects, valid, bw; } distance;
static struct ar_dev_idle_config dev_config;
static struct ar_binding binding;
static uint8_t dev_candidates[AR_DEV_MAX_CANDIDATES][4], dev_status_a, dev_status_b;
static unsigned host_prepared;
static unsigned host_initialized, host_start_required;
static struct {
	uint8_t channels[AR_RF_DEV4_PREFIX + AR_RF_MAX_CHANNELS * 4];
	size_t length;
	uint32_t frequency, milliseconds;
	uint8_t identity;
	unsigned deferred, valid, has_power;
	struct ar_power_config power;
} host_config;
static struct ar_rf_lifecycle_info lifecycle = {.version = 1};
static struct ar_channel_monitor channel_monitor;
static uint64_t monitor_last_ms;
static uint8_t monitor_interval_ms;
static struct ar_rf_recovery_info recovery = {.version = 2};
static unsigned recovery_reset_pending;
static void recover_host(void);
static unsigned ap_role;
static void ap_command_done(unsigned command, const uint8_t *payload);
static void ap_trace_irq(unsigned index);
static struct ar_rf_ap_trace ap_trace;
static unsigned ap_rearm_pending, ap_rx_late, ap_fch_bad, ap_sweeps;
/* Uplink interrupts handled past the slot this frame, and frames in a row
 * with one; uplink packets at the last halted-bit check. The interrupts are
 * polled: a main loop held 100-400 us by host work makes a timely uplink
 * look late once in a while, and each re-arm holds the sequencer for a
 * frame (both of its pre-encodes lost, the frame sent again: 410 re-arms
 * in 30 s, 2.3% of frames failing where stock has none, bench 2026-11-02).
 * A receive path really behind its slot is late every frame. */
static unsigned ap_late_now, ap_late_run;
static uint32_t ap_halted_packets;
#define AP_LATE_FRAMES 3
/* The ground's slot ends here (us after IRQ 5), and the frame's length:
 * receive events later than the slot mean the receive path fell behind. */
static uint32_t ap_rx_end_us = 2300, ap_frame_us = 5634;
static unsigned ap_frame_mode, ap_frame_pending;   /* 0 standard, 2 short; pending = 0x80 | mode */
/* A switch due without the ground's last TTL (timer, lost link): done at the
 * next IRQ 5, between the broadcast and the receive slot. From the poll loop
 * at any point of the frame, the engine hold could land in a transmission
 * and the broadcast stopped for good (no pre-encodes after a lost link). */
static unsigned ap_frame_at_sweep;
/* Channels on the AP: its whole configured list (GET 10, the ground's
 * command 4 and the host's SET 6 index into it), one in use at a time. A
 * hop is applied at IRQ 5 too: 0x10000 | index. */
static uint8_t ap_channels[AR_RF_MAX_CHANNELS * 4];
static unsigned ap_channel_count, ap_channel_start;
static uint32_t ap_chan_pending, ap_chan_at_sweep, ap_chan_hop, ap_chan_deadline_ms;
static struct ar_ap ap_link;            /* the AP link (defined with its code below) */
static void ap_retune(unsigned index);
static int ap_host_channel(const uint8_t *payload, size_t length);
static void ap_up_measure(uint32_t status);
static void ap_up_observe(unsigned mcs);
static void ap_up_tick(void);
static void ap_hop_feed(uint8_t first, uint8_t second);
static void ap_up_connect(unsigned connected);
static unsigned ap_up_active(void);
static void ap_up_build(void);
static uint8_t ap_up_ready, ap_up_min, ap_up_max = 0xff;   /* SET 0x1b's range */
static void ap_auto_channel(void);
/* Own channel choice (see ap_auto_channel): the HARQ run/window, our hop
 * in flight, the last one for a rollback, the last blind one. */
static struct ar_hop_measure ap_measure = {.newest = 255};
static uint8_t ap_hop_evaluate, ap_chan_hop_auto;
static uint8_t ap_auto_from, ap_auto_hopping, ap_measured_start;
static uint32_t ap_blind_ms, ap_auto_hop_ms;
static uint32_t ap_frame_deadline_ms;
/* Automatic TX power (ar_pwr_auto): the DEV asks the AP for its video
 * power from how it hears it (SET 9 {1, max, min} turns that on); the AP
 * applies such requests inside its host's range (SET 9 {2, max, min}, min
 * below max), and goes back to max when the link drops. A request arrives
 * in every header until its TTL runs out: applied once, at the last. */
static struct ar_pwr_auto pwr_auto;
static uint8_t pwr_auto_ready, ap_pwr_min, ap_pwr_max, ap_pwr_announce, dev_peer_dbm;
static uint32_t ap_pwr_pending, ap_pwr_deadline_ms;
static struct ar_pwr_auto *pwr_auto_context(void)
{
	if (!pwr_auto_ready) { ar_pwr_auto_init(&pwr_auto); pwr_auto_ready = 1; }
	return &pwr_auto;
}
static void ap_apply_frame_mode(unsigned mode);
static void ap_trace_mark(unsigned source, uint32_t a, uint32_t b, uint32_t c);
/* The broadcast's HARQ, as a DEV runs its own (0x22c6ec): the packet IDs the
 * engine asks for at each frame-control interrupt; when they stop moving on
 * (a block resent over and over, as after a lost uplink: nothing confirms
 * it), retransmission goes off for two frames, which drops that block. */
static struct ar_dev_retx ap_retx;
#define AP_RETX_THRESHOLD 4   /* when the configuration has none */
static unsigned ap_bw_code = 2, ap_bw_pending, ap_bw_at_sweep;   /* at_sweep = 0x80 | code */
/* A width switch's quiet: IRQ 5s left in which the broadcast carries no
 * stream bytes. From the header write that leaves command 5 at TTL 0 (the
 * switch is at the next IRQ 5). The ground switches at the same IRQ 5 and
 * listens from its re-arm, so the first frame on the new width carries
 * video again; what the quiet keeps empty is the block the re-arm flushes
 * (ap_bw_prequiet below). Bench, 2026-10-26: with the ground two frames
 * behind, the video in the AP's first frames was lost or came late. */
static unsigned ap_bw_quiet;
/* ... and the frame before it: the AP fills its blocks a frame ahead, so
 * both pre-encodes of the last frame on the old width (3.1 and 0.9 ms
 * before the switch at 20 MHz, 1.7 and 0.6 at 40) are for the next frame,
 * and the re-arm flushes them. Bench, 2026-10-26, AP and ground packet
 * traces paired over 24 switches: exactly those two never arrived, the
 * first carrying video in 16 (the slice lost or damaged), while the
 * first block after the switch always did. From the IRQ 5 that starts
 * that frame (command 5 left at TTL 1) every pre-encode is quiet. */
static unsigned ap_bw_prequiet;
/* Frame stamps sent before this link's login: an Open8030 ground takes a
 * stamp as the AP being Open8030 and from then on times every switch to
 * the IRQ 5 (not only across 40 MHz). */
static uint32_t ap_link_stamps;
/* The broadcast stalled: a frame with no IRQ 17 (its last block out) while
 * a ground is connected. Bench, 2026-10-26: ~1 in 100 switches up to 40 MHz,
 * 2-3 frames in, IRQ 7s still coming but no block out and the ground hearing
 * only invalid headers until the link was lost; one came back by itself
 * after two frames (the watchdog's retransmission off). The guard does that
 * at the second such frame, re-arms the TX engine at the fourth. */
static unsigned ap_frame_17, ap_stall_frames, ap_stall_off;
static unsigned ap_bw_aligned(unsigned code)
{
	return code == 5 || ap_bw_code == 5 || phy_control.stamps - ap_link_stamps >= 2;
}
#ifdef AR_BW_TRACE
/* Bench: the AP's side of a width switch, as event 2 (stock's 4 bytes; the
 * air host logs it): 0xa5, 0x10 + item, the value in 8 us, low byte first.
 * Items: 0 IRQ 5 to re-arm, 1 the engine hold, 2 the quiet's IRQ 17 before
 * the IRQ 5, 3 the first pre-encode after the switch, 4-11 the next eight
 * IRQ 5s (times from the switch's IRQ 5), 12 the IRQ 5 spacing before it,
 * 13-20 the next eight IRQ 17s (the broadcast's last block out), 21-24
 * the last four pre-encodes before the switch's IRQ 5 (its time minus
 * theirs; bit 15 set when the block carried stream bytes). */
static uint64_t ap_trace_irq17, ap_trace_switch, ap_trace_pre_at[4];
static unsigned ap_trace_after, ap_trace_pre, ap_trace_17, ap_trace_pre_head, ap_trace_pre_stream[4];
/* 25-28: the MAC sequence (low byte) and stream length /256 (high byte) of
 * the last four packets before the switch, 29-32 of the first four after. */
static uint16_t ap_trace_pkt[4];
static unsigned ap_trace_pkt_after;
/* 34: IRQ 7s with the broadcast's entry (0xa48 written) in the 8 frames
 * after the switch, low byte, and all IRQ 7s, high byte; 35-38: the user
 * nibbles (0xe6c [19:16]) of the first IRQ 7s after it. */
static unsigned ap_trace_i7_frames, ap_trace_i7_user8, ap_trace_i7_all, ap_trace_i7_seen, ap_trace_i7_preset;
/* 39: of those user-8 IRQ 7s, how many found 0xa48 still set: low byte at
 * 0xe6c [23:16] 0x08, high byte at 0x28. */
/* 40-42: a broadcast stall (ap_stall_frames): the four frames up to it and
 * the six after, three items a frame. 40 = IRQ 17s [1:0], user-8 IRQ 7s
 * [3:2], IRQ 11s [6:4], user-8 IRQ 18s [9:7], IRQ 13s [12:10], 0xb20 bit 30
 * [13], 0xf70 [7:6] both set [14], 0xa48 set [15] (at the IRQ 5); 41 = the
 * longest poll gap in 8 us [7:0], the watchdog's repeated [10:8] and phase
 * [12:11], all IRQ 7s [15:13]; 42 = the last user-8 request ids [3:0] and
 * [7:4] (15: none), the watchdog's last id [11:8], frames since the switch
 * [15:12]. 43-45: the same for the four frames after every switch. 46: the
 * guard's action (1 retransmission off, 2 TX re-arm, 3 back on) | frames
 * since the switch << 8. */
static unsigned ap_tf_7, ap_tf_7u8, ap_tf_11, ap_tf_18u8, ap_tf_13, ap_tf_gap, ap_tf_req = 0xff, ap_tf_since = 15, ap_tf_after, ap_tf_head;
static uint16_t ap_tf_ring[4][3];
static void ap_trace_raw(unsigned item, uint16_t v)
{
	const uint8_t mark[] = {0xa5, (uint8_t)(0x10 + item), (uint8_t)v, (uint8_t)(v >> 8)};
	publish_event(2, mark, sizeof mark);
}
static void ap_trace_item(unsigned item, uint64_t ticks)
{
	uint32_t v = (uint32_t)(ticks / 320);
	const uint8_t mark[] = {0xa5, (uint8_t)(0x10 + item), (uint8_t)v, (uint8_t)(v >> 8)};
	publish_event(2, mark, sizeof mark);
}
#endif
/* The ground's uplink MCS. Stock asks for it once the identity has moved
 * (command 3 [slot 0, mcs], TTL 31): its ground then sends modulation 0,
 * code 0, rate 0 (0xb00 0x14086021 in stock's linked dumps). Left alone, it
 * stays on its first key (rate 1, 0x14093021), which halves its budget: 65
 * bytes a standard frame, 29 a short one - too few for its report, so the
 * short frame lost the link. TTL 31 repeats until cancelled and the PHY
 * block holds one command, so the request is held AP_UPLINK_MCS_MS. */
static unsigned ap_uplink_mcs_pending, ap_uplink_mcs_max = 0xff;   /* SET 0x1b's top, 0xff: key 0's index */
/* Command 6 (the identity switch) still to send: the block was busy at the
 * login (the frame stamp holds it one frame in every 100 ms). */
static unsigned ap_identity_pending;
/* The host's channel (SET 6) still to send as command 4: 0x10000 | index. */
static uint32_t ap_chan_host_pending;
static uint32_t ap_uplink_mcs_ms;
#define AP_UPLINK_MCS_MS 100
static void ap_apply_bandwidth(unsigned code);
static int apply_power(unsigned user, unsigned level);
static uint8_t power_level[AR_POWER_USERS];   /* defined with the power state below */
static void ap_set_mcs(unsigned mcs);
static uint32_t ap_sweep_us;
/* Test builds: the same interrupt trace on a DEV, frozen 140 interrupts after
 * it admits an AP (what a ground sees of the broadcast, frame by frame). */
#ifdef AR_DEV_TRACE
#define TRACE_ON 1
#else
#define TRACE_ON ap_role
#endif
static int automatic_recovery(void) { return receive_operation == 76 && !receive_milliseconds && !ap_role; }
static int start_prepared_receive(void);
static void stop_receive(void);
static int failure(int code);
static int configure_monitor(uint32_t frequency, uint32_t milliseconds, uint8_t identity_hash,
	unsigned operation, const uint8_t *channels, size_t length, unsigned deferred,
	const struct ar_power_config *power);
static uint8_t dev_power_index, dev_sync_ack[5];
static struct ar_dev_tx_context dev_tx;
static struct ar_dev_retx dev_retx;
static struct ar_dev_stream dev_stream;
static struct ar_rf_phy_info phy;
/* The last 8 per-frame user-8 samples, as stock's quality record keeps them
 * (0x221f1c averages SNR and both gains over 8; samples older than 200 ms
 * make the reply fail). */
struct quality_sample { uint16_t snr; uint8_t gain_a, gain_b, errored; uint32_t ms; };
static struct quality_sample quality_ring[8];
/* Per frame of ours (the frames' count, & 255), as its header came: the
 * frame info's q_word (ar_rf_device.h), for the host's per-frame log. */
static uint32_t fq_word[256], fq_newest;
static uint8_t fq_have;
static unsigned quality_head, quality_count;
static struct ar_rf_phy_trace phy_trace;
static uint32_t phy_poll_tick;
const struct ar_rf_phy_trace *ar_rf_phy_trace_information(void) { return &phy_trace; }
static struct ar_dev_mcs dev_mcs;
static struct ar_rf_mcs_info mcs_info;
static uint32_t mcs_fch_keys[25];
static uint8_t mcs_sync_mmio;
static struct ar_rf_sync_info sync_info;
static uint32_t elapsed_ms(void);
static void adc_poll(void);
static void power_select_band(uint32_t frequency_khz);
#define ADC_BASE UINT32_C(0x0a108180)   /* the ADC (see adc_start) */
static struct { uint8_t on, phase, channel; uint32_t period_ms, next_ms, samples; int32_t mv[8]; } adc;
const struct ar_rf_sync_info *ar_rf_sync_information(void) { return &sync_info; }
static struct ar_rf_traffic_info traffic_info;
void ar_socket_rx_totals(uint32_t *offered, uint32_t *dropped);
void ar_main_loop_statistics(uint32_t out[6]);
void ar_rf_note_push_time(uint32_t us)
{
	phy_trace.reserved[3] += us; phy_trace.reserved[4]++;
	if (us > phy_trace.reserved[5]) { phy_trace.reserved[5] = us; }
}
const struct ar_rf_traffic_info *ar_rf_traffic_information(void)
{
	ar_socket_rx_totals(&traffic_info.socket_offered, &traffic_info.socket_dropped);
	uint32_t loop[6]; ar_main_loop_statistics(loop);
	traffic_info.loop_iterations = loop[0]; traffic_info.loop_pushes = loop[1]; traffic_info.loop_refused = loop[2];
	traffic_info.loop_max_us = loop[3]; traffic_info.bus_transfers = loop[4]; traffic_info.bus_high_water = loop[5];
	return &traffic_info;
}
static struct ar_rf_tx_mcs_info tx_mcs_info;
/* User-0 drain/application and attempt history. Initialization leaves the retry
 * limit at zero; no runtime retry-limit policy is established yet. */
static struct ar_interleaver tx_interleaver;
static struct ar_rf_tx_history_info tx_history_info;
const struct ar_rf_tx_history_info *ar_rf_tx_history_information(void)
{
	const struct ar_interleaver_user *u = &tx_interleaver.users[0];
	tx_history_info.history = u->history;
	tx_history_info.bookkeeping = u->active | (uint32_t)u->retries << 8 |
		(uint32_t)u->limit << 16 | (uint32_t)u->drain << 24;
	tx_history_info.mcs = u->mcs; tx_history_info.previous_mcs = u->previous_mcs;
	return &tx_history_info;
}
const struct ar_rf_tx_mcs_info *ar_rf_tx_mcs_information(void)
{
	const struct ar_interleaver_user *u = &tx_interleaver.users[0];
	tx_mcs_info.pending = u->active | (uint32_t)u->drain << 8 | (uint32_t)u->mcs << 16;
	return &tx_mcs_info;
}
static struct { uint32_t commands, applied, rejected, last, sent; } chan_info;
static void tune_context_plls(uint64_t pll, uint32_t frequency);
static void ap_receive(const struct ar_mac_frame *frame);
static uint32_t bw_retry_ms;
static unsigned bw_code = 2, bw_state, bw_request_pending, bw_settle_step, chain_snr_fresh, bw_target = 2;
/* The receiver's width policy (dev_width.c): bw_target is what it asks for. */
/* 40 MHz only where MCS 8 holds cleanly: down to 20 under 19 dB (36 x
 * 10^1.9: SNR register units), back at 21 dB read on 20 MHz (36 x 10^2.1; 40
 * MHz reads ~0.5 dB under 20, so ~1.5 dB between the two), and for 4 s after
 * the step only under 16 dB (36 x 10^1.6). Each step into 40 resends ~8x for
 * half a second (9-11% of frames against 1.1-1.5%, flights 2026-10-30) and
 * reads low: at 21 dB half the steps came back to 20 within 0.7-4.2 s. Flights
 * 2026-10-30 per frame: 40 MHz MCS 8 at 18-19 dB 2.6% of frames with errors,
 * 16-17 12%; 20 MHz MCS 10 (31 Mbps, against 34) at 18-19 0.3%, MCS 8 (23.7,
 * more than 40 MHz MCS 6's 22.4) at 16-17 0.1% and 14-15 1.4%. It was 16 /
 * 19.5 dB: at 150-200 m the link sat at 40 or 20 MHz MCS 6 with 8-12 Mbps of
 * video where beta4 (20 MHz, the rows' own MCS) held 11.5-13. Under 20 MHz
 * straight to 20 once MCS 6 holds, whatever the SNR (dev_width.c): flight
 * 2026-10-29 read ~15 dB on 5 MHz, 17-18 on 20, and a relink sat 13 s at 5
 * MHz waiting for 15.2. Back at 21, not 22 (flight 2026-11-01): with the air
 * at 22-24 dBm its own ceiling is ~22.5-23 dB, so 20 MHz read 21-22.9 close
 * by and 40 came back long after the spot it had left at, or not at all at 24
 * dBm; the 4 s hold keeps a step that reads low at 40. */
static struct ar_width_config bw_policy = {.min = 2, .max = 4, .narrow_mcs = 5, .climb_mcs = 8,
	.narrow_snr_40 = 2859, .widen_snr_40 = 4532,
	.narrow_snr_40_hold = 1433, .hold_40_ms = 4000,
	.narrow_ms = 200, .widen_ms = 1000, .settle_ms = 500};
/* At 40 MHz no MCS above 8: over it a block in ten or more was sent again at
 * any SNR on the bench (sweep with 0xbac clear, 2026-10-27: MCS 10 2.2 and
 * MCS 11 23 blocks per 100 frames, MCS 8 and under ~0), a frame or more of
 * latency each. The host's ceiling holds under it. */
#define CEILING_40 (8u + 2u)   /* internal = stock + 2 */
static unsigned host_ceiling;   /* internal MCS the host asked for (0: none) */
/* Wider to 40 MHz wanted from over CEILING_40: its cap first, on the width the
 * link is on, and the switch once the AP sends under it. Widened at MCS 11
 * the AP sent it on 40 MHz 4 dB down for 0.6 s - errored frames, a glitch
 * (flight 2026-11-06). */
static unsigned bw_precap;
static uint32_t bw_precap_ms;
/* MCS 8 and 10 by width. At 40 MHz MCS 8 down under 17 dB and up from 6 at
 * 18.5 (its row: 14.2 and 15.2) - under 19 the link is at 20 MHz anyway. At
 * 20 MHz MCS 8 on its row's own values (flight 2026-10-30, beta4: clean to
 * ~15 dB) and MCS 10 down under 17 and up at 18.5 (its row: 16.1 and 17.1;
 * 16-17 dB 9-17% of frames with errors, 18-19 0.3%) - with 40 MHz allowed;
 * with Max Bandwidth 20 its row's own, as beta4. Under 20 MHz the rows' own
 * values (no data). */
#define MCS_ROW_8 (8u + 2u)    /* internal */
#define MCS_ROW_10 (10u + 2u)
#define WIDE_DOWN 1805u        /* 36 x 10^1.7 */
#define WIDE_UP 2550u          /* 36 x 10^1.85 */
static uint16_t row_own[2][2];   /* MCS 8 / 10 rows' own down, up (0: not seen) */
static void mcs_rows_for_width(void)
{
	for (unsigned i = 0; i < dev_mcs.config.count && i < AR_DEV_MCS_ROWS; i++) {
		struct ar_dev_mcs_row *row = &dev_mcs.config.rows[i];
		const unsigned k = row->mcs == MCS_ROW_8 ? 0 : row->mcs == MCS_ROW_10 ? 1 : 2;
		if (k > 1) { continue; }
		if (row->snr_down != WIDE_DOWN || row->snr_up != WIDE_UP) { row_own[k][0] = row->snr_down; row_own[k][1] = row->snr_up; }
		/* MCS 10's at 20 MHz only with 40 allowed: with Max Bandwidth 20 the
		 * rows' own, as beta4 ran (16.1 / 17.1 dB). */
		const int wide = k == 0 ? bw_code >= 5 : bw_code == 4 && bw_policy.max >= 5;
		if (wide) { row->snr_down = WIDE_DOWN; row->snr_up = WIDE_UP; }
		else if (row_own[k][0]) { row->snr_down = row_own[k][0]; row->snr_up = row_own[k][1]; }
	}
}
static void apply_ceiling(void)
{
	mcs_rows_for_width();
	unsigned c = host_ceiling;
#ifndef AR_NO_CEILING_40   /* test builds: every MCS at 40 MHz (how far the link goes) */
	if ((bw_code == 5 || bw_precap) && (!c || c > CEILING_40)) { c = CEILING_40; }
#endif
	ar_dev_mcs_set_ceiling(&dev_mcs, c);
}
static uint8_t bw_widen_mcs = 8;   /* internal: the row whose snr_up widening keeps */
static struct ar_width_state bw_width;
static unsigned bw_automatic, bw_manual = 2;
static void bw_policy_rows(void);
/* Open8030's frame stamps (ar_frame_stamp.h): our IRQ 5s on both roles, the
 * AP's clock on a ground. */
static struct ar_frame_stamp fstamp;
static struct ar_rf_frame_info frame_stamp_info;
/* The AP's resend maps (AR_PHY_RETRIES) on a ground, per frame of ours (the
 * frames' count; & 255): 0..2 blocks sent again, 3 none confirmed, 0xff not
 * known. retry_newest: our frame of the newest known; retry_last_ap: the AP's
 * index of the last map on this link (retry_link). Totals since start. */
static uint8_t retry_ring[256], retry_last_ap, retry_link, retry_have;
static uint32_t retry_newest, retry_blocks, retry_known_frames, retry_stalls;
/* AP: blocks sent again in this frame (its HARQ request ids at IRQ 18),
 * whether one came at all, and the map of the frames before (two bits each,
 * the newest low) with the newest's index. */
static unsigned ap_frame_resent, ap_frame_fed;
static uint32_t ap_retry_map;
static uint8_t ap_retry_frame;
static uint32_t phy_last_invalid, phy_last_mac_invalid;
static uint32_t bw_deadline_ms, bw_settle_us;
static uint32_t bw_mcs_resume_ms;
static unsigned bw_mcs_settling;
static uint8_t dev_bw_request[5];
static struct { uint32_t requests, sent, applied, announced, commands, last, ttl_mask, wide_ok, wide_bad, wide_last_ok, wide_last_bad; } bw_info;
static unsigned frame_mode, frame_pending;   /* 0 standard, 2 short (TDD frame mode, SET 16); pending = 0x80 | mode */
/* A host's frame mode that met the command slot busy (0x80 | mode): sent as
 * the slot frees. At link-up SET 16 met the width request there and was
 * refused (-16); the host does not ask again, so the link ran the
 * standard frame (bench, 2026-10-26). */
static unsigned frame_request;
/* The frame mode the host asked for on this link (0x80 | mode, 0: none) and
 * the retries left: a command 8 can still be lost after it went out (the
 * slot cleared under it), so it goes again while ours differs - up to
 * FRAME_RETRIES a link, a second apart. Flight 2026-10-28: SET 16 answered
 * 0, the link ran the standard frame (26 Mbit/s at 40 MHz MCS 8, not 34). */
#define FRAME_RETRIES 5u
static unsigned frame_want, frame_retries;
static uint32_t frame_retry_ms;
static struct { uint32_t requests, applied, from_ap, last, retries; } frame_info;
static unsigned sync_timer_apply, bw_at_sweep;   /* bw_at_sweep = 0x80 | code */
/* A switch across 40 MHz (only an Open8030 AP makes one), or any switch
 * once the AP's frame stamps say it is Open8030: 0xc0 | code, and our timer
 * (ticks) past which the IRQ 5 both ends switch at falls. The AP
 * writes each header's TTL a frame ahead, so it switches at the first IRQ 5
 * after the header that carried TTL 2 (the TTL-1 one goes out on the new
 * width): TTL - 1 frames after the IRQ 5 before the header, taken half a
 * frame early. Bench, 2026-10-26: between that header and a switch two
 * frames later the ground heard two headers, both invalid - the AP had
 * moved, and two frames of its video went to a ground on the old width.
 * The robust timer (4 ms after TTL 2's frame, whole milliseconds) lands
 * two frames late and sometimes one, which lost the link. */
static unsigned bw_due_code;
static uint64_t bw_due_ticks;
#ifdef AR_BW_TOGGLE_MS
/* Bench: how the last switch went, in its event 12's first byte - bits 1:0
 * the lowest TTL heard, 2 a receive finished first, 3 an uplink TX end
 * retired, 4 put off a frame, 7:5 the IRQ 5s from the last header that
 * carried the AP's command 5 (0x80 | code in bw_heard) to the switch. */
static unsigned bw_path, bw_heard, bw_sweeps_left;
/* ... and its times (event 2 after the event 12: 0xb5, the header's after
 * the IRQ 5 before it /32 us, the header to the switch /64 us, the longest
 * IRQ 5 gap in between /64 us). */
static uint32_t bw_irq5_us, bw_header_us, bw_header_phase, bw_irq5_gap;
/* ... and how the receiver came back (event 2 0xb6: unmuted after the
 * switch /32 us, released /64 us, 1 by headers 0 by the timer, per step). */
static uint32_t bw_unmute_us, bw_release_us;
static unsigned bw_settle_how;
/* ... and the frames after it (event 2 0xb8: the re-arm after the switch's
 * IRQ 5; 0xb7: the next six broadcast headers, index | valid << 4; times
 * in 8 us, low byte first). */
static unsigned bw_trace_left, bw_trace_i5;   /* 0xb9: the next two IRQ 5s */
/* ... and the broadcast packets around it (0xb4: index, the AP's MAC
 * sequence, length /256; 0-2 the last three before, 3-6 the first four
 * after). */
static uint8_t bw_seq_ring[4], bw_len_ring[4];
static unsigned bw_seq_head, bw_seq_after;
/* ... and the broadcast headers between the last command-5 header and the
 * switch (0xb3: valid ones, invalid ones, IRQ 14s for other users). */
static unsigned bw_pre_valid, bw_pre_invalid, bw_pre_other;
static uint32_t bw_switch_irq5_us;
#endif
const struct ar_rf_mcs_info *ar_rf_mcs_information(void)
{
	mcs_info.samples = dev_mcs.samples; mcs_info.requests = dev_mcs.requests;
	mcs_info.acknowledgements = dev_mcs.acknowledgements; mcs_info.unsolicited = dev_mcs.unsolicited;
	mcs_info.timeouts = dev_mcs.timeouts; mcs_info.advertisements = dev_mcs.advertisements;
	mcs_info.cancellations = dev_mcs.cancellations;
	mcs_info.state = dev_mcs.connected | dev_mcs.ready << 1 | dev_mcs.started << 2 |
		dev_mcs.pending << 3 | dev_mcs.phase << 4 | dev_mcs.advertised << 5 | dev_mcs.direction << 6;
	mcs_info.current = dev_mcs.current < dev_mcs.config.count ? dev_mcs.config.rows[dev_mcs.current].mcs : 25;
	mcs_info.previous = dev_mcs.previous < dev_mcs.config.count ? dev_mcs.config.rows[dev_mcs.previous].mcs : 25;
	mcs_info.observed = dev_mcs.observed; mcs_info.average_snr = dev_mcs.average_snr;
	mcs_info.reserved[0] = bw_code | bw_state << 8 | frame_mode << 16 | (frame_info.applied & 255) << 24;
	mcs_info.reserved[1] = bw_info.requests | bw_info.sent << 16;
	mcs_info.reserved[2] = (bw_info.applied & 255) | (bw_info.commands & 255) << 8 | bw_info.last << 16;
	mcs_info.reserved[3] = (bw_info.announced & 255) | bw_info.ttl_mask << 8;
	mcs_info.padding[0] = chan_info.commands | chan_info.applied << 16;
	mcs_info.padding[1] = chan_info.rejected | chan_info.sent << 16; mcs_info.padding[2] = chan_info.last;
	mcs_info.padding[3] = (bw_info.wide_ok & 0xffff) | bw_info.wide_bad << 16;
	mcs_info.padding[4] = bw_info.wide_last_ok; mcs_info.padding[5] = bw_info.wide_last_bad;
	mcs_info.bad_samples = dev_mcs.bad_samples; mcs_info.recent_bad = dev_mcs.recent_bad;
	mcs_info.sample_ms = dev_mcs.sample_ms; mcs_info.request_ms = dev_mcs.request_ms;
	mcs_info.direction_ms = dev_mcs.direction_ms; mcs_info.sync_mmio = mcs_sync_mmio;
	return &mcs_info;
}
static uint8_t phy_header_valid[10], phy_eop_seen[10];
const struct ar_rf_phy_info *ar_rf_phy_information(void) { return &phy; }
static ar_rf_payload_callback payload_callback;
static ar_rf_tx_peek_callback tx_peek_callback;
static ar_rf_tx_commit_callback tx_commit_callback;
static void *tx_context;
static struct ar_rf_tx_payload_info tx_payload_info;

static ar_rf_tx_peek_at_callback tx_peek_at_callback;
static ar_rf_tx_available_callback tx_available_callback;
void ar_rf_set_tx_staging(ar_rf_tx_peek_at_callback peek_at, ar_rf_tx_available_callback available)
{
	tx_peek_at_callback = peek_at; tx_available_callback = available;
}
void ar_rf_set_tx_callbacks(ar_rf_tx_peek_callback peek, ar_rf_tx_commit_callback commit, void *context)
{
	tx_peek_callback = peek; tx_commit_callback = commit; tx_context = context;
}

const struct ar_rf_tx_payload_info *ar_rf_tx_payload_information(void)
{
	return &tx_payload_info;
}
static void *payload_context;

void ar_rf_set_payload_callback(ar_rf_payload_callback callback, void *context)
{
	payload_callback = callback; payload_context = context;
}
static unsigned dev_active;
static void dev_mcs_apply(unsigned action);
static uint32_t dev_frame_microseconds(void);

#ifdef AR_RF_SIMULATED
extern uint32_t ar_rf_test_read(uint32_t address);
extern void ar_rf_test_write(uint32_t address, uint32_t value);
extern void ar_rf_test_start_dma(uint8_t *buffer, unsigned length);
#define read32 ar_rf_test_read
#define write32 ar_rf_test_write
static uint8_t read8(uint32_t address) { return read32(address & ~3u) >> ((address & 3) * 8); }
static void write8(uint32_t address, uint8_t value)
{
	unsigned shift = (address & 3) * 8;
	write32(address & ~3u, (read32(address & ~3u) & ~(0xffu << shift)) | (uint32_t)value << shift);
}
static void barrier(void) { }
#else
static uint32_t read32(uint32_t address) { return *(volatile uint32_t *)(uintptr_t)address; }
static void write32(uint32_t address, uint32_t value) { *(volatile uint32_t *)(uintptr_t)address = value; }
static uint8_t read8(uint32_t address) { return *(volatile uint8_t *)(uintptr_t)address; }
static void write8(uint32_t address, uint8_t value) { *(volatile uint8_t *)(uintptr_t)address = value; }
static void barrier(void) { __asm__ volatile("fence rw,rw" ::: "memory"); }
#endif

/* The hardware counter is 64 bits at 40 MHz. Read a coherent high/low pair
 * rather than dividing a truncated 32-bit difference (wraps every 107 s).
 * Diagnostic microsecond fields retain their ABI and wrap at 2^32 us; control
 * timers use milliseconds directly, never a truncated microsecond timestamp. */
static uint64_t timer_ticks(void)
{
	uint32_t high, low, again;
	do {
		high = read32(TIMER + 4); low = read32(TIMER); again = read32(TIMER + 4);
	} while (high != again);
	return (uint64_t)high << 32 | low;
}
static uint64_t elapsed_ticks(void) { return timer_ticks() - receive_started; }
static uint32_t elapsed_us(void) { return (uint32_t)(elapsed_ticks() / 40); }

static struct ar_rf_capture_info capture = {.version = 1};
static uint8_t capture_last_full[48], capture_last_compact[6], capture_last_phy[7], capture_last_layout[16];
const struct ar_rf_capture_info *ar_rf_capture_information(void)
{
	capture.dev_state = receiver.dev_state; capture.assigned_slot = receiver.assigned_slot;
	capture.tx_preencode = receiver.tx_preencode; capture.tx_packets = receiver.tx_packets;
	capture.tx_errors = receiver.tx_errors; capture.tx_budget = receiver.last_tx_budget;
	capture.tx_user = receiver.last_tx_user;
	capture.lock_result = receiver.last_lock_result; capture.login_packets = receiver.login_extension_packets;
	capture.login_matching = receiver.login_matching_packets; capture.sync_unsupported = receiver.sync_unsupported;
	capture.dev_active = dev_active; capture.admission_result = receiver.last_admission_result;
	capture.lock_messages = receiver.lock_messages; capture.sync_acks = receiver.sync_ack_submitted;
	return &capture;
}
static void capture_add(unsigned kind, const uint8_t *data, size_t length, uint32_t extra)
{
	struct ar_rf_capture_entry *e = &capture.entries[capture.head % AR_RF_CAPTURE_ENTRIES];
	e->seq = capture.head++; e->elapsed_us = elapsed_us(); e->extra = extra;
	e->kind = (uint16_t)kind; e->length = (uint16_t)length;
	memset(e->data, 0, sizeof e->data);
	memcpy(e->data, data, length < sizeof e->data ? length : sizeof e->data);
	capture.recorded++;
}
/* A received MAC packet: kind 1 full control, 2 compact, 3 none. Recorded when
 * its control (sequence and check bytes left out) or its descriptor layout
 * (count, ports and flags) changed, and every 512th packet. */
static void capture_packet(const uint8_t *bytes, size_t length, const struct ar_mac_frame *frame)
{
	capture.packets++;
	unsigned kind = frame->outband_length == 6 ? 2 : frame->outband_length ? 1 : 3;
	if (kind == 1) { capture.full++; } else if (kind == 2) { capture.compact++; } else { capture.other++; }
	capture.outband_lengths[frame->outband_length < 8 * 32 ? frame->outband_length / 32 : 7]++;
	uint8_t layout[16] = {(uint8_t)frame->descriptor_count, (uint8_t)frame->outband_length};
	for (unsigned i = 0; i < frame->descriptor_count && i < 7; i++) {
		layout[2 + 2 * i] = bytes[8 + 4 * i];
		layout[3 + 2 * i] = bytes[8 + 4 * i + 1] & 0x0f;
	}
	unsigned changed = memcmp(layout, capture_last_layout, sizeof layout) != 0;
	if (kind == 1) {
		uint8_t key[48] = {0}; size_t n = frame->outband_length < sizeof key ? frame->outband_length : sizeof key;
		memcpy(key, frame->outband, n); key[1] = key[2] = 0;
		changed |= memcmp(key, capture_last_full, sizeof key) != 0; memcpy(capture_last_full, key, sizeof key);
	} else if (kind == 2) {
		uint8_t key[6]; memcpy(key, frame->outband, 6); key[0] = key[1] = 0;
		changed |= memcmp(key, capture_last_compact, 6) != 0; memcpy(capture_last_compact, key, 6);
	}
	memcpy(capture_last_layout, layout, sizeof layout);
	if (changed || !(capture.packets & 511)) { capture_add(kind, bytes, length, (uint32_t)length); }
}
static void capture_phy(const uint8_t control[7], uint32_t header)
{
	capture.phy++; capture.phy_commands[control[1] & 15]++;
	uint8_t key[7]; memcpy(key, control, 7); key[0] = 0;
	unsigned changed = memcmp(key, capture_last_phy, 7) != 0;
	memcpy(capture_last_phy, key, 7);
	if ((control[1] & 15) == 7) { changed = !(capture.phy_commands[7] & 63); }
	if (changed) { capture_add(4, control, 7, header); }
}
static uint32_t elapsed_ms(void) { return (uint32_t)(elapsed_ticks() / 40000); }

static uint32_t le32(const uint8_t *data)
{
	return data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}
static void update(uint32_t address, uint32_t clear, uint32_t set)
{
	uint32_t value = (read32(address) & ~clear) | set;
	write32(address, value);
	barrier();
	if (!commands_ready) { return; }
	/* Mirror the profile fields consumed by the original metadata getters.
	 * Updates preserve first-match behavior and unrelated entry fields. */
	unsigned offset = address - BB;
	if (offset >= 0x9a0 && offset <= 0x9f8 && !(offset & 7)) {
		unsigned user = (offset - 0x9a0) / 8;
		if (clear & 0x700) { (void)ar_command_update(&command_program,1,user,1,0,value >> 8,0); }
		if (clear & 0xc7) {
			unsigned type = value & 7, mode = value >> 6 & 3;
			unsigned flag = (type == 1 && mode < 2) || (type == 2 && mode >= 2);
			(void)ar_command_update(&command_program,1,user,1,4,0,flag);
		}
	} else if (offset >= 0xb00 && offset <= 0xb2c && !(offset & 3) && offset != 0xb28 && (clear & 7)) {
		(void)ar_command_update(&command_program,1,(offset - 0xb00) / 4,0,0,value,0);
	} else if (offset == 0xb60 && (clear & 0x07000000)) {
		(void)ar_command_update(&command_program,1,10,0,0,value >> 24,0);
	}
}
static void field(uint32_t encoded, uint32_t value)
{
	unsigned width = encoded & 31, shift = (encoded >> 8) & 31;
	uint32_t mask = encoded & 0xe0 ? UINT32_MAX : (UINT32_C(1) << width) - 1;
	update(BB + (encoded >> 16), mask << shift, (value & mask) << shift);
}
static uint32_t tx_mcs_read(uint32_t encoded, void *context)
{
	(void)context;
	return (read32(BB + (encoded >> 16)) >> ((encoded >> 8) & 31)) &
		((UINT32_C(1) << (encoded & 31)) - 1);
}
static uint32_t tx_mcs_word(uint32_t offset, void *context)
{
	(void)context; return read32(BB + offset);
}
static void tx_mcs_field(uint32_t encoded, uint32_t value, void *context)
{
	(void)context; field(encoded,value);
}
static void tx_mcs_event(unsigned event, const uint8_t payload[9], void *context)
{
	(void)context; publish_event(event,payload,9);
}
static const struct ar_tx_mcs_ops tx_mcs_ops = {tx_mcs_read,tx_mcs_word,tx_mcs_field,tx_mcs_event};
static const struct ar_tx_irq_ops tx_history_ops = {.word=tx_mcs_word,.read=tx_mcs_read};
static void tx_mcs_applied(unsigned mcs)
{
	tx_mcs_info.after = read32(BB + 0xb00);
	if (((tx_mcs_info.before >> 22) & 3) != ((tx_mcs_info.after >> 22) & 3))
		tx_mcs_info.mode_changed++;
	mcs_event(0,mcs,tx_mcs_info.current); tx_mcs_info.current = mcs;
	tx_mcs_info.last_apply_us = elapsed_us();
	if (!tx_mcs_info.applied++) { tx_mcs_info.first_apply_us = tx_mcs_info.last_apply_us; }
}
static void tx_mcs_drain_apply(unsigned user, unsigned mcs, void *context)
{
	(void)context;
	tx_mcs_info.before = read32(BB + 0xb00); tx_mcs_info.key = mcs_fch_keys[mcs];
	if (ar_tx_mcs_control(&tx_interleaver,AR_TX_MCS_DRAIN_APPLY,user,mcs,
	    mcs_fch_keys,information.chip_profile,1,&tx_mcs_ops,NULL)) {
		tx_mcs_info.rejected++; return;
	}
	tx_mcs_info.drained++; tx_mcs_applied(mcs);
}
static void tx_mcs_retry_reset(unsigned user, void *context)
{
	(void)context;
	(void)ar_tx_mcs_control(&tx_interleaver,AR_TX_MCS_RETRY_RESET,user,0,
		mcs_fch_keys,information.chip_profile,1,&tx_mcs_ops,NULL);
}
static const struct ar_interleaver_ops tx_step_ops = {
	.apply=tx_mcs_drain_apply,.retry_reset=tx_mcs_retry_reset
};
/* Link bandwidth. Every stock link starts on the configured 5 MHz broadcast
 * profile (code 2). The side that transmits owns the switch: it pauses its TX
 * (0x22e0d0, TX word bit 30), sends PHY command 5 with TTL 3 carrying
 * [type 0, slot | code << 5, 0], and both ends apply the new profile when the
 * TTL expires (0x238960; link messages 0x306/0x307). A receiver asks for a
 * wider downlink the way the stock host's BB_SET_BANDWIDTH dir=RX does
 * (0x24ed72): port-0 internal command 9 with the one-byte SDK code, which the
 * AP's 0x234ee6 turns into its own 0x306. */

static int hold_engines(void);
static void release_engines(void);
static void bandwidth_settle(unsigned by_header);
static void bandwidth_mcs_settle(uint32_t lead_ms)
{
	bw_mcs_resume_ms = elapsed_ms() + lead_ms + bw_policy.settle_ms;
	bw_mcs_settling = 1;
	dev_mcs.ready = dev_mcs.index = dev_mcs.direction = 0;
}
/* aligned: at the IRQ 5 the AP switches at (timed, or across 40 MHz). */
static void apply_bandwidth(unsigned code, unsigned aligned)
{
	unsigned previous = bw_code;
	if (code > 5) { return; }
	/* Changing or restarting the receive producer while its DMA is owned
	 * can remove the completion that shutdown needs to drain the buffer.
	 * An uplink packet still owned here is the last frame's (its slot
	 * follows this IRQ 5) whose TX end never came: at the AP's IRQ 5, where
	 * a frame's wait is a frame off the AP (across 40 MHz the link lost),
	 * it is retired instead. */
	unsigned across = aligned || code == 5 || previous == 5;
	if (receive_pending || (dev_tx.owned && !across)) {
#ifdef AR_BW_TOGGLE_MS
		bw_path |= 16;
#endif
		bw_at_sweep = 0x80 | code; return;
	}
	int result = hold_engines();
	if (result) { stop_receive(); (void)failure(result); return; }
	if (dev_tx.owned) {
		lifecycle.cancelled_tx += dev_tx.owned;
		(void)ar_dev_tx_retire(&dev_tx, 0); receiver.tx_owned = 0;
#ifdef AR_BW_TOGGLE_MS
		bw_path |= 8;
#endif
	}
	/* 0x238960 (received): mute RX8 (0x22ce78(8,1): 0x9e4 bit 24) and hold
	 * its path (0xcfc bit 23 for buffer user 10, 0xbf8 bit 7 for user 8)
	 * before the profile change; the deferred step 0x238842 releases them
	 * over the next two frames (bandwidth_poll). */
	field(0x9e41801, 1); field(0xcfc1701, 1); field(0xbf80701, 1);
	/* The receiving side changes only the RX8 bandwidth field (0x22c6f6);
	 * the slot-0 TX word stays at the configured uplink bandwidth, which the
	 * AP keeps receiving. The AP's broadcast keeps its symbol structure
	 * (25933 kbps at 20 MHz against 6483 at 5 MHz is exactly four times the
	 * bytes in the same burst), so the configured selector/interleave and
	 * the 3000 us window (0x1250) stay: the frame remains 3 x 100 + 3000 +
	 * 100 + 2234 us. (Mode 0 also clears the 40 MHz bits in 0xbac/0xbd4 via
	 * 0x21b1ae(1); never set on this path.) */
	field(0x9e00803, code);
	/* Mode 0: 40 MHz sets the receiver's 0xbac[31:14,7:6] / 0xbd4[17:8] bits
	 * (0x21b1ae(0)), any other width clears them (0x21b1ae(1)). 40 MHz is also
	 * the one width that changes the frame: the RX8 window [31:16] is 0x09c4
	 * (1600 us) instead of 0x1250 (3000 us). Nothing else: stock writes no
	 * clock or front-end register for 40 (stock dumps at 5/20/40 MHz,
	 * 2026-10-18; the other words that differ are live AGC/calibration state). */
	/* 0xbac's bits stay clear at 40 MHz too: set (stock's "kikp" receive
	 * mode), a block in 15 was sent again at MCS 8, clear one in 150-250 (A/B
	 * on a live link, 2026-10-27; 0xbd4's bits changed nothing). */
	if (code == 5) {
		update(BB + 0xbac, 0xffffc0c0, 0); update(BB + 0xbd4, 0x0003ff00, 0x0003ff00);
		field(0x9701010, 0x09c4);
	} else {
		update(BB + 0xbac, 0xffffc0c0, 0); update(BB + 0xbd4, 0x0003ff00, 0);
		field(0x9701010, 0x1250);
	}
	/* Re-arm the running sequence on the new width, as a frame-mode change
	 * does: 40 MHz changes the frame (the broadcast window; stock tears the
	 * link down here instead and searches again, ~1 s), and 5 -> 20 without
	 * it lost the link every time (bench, 2026-10-19). */
	release_engines();
	bw_settle_step = 1; bw_settle_us = elapsed_us();
	/* At the AP's IRQ 5 both ends re-arm together and the AP's first
	 * burst comes ~2.6 ms later, with nothing of the old width left to come
	 * (the receive was settled before the hold): the receiver comes back at
	 * once. Muted, or held, until that burst's header, its first block was
	 * lost (bench: one more packet missing at every switch). */
	if (across && !receive_pending) {
		field(0x9e41801, 0); bw_settle_step = 2; bandwidth_settle(1);
	}
	bandwidth_mcs_settle(0);
	/* Diagnostics: the frame registers at the switch (sync reserved[3..5]). */
	sync_info.reserved[3] = read32(BB + 0x970);
	sync_info.reserved[4] = (read32(BB + 0xa00) & 0xffff) | read32(BB + 0x984) << 16;
	sync_info.reserved[5] = (read32(BB + 0x980) & 0xffff) | (read32(BB + 0x9e0) & 0xffff) << 16;
	bw_code = code; bw_info.applied++;
	apply_ceiling();                       /* MCS 8 at most at 40 MHz */
	receiver.search_frame_us = dev_frame_microseconds();
#ifdef AR_BW_TOGGLE_MS
	bw_unmute_us = bw_release_us = 0;
	{
		bw_switch_irq5_us = bw_irq5_us;
		{
			const uint8_t pre[] = {0xb3, (uint8_t)bw_pre_valid, (uint8_t)bw_pre_invalid, (uint8_t)bw_pre_other};
			publish_event(2, pre, sizeof pre);
		}
		uint32_t rearm = (elapsed_us() - bw_irq5_us) / 8;
		const uint8_t mark[] = {0xb8, 0, (uint8_t)rearm, (uint8_t)(rearm >> 8)};
		publish_event(2, mark, sizeof mark); bw_trace_left = 6; bw_trace_i5 = 2;
		for (unsigned i = 0; i < 3; i++) {
			unsigned k = (bw_seq_head + 2 + i) & 3;   /* the newest three, oldest first */
			const uint8_t seq[] = {0xb4, (uint8_t)i, bw_seq_ring[k], bw_len_ring[k]};
			publish_event(2, seq, sizeof seq);
		}
		bw_seq_after = 4;
	}
#endif
	/* Stock event 12: [slot, received, new, old]. */
	uint8_t data[] = {0, 1, (uint8_t)code, (uint8_t)previous};
#ifdef AR_BW_TOGGLE_MS
	data[0] = (uint8_t)(bw_path | bw_sweeps_left << 5); bw_path = 0; bw_heard = bw_sweeps_left = 0;
#endif
	publish_event(12, data, sizeof data);
#ifdef AR_BW_TOGGLE_MS
	{
		uint32_t since = elapsed_us() - bw_header_us;
		const uint8_t times[] = {0xb5, (uint8_t)(bw_header_phase / 32 > 255 ? 255 : bw_header_phase / 32),
			(uint8_t)(since / 64 > 255 ? 255 : since / 64), (uint8_t)(bw_irq5_gap / 64 > 255 ? 255 : bw_irq5_gap / 64)};
		publish_event(2, times, sizeof times);
		bw_irq5_gap = 0;
	}
#endif
}

/* The receiver comes back at the first valid header on the new width:
 * unmuted, its holds released and the buffer user restarted (0xd08 bit 16,
 * 0x22459e). Stock (0x238842) releases the holds a header later; with the
 * AP's first frame on the new width empty (ap_bw_quiet) nothing is in the
 * path to hold, and holding the next one put its video a frame late: to
 * 40 MHz, where the first header heard is the second frame's, slices came
 * ~8 ms late, a picture past its refresh (artifacts, bench 2026-10-26).
 * Waiting a frame on our timer lost the first frame's blocks.
 * by_header: from a valid header (else the timer's fallback, one step). */
static void bandwidth_settle(unsigned by_header)
{
	if (bw_settle_step == 1) {
		field(0x9e41801, 0); bw_settle_step = 2;
#ifdef AR_BW_TOGGLE_MS
		bw_unmute_us = elapsed_us() - bw_settle_us; bw_settle_how = by_header;
#endif
		if (!by_header) { return; }
	}
	if (bw_settle_step == 2 && !receive_pending) {
		field(0xcfc1701, 0); field(0xbf80701, 0);
		field(0xd081001, 1); field(0xd081001, 0);
		bw_settle_step = 0;
#ifdef AR_BW_TOGGLE_MS
		bw_release_us = elapsed_us() - bw_settle_us; bw_settle_how |= by_header << 1;
		const uint8_t times[] = {0xb6, (uint8_t)(bw_unmute_us / 32 > 255 ? 255 : bw_unmute_us / 32),
			(uint8_t)(bw_release_us / 64 > 255 ? 255 : bw_release_us / 64), (uint8_t)bw_settle_how};
		publish_event(2, times, sizeof times);
#endif
	}
}

/* Channel commands index the validated 5-GHz catalog. PLL banks and
 * preparation metadata must change together, only in the named direction. */
static void dev_mcs_apply(unsigned action);
static void serial_word(unsigned offset, uint64_t word);
static int retune_user(unsigned index, unsigned user, unsigned direction)
{
	uint64_t pll;
	if (index >= receiver.channel_count || ar_rf_pll_word(receiver.channels[index].frequency_khz, &pll)) { return -22; }
	/* Supported mode-0 paths: uplink TX0, broadcast TX8 and downlink RX8.
	 * A direction-specific command must not move any other PLL bank. */
	unsigned offset = direction ? 0x905 : user == 0 ? 0x910 : 0x900;
	uint32_t frequency = receiver.channels[index].frequency_khz;
	serial_word(offset, pll);
	field(((0x9a4 + user * 8) << 16) | (direction ? 0x1903 : 0x1d03), 4);
	tuned_frequency[user][direction] = frequency;
	if (commands_ready) {
		(void)ar_command_update(&command_program,1,user,direction,1,2,0);
		(void)ar_command_update(&command_program,1,user,direction,3,frequency,0);
	}
	if (direction) {
		field(0xa900010, frequency / 2000);
		receiver.channel_index = index; dev_config.receive_channel = index;
		receiver.frequency_khz = frequency; receiver.channels[index].visits++;
	} else if (!user) { dev_transmit_channel = index; power_select_band(frequency); }
	return 0;
}
static int retune_channel(unsigned index, unsigned slot, unsigned direction)
{
	unsigned previous = direction ? receiver.channel_index : dev_transmit_channel;
	int result = retune_user(index, direction ? 8 : 0, direction);
	if (result) { return result; }
	chan_info.applied++;
	const uint8_t data[] = {(uint8_t)slot, (uint8_t)direction, (uint8_t)index, (uint8_t)previous};
	publish_event(2, data, sizeof data);
	return 0;
}
/* Spectators accept only the AP's receive-side channel/width controls.
 * Never call sync_receive: that path also owns admission and transmit state. */
static void listener_control(const uint8_t control[7])
{
    unsigned cmd=control[1]&15, life=(control[1]>>4)|((control[2]&1)<<4);
    if (!listener_follow || (cmd!=4 && cmd!=5)) { return; }
    if (control[0]!=ar_phy_control_checksum(control) || ((control[2]>>1)&7)!=3 || !life) {
        listener_change.rejected++; return;
    }
    unsigned value;
    if (cmd==4) {
        value=control[4]|(unsigned)control[5]<<8;
        if (!(control[3]&0x20) || (control[3]&0x8f) || value>=receiver.channel_count) {
            listener_change.rejected++; return;
        }
        if (value==receiver.channel_index) { return; }
    } else {
        value=control[4]>>5;
        if (control[3] || (control[4]&31)!=8 || value<2 || value>5) {
            listener_change.rejected++; return;
        }
        if (value==bw_code) { return; }
    }
    listener_change.command=cmd; listener_change.value=value;
    listener_change.due=timer_ticks()+(life==31?0:(uint64_t)(life-1)*receiver.search_frame_us*40);
}
static void listener_poll(void)
{
    if (!listener_follow || !listener_change.command || timer_ticks()<listener_change.due || receive_pending) { return; }
    /* Flush/rearm only with no DMA in flight. hold/release_engines preserve
     * TX hold bit 2; no transmit descriptor, power or uplink PLL is touched. */
    if (hold_engines()) { stop_receive(); return; }
    if (listener_change.command==4) {
        if (!retune_user(listener_change.value,8,1)) { listener_change.hops++; }
    } else {
        unsigned code=listener_change.value;
        field(0x9e00803,code);
        update(BB+0xbac,0xffffc0c0,0);
        update(BB+0xbd4,0x0003ff00,code==5?0x0003ff00:0);
        field(0x9701010,code==5?0x09c4:0x1250);
        bw_code=code; listener_change.widths++;
        receiver.search_frame_us=dev_frame_microseconds();
    }
    field(0x8000201,1);
    release_engines();
    listener_change.command=0;
}
int ar_rf_listener_state(uint32_t output[8])
{
    if (!output) { return -22; }
    output[0]=listener_follow && information.phase==AR_RF_RECEIVING;
    output[1]=receiver.frequency_khz; output[2]=bw_code;
    output[3]=listener_change.hops; output[4]=listener_change.widths;
    output[5]=listener_change.rejected; output[6]=listener_change.command;
    output[7]=receiver.channel_count;
    return 0;
}
/* Received command 4 has independent direction flags even in mode 0:
 * 0x40 moves our slot TX, 0x10 broadcast TX, 0x20 broadcast RX. The 0x80
 * arm applies only to modes 1/3; it must not move mode-0 reception. */
static void sync_channel(const uint8_t control[7])
{
	unsigned index = control[4] | (unsigned)control[5] << 8, flags = control[3];
	chan_info.commands++; chan_info.last = flags | (uint32_t)control[4] << 8 | (uint32_t)control[5] << 16;
	if (!(flags & 0xf0) || (flags & 15) || index >= receiver.channel_count) { chan_info.rejected++; return; }
	if (flags & 0x40) { (void)retune_channel(index, 0, 0); }
	if (flags & 0x10) { (void)retune_user(index, 8, 0); }
	if (flags & 0x20) { (void)retune_channel(index, 0, 1); }
}
/* A host channel set on the node (stock: SET 6 dir RX -> 0x221c48 case 1
 * -> message 0x108 {1, index} -> 0x2314xx builds command 4
 * [0x80 | own slot, index lo, index hi] with TTL 16 into the 7-byte FCH
 * control block at 0xb8c, counted down once per frame by 0x2545xx; at zero
 * the node retunes its broadcast RX user (0x231cc6 local origin) and the AP
 * its TX (remote origin, "set slot %u tx chan %u"). Both ends hop on the
 * same frame; nobody retunes on receipt of the host command itself. The
 * block is shared with the MCS request (dev_mcs.fch), which only enqueues
 * when the command nibble is free. */
/* Commands the DEV originates, one at a time, through its 7-byte control
 * block (shared with the MCS request in dev_mcs.fch, which only enqueues
 * when the command nibble is free): command 4 channel hop (TTL 16) and
 * command 8 frame mode (TTL 5, stock SET 16 -> 0x22d42a(8, 5, mode << 4)).
 * The block goes out with TTL n, then n-1 is stored each frame (0x2545xx);
 * at zero the DEV applies the command itself and clears the nibble. */
static struct { unsigned state, command, ttl, length; uint8_t payload[4]; } dev_cmd;
static void apply_frame_mode(unsigned mode);
/* Our command out (state 2) but no longer in the slot: something cleared it
 * (it never expires then) - free, so the next one goes. */
static void dev_cmd_check(void)
{
	if (dev_cmd.state == 2 && (phy_control.tx[1] & 15) != dev_cmd.command) { dev_cmd.state = 0; }
}
static int dev_command_request(unsigned command, unsigned ttl, const uint8_t *payload, unsigned length)
{
	dev_cmd_check();
	if (dev_cmd.state) { return -16; }
	dev_cmd.command = command; dev_cmd.ttl = ttl; dev_cmd.length = length;
	memset(dev_cmd.payload, 0, sizeof dev_cmd.payload); memcpy(dev_cmd.payload, payload, length);
	dev_cmd.state = 1;
	return 0;
}
static int dev_hop_request(unsigned index)
{
	if (index >= receiver.channel_count) { return -22; }
	const uint8_t payload[3] = {(uint8_t)(0x80 | (receiver.assigned_slot & 15)), (uint8_t)index, (uint8_t)(index >> 8)};
	int result = dev_command_request(4, 16, payload, 3);
	if (!result) { chan_info.sent++; }
	return result;
}
/* Our own commands to the AP go through upstream's single transmit slot
 * (phy_control): it sends them, counts the TTL down each frame and calls
 * phy_dispatch(outgoing) at zero, where dev_command_done() applies them here.
 * The slot is busy while an MCS request (TTL 31) is out: try again each
 * preencode. state 1 = waiting for the slot, 2 = in it. */
static const struct ar_phy_control_ops phy_ops;
static void dev_hop_poll(void)
{
	dev_cmd_check();
	if (dev_cmd.state != 1 || dev_mcs.advertised) { return; }
	if (ar_phy_control_enqueue(&phy_control, dev_cmd.command, dev_cmd.ttl, dev_cmd.payload,
	    dev_cmd.length, &phy_ops, NULL)) { return; }
	dev_cmd.state = 2;
}
static void dev_command_done(unsigned command, const uint8_t *payload)
{
	if (dev_cmd.state != 2 || command != dev_cmd.command) { return; }
	dev_cmd.state = 0;
	if (command == 4) {
		/* Our own hop: our uplink TX too, as the AP's 0x40 moves it - with
		 * the receiver alone the AP stopped hearing us on the new channel
		 * and the link dropped (re-formed ~1.4 s later, bench 2026-10-26). */
		unsigned index = payload[1] | (unsigned)payload[2] << 8;
		if (retune_channel(index, 0, 0) || retune_channel(index, 0, 1)) { chan_info.rejected++; }
	} else if (command == 8 && !(payload[0] & 15)) {   /* sub-type 0: the frame mode */
		frame_pending = 0x80 | ((payload[0] >> 4) & 7);   /* at the next frame header */
	}
}
static void dev_hop_abort(void)
{
	if (dev_cmd.state == 2) { (void)ar_phy_control_cancel(&phy_control, &phy_ops, NULL); }
	dev_cmd.state = 0;
}
/* TDD frame mode (stock 0x238bc8, mode 0 link). The DEV side of the short
 * frame (mode 2) is the uplink slot: 0xa00 [15:0] 2234 -> 1251 us and the
 * uplink TX profile's time interleave 0xb00 [20:18] 2 -> 1, with the
 * code-length class [15:12] one step shorter (linked register dumps of
 * stock in both modes, 2026-10-16: nothing else on the DEV changes). The
 * frame goes from 5633.92 to 4650.88 us; the broadcast window stays. */
static void apply_frame_mode(unsigned mode)
{
	if (mode != 0 && mode != 2) { return; }
	frame_info.last = mode;
	if (mode == frame_mode) { return; }
	/* The frame restart resets both engines. Keep the old profile intact
	 * until every borrowed RX and submitted TX buffer has retired. */
	if (receive_pending || dev_tx.owned) { frame_pending = 0x80 | mode; return; }
	int result = hold_engines();
	if (result) { stop_receive(); (void)failure(result); return; }
	uint32_t word = read32(BB + 0xb00);
	unsigned code = (word >> 12) & 15, length = code / 3;
	code %= 3;
	if (mode == 2) {
		if (length) { length--; }
		field(0xb001203, 1);
		field(0xa000010, 0x07a3);
	} else {
		if (length < 2) { length++; }
		field(0xb001203, 2);
		field(0xa000010, (2234u * 10000 / 640 + 5) / 10);
	}
	field(0xb000c04, code + length * 3);
	/* The running sequence keeps the old slot until the engines are re-armed:
	 * without this every frame event stopped (TX, headers) and the AP, which
	 * had switched, dropped the link ~2 s later. */
	release_engines();
	frame_mode = mode; frame_info.applied++;
	receiver.search_frame_us = dev_frame_microseconds();
}
int ar_rf_host_frame_mode(const uint8_t *payload, size_t length)
{
	if (!payload || length < 1) { return -22; }
	unsigned mode = payload[0];
	if (mode != 0 && mode != 2) { return -95; }   /* mode 1 (other users' frames) not here */
	/* Refused for now: switching to 2 dropped the link on the bench (the AP
	 * did not follow, or not on the same frame). Under investigation. */
	if (!dev_active || receiver.dev_state != 2) { return -7; }
	frame_info.requests++;
	frame_request = 0;
	frame_want = 0x80 | mode; frame_retries = FRAME_RETRIES; frame_retry_ms = elapsed_ms();
	if (mode == frame_mode) { return 0; }
	const uint8_t word[4] = {(uint8_t)(mode << 4), 0, 0, 0};
	int result = dev_command_request(8, 5, word, 4);
	if (result == -16) { frame_request = 0x80 | mode; return 0; }
	return result;
}
/* Host channel commands (the stock daemon's set_rf_channel sequence):
 * GET 10 lists the configured channels, SET 5 (auto mode) and SET 35 (work
 * list) are recorded, SET 6 {direction, index} retunes like command 4. */
static void serial_word(unsigned offset, uint64_t word);
static uint8_t chan_auto = 1, chan_work_count, chan_work[128];
/* Distance (stock bb_phy_distc_proc): the baseband ranges by itself once
 * 0xd30 is set (window, timeout, slot enables; DEV role 0 in [31:24]) and
 * raises IRQ 6. 0xfe8 [2:0] picks one of eight 24-bit entries, four packed
 * at 0xfd0 and four at 0xfdc: [19:0] the round-trip count (bit 19 = below
 * zero, taken as 0), bits 20, 22, 23 mark an invalid sample. A sample more
 * than 500 counts from the last is an outlier and dropped, unless ten come
 * in a row. Metres = (count - offset - the radios' delay) * 1.5; none
 * after 1 s without one. The count carries the radio filters' delay, which
 * grows as the width narrows: at the same spot (~1 m) with offset 20 stock's
 * formula read 63 m at 40 MHz, 70 at 20 and 111 at 5 (2026-10-30/11-01) -
 * 56 m + 280/MHz, about 185/MHz counts, taken off at the width the sample
 * was ranged at. The 56 m left at every width is the rest of both radios'
 * chains: 37 counts (55.5 m) taken off too, so the metres are the distance
 * itself, with no zero on the host. Measured on a Lite+ only. */
#define DISTANCE_FILTER_COUNTS_MHZ 185u
#define DISTANCE_CHAIN_COUNTS 37u
static uint32_t distance_filter_counts(unsigned code)
{
	static const uint8_t mhz[6] = {0, 0, 5, 10, 20, 40};   /* bandwidth codes 2..5 */
	return code >= 2 && code <= 5 ? (DISTANCE_FILTER_COUNTS_MHZ + mhz[code] / 2u) / mhz[code] : 0;
}
/* One ranging sample is coarse (a count is 1.5 m) and jumps a count or two
 * from frame to frame (detection jitter, reflections): at rest 3 m read 1-6.
 * The report is the mean of the last 400 ms of samples, each with its own
 * width's filter delay taken off, in four 100 ms buckets (whatever the
 * ranging rate). */
static struct { int32_t sum[4]; uint32_t n[4]; uint32_t bucket_ms; uint8_t head; } distance_avg;
static void distance_avg_age(uint32_t now)
{
	if (!distance_avg.bucket_ms) { distance_avg.bucket_ms = now; return; }
	for (unsigned k = 0; k < 4 && now - distance_avg.bucket_ms >= 100; k++) {
		distance_avg.head = (uint8_t)((distance_avg.head + 1) & 3);
		distance_avg.sum[distance_avg.head] = 0; distance_avg.n[distance_avg.head] = 0;
		distance_avg.bucket_ms += 100;
	}
	if (now - distance_avg.bucket_ms >= 100) {   /* more than 400 ms went by */
		memset(&distance_avg, 0, sizeof distance_avg); distance_avg.bucket_ms = now;
	}
}
static uint32_t distance_filter_counts(unsigned code);
static void distance_sample(void)
{
	distance.irqs++;
	unsigned index = read32(BB + 0xfe8) & 7;
	uint32_t base = index < 4 ? 0xfd0 : 0xfdc, at = base + 3 * (index & 3);
	uint32_t e = 0;
	for (unsigned b = 0; b < 3; b++) {
		uint32_t a = at + b;
		e |= ((read32(BB + (a & ~3u)) >> ((a & 3) * 8)) & 255) << (8 * b);
	}
	if (e & (1u << 20 | 1u << 22 | 1u << 23)) { distance.dropped++; return; }
	uint32_t value = e & 0x80000 ? 0 : e & 0xfffff, now = elapsed_ms();
	if (distance.valid && now - distance.ms <= 999) {
		uint32_t diff = value > distance.raw ? value - distance.raw : distance.raw - value;
		if (diff > 500) {
			if (++distance.rejects < 10) { distance.dropped++; return; }
		}
	}
	/* A jump taken as the new level (ten in a row past the outlier limit):
	 * the average starts over from it. */
	if (distance.rejects >= 10) { memset(&distance_avg, 0, sizeof distance_avg); }
	distance.rejects = 0; distance.raw = value; distance.ms = now; distance.valid = 1; distance.bw = (uint8_t)bw_code;
	distance.accepted++;
	distance_avg_age(now);
	const uint32_t filter = distance_filter_counts((unsigned)bw_code);
	distance_avg.sum[distance_avg.head] += (int32_t)value - (int32_t)filter;
	distance_avg.n[distance_avg.head]++;
}
/* The distance in metres (the averaged samples, the radios' delays off), or
 * -1: no sample in the last second, or ranging off. */
static int32_t distance_metres(void)
{
	const uint32_t rest = dev_mcs.config.distance_offset + DISTANCE_CHAIN_COUNTS, now = elapsed_ms();
	if (!distance.valid || !dev_mcs.config.distance_enabled || now - distance.ms > 1000) { return -1; }
	/* The last 400 ms of samples (x16 for the mean's fraction), or with none
	 * in them the latest sample, as before. */
	distance_avg_age(now);
	int32_t sum = 0; uint32_t n = 0;
	for (unsigned k = 0; k < 4; k++) { sum += distance_avg.sum[k]; n += distance_avg.n[k]; }
	const int64_t counts_x16 = n ? (int64_t)sum * 16 / (int64_t)n
		: ((int64_t)distance.raw - (int64_t)distance_filter_counts(distance.bw)) * 16;
	const int64_t over = counts_x16 - (int64_t)rest * 16;
	return over > 0 ? (int32_t)((over * 3) / 32) : 0;
}
int ar_rf_host_distance(const uint8_t *payload, size_t length, uint8_t output[32])
{
	if (!payload || length < 1 || !output) { return -22; }
	memset(output, 0, 32);
	for (unsigned slot = 0; slot < 8; slot++) {
		if (!(payload[0] & (1u << slot))) { continue; }
		/* The DEV keeps one record (slot 0), as stock. */
		const int32_t metres = slot == 0 ? distance_metres() : -1;
		for (unsigned b = 0; b < 4; b++) { output[slot * 4 + b] = (uint8_t)((uint32_t)metres >> (8 * b)); }
	}
	return 0;
}
/* Ours: event 15 {slot 0, 1 = known, metres low, high} - the distance pushed
 * to the host as it changes (at most every 100 ms) and once a second anyway,
 * so a host needs no request for it; 0 known = no distance (stale or off). */
static void distance_push(void)
{
	static uint32_t last_ms; static int32_t last = -2;
	const uint32_t now = elapsed_ms();
	if (now - last_ms < 100) { return; }
	const int32_t m = distance_metres();
	if (m == last && now - last_ms < 1000) { return; }
	last_ms = now; last = m;
	const uint32_t v = m < 0 ? 0 : (uint32_t)m > 0xffff ? 0xffff : (uint32_t)m;
	const uint8_t data[] = {0, (uint8_t)(m >= 0), (uint8_t)v, (uint8_t)(v >> 8)};
	publish_event(15, data, sizeof data);
}
/* The scan's row for a configured channel (by frequency), or -1. */
static int scan_row_of(uint32_t frequency_khz)
{
	for (unsigned r = 0; r < scan_info.count && r < AR_RF_MAX_CHANNELS; r++) {
		if ((scan_rows[r] & 0xffffff) == frequency_khz) { return (int)r; }
	}
	return -1;
}
/* SET 35's work list is the scan's list too (stock: bit 24 of each channel
 * word); with none set, every channel. Re-applied when the scan is rebuilt. */
static void scan_apply_work_list(void)
{
	for (unsigned r = 0; r < scan_info.count && r < AR_RF_MAX_CHANNELS; r++) {
		unsigned on = !chan_work_count || !receiver.channel_count;
		for (unsigned i = 0; i < chan_work_count && !on; i++) {
			on = chan_work[i] < receiver.channel_count &&
				receiver.channels[chan_work[i]].frequency_khz == (scan_rows[r] & 0xffffff);
		}
		scan_rows[r] = (scan_rows[r] & ~(UINT32_C(1) << 24)) | (uint32_t)on << 24;
	}
}
int ar_rf_host_channel_info(uint8_t output[1028])
{
	if (!output) { return -22; }
	if (information.phase < AR_RF_PSRAM || !receiver.channel_count) { return -11; }
	memset(output, 0, 1028);
	output[0] = (uint8_t)receiver.channel_count; output[1] = chan_auto;
	/* acs_chan: the scan's start-up choice (scan scores), else the link's. */
	uint8_t acs = (uint8_t)receiver.channel_index;
	if (scan_scores.selected < scan_info.count) {
		for (unsigned i = 0; i < receiver.channel_count; i++) {
			if (receiver.channels[i].frequency_khz == (scan_rows[scan_scores.selected] & 0xffffff)) { acs = (uint8_t)i; }
		}
	}
	output[2] = acs; output[3] = (uint8_t)receiver.channel_index;
	for (unsigned i = 0; i < receiver.channel_count && i < 128; i++) {
		uint32_t f = receiver.channels[i].frequency_khz;
		int r = scan_row_of(f);
		uint32_t p = r >= 0 ? (uint32_t)scan_history[r].score : 0;
		for (unsigned b = 0; b < 4; b++) { output[4 + i * 4 + b] = f >> (8 * b); output[516 + i * 4 + b] = p >> (8 * b); }
	}
	return 0;
}
int ar_rf_host_channel_set(unsigned order, const uint8_t *payload, size_t length)
{
	if (order == 5) { if (length < 1) { return -22; } chan_auto = payload[0] & 1; return 0; }
	if (order == 35) {
		if (length < 1 || payload[0] > 128 || length < 1u + payload[0]) { return -22; }
		for (unsigned i = 0; i < payload[0]; i++) { if (payload[1 + i] >= receiver.channel_count) { return -22; } }
		chan_work_count = payload[0]; memcpy(chan_work, payload + 1, payload[0]);
		scan_apply_work_list();   /* dropped channels keep their last power */
		/* The AP with no ground: a list without the channel it is on moves it
		 * onto the list's first, at the next IRQ 5, as a stock air radio
		 * does. Pairing relies on it: the air app narrows the list to the
		 * common channel (5839 MHz) for the bind window, where a ground in
		 * pairing mode looks, and restores it afterwards. */
		if (ap_role && ap_link.state != AR_AP_CONNECTED && chan_work_count && !ap_chan_hop) {
			unsigned on = 0;
			for (unsigned i = 0; i < chan_work_count && !on; i++) { on = chan_work[i] == receiver.channel_index; }
			if (!on) { ap_chan_at_sweep = 0x10000 | chan_work[0]; }
		}
		return 0;
	}
	if (order == 6 && ap_role) { return ap_host_channel(payload, length); }
	if (order == 6) {
		if (length < 2 || !dev_active || receiver.dev_state != 2) { return -7; }
		if (payload[1] >= receiver.channel_count) { return -6; }
		if (payload[1] == receiver.channel_index) { return 0; }
		return dev_hop_request(payload[1]);
	}
	return -95;
}

/* The AP's command 5 arrived: the switch waits for this frame's IRQ 5, as
 * the AP does, so both run the new width from the next frame. Across 40 MHz
 * the frame changes; 5/10/20 at once dropped the link on the short frame
 * (mode 2) every time (bench, 2026-10-19), at IRQ 5 they do not. */
static void bandwidth_switch(unsigned code)
{
	if (code > 5 || code == bw_code) { return; }
	if ((bw_due_code & 7) == code && bw_due_code) { return; }   /* timed: its frame is known */
	bw_at_sweep = 0x80 | code;
}
static void sync_bandwidth(const uint8_t control[7])
{
	/* 0x238960 with param1 == 0: a type-0 switch for slot bits 3:0, code bits 7:5. */
	bw_info.announced++; bw_info.commands++;
	bw_info.last = control[3] | (uint32_t)control[4] << 8 | (sync_timer_apply ? 0x4000u : 0);
	if (control[3] != 0) { return; }
	unsigned code = control[4] >> 5;
	bandwidth_switch(code);
	bw_state = 2;
}

static void bandwidth_poll(void)
{
	static unsigned settled_mcs; static uint32_t settled_ms;
	uint32_t now = elapsed_ms();
	if (dev_mcs.current != settled_mcs) { settled_mcs = dev_mcs.current; settled_ms = now; }
#ifdef AR_BW_TOGGLE_MS
	/* Bench only: the widest allowed flips between AR_BW_TOGGLE_LOW and
	 * AR_BW_TOGGLE_HIGH (width codes, 20 and 40 MHz unless given) every
	 * AR_BW_TOGGLE_MS while linked, so the width rule narrows and widens through
	 * the real request path (command 9, the AP's command 5) for counting what
	 * each switch costs. */
#ifndef AR_BW_TOGGLE_LOW
#define AR_BW_TOGGLE_LOW 4
#endif
#ifndef AR_BW_TOGGLE_HIGH
#define AR_BW_TOGGLE_HIGH 5
#endif
	{
		static uint32_t toggle_ms;
		if (dev_active && receiver.dev_state == 2 && (uint32_t)(now - toggle_ms) > AR_BW_TOGGLE_MS) {
			bw_policy.max = bw_policy.max == AR_BW_TOGGLE_HIGH ? AR_BW_TOGGLE_LOW : AR_BW_TOGGLE_HIGH;
			toggle_ms = now;
		}
	}
#endif
	if (bw_settle_step) {
		/* The new width's headers bring the receiver back (bandwidth_settle);
		 * if they do not come, a frame later than they would. */
		uint32_t age = elapsed_us() - bw_settle_us;
		uint32_t frame = receiver.search_frame_us ? receiver.search_frame_us : 5634;
		if ((bw_settle_step == 1 && age >= 2 * frame) || (bw_settle_step == 2 && age >= 3 * frame)) {
			bandwidth_settle(0);
		}
	}
	/* A switch done (command 5 applied, the receiver released): look again.
	 * One due across 40 MHz is still under way: decided on the width we leave,
	 * the next request undid it (bench: tens of switches in 5 s). */
	unsigned switching = bw_settle_step || bw_at_sweep || bw_due_code;
	if (bw_state == 2 && !switching) { bw_state = 0; bw_info.requests = 0; }
	if (bw_automatic && bw_state == 0 && !bw_precap && dev_active && receiver.dev_state == 2 && receiver.assigned_slot == 0 &&
	    !switching && dev_mcs.current < dev_mcs.config.count) {
		/* A stock AP drops and re-forms the link at every step to or from
		 * 40 MHz: up to 20 until a frame stamp says the AP is Open8030 (one
		 * every 100 ms; requests wait 3 s into a link anyway). */
		struct ar_width_config policy = bw_policy;
		if (!fstamp.valid && policy.max > 4) { policy.max = 4; }
		unsigned want = ar_width_decide(&policy, &bw_width, bw_code,
			dev_mcs.config.rows[dev_mcs.current].mcs, dev_mcs.average_snr, now);
		if (want != bw_target) { bw_target = want; bw_info.requests = 0; }
	}
	if (frame_want && !frame_request && !frame_pending && dev_cmd.state == 0 && frame_retries &&
	    dev_active && receiver.dev_state == 2 && (frame_want & 7) != frame_mode && (uint32_t)(now - frame_retry_ms) > 1000) {
		frame_request = frame_want; frame_retries--; frame_retry_ms = now; frame_info.retries++;
	}
	if (frame_request) {
		if (!dev_active || receiver.dev_state != 2 || (frame_request & 7) == frame_mode) { frame_request = 0; }
		else {
			const uint8_t word[4] = {(uint8_t)((frame_request & 7) << 4), 0, 0, 0};
			if (!dev_command_request(8, 5, word, 4)) { frame_request = 0; }
		}
	}
#ifndef AR_NO_CEILING_40
	/* Held, the width decision would say "stay" again for widen_ms: the target
	 * is kept until the switch is asked for, or for a second at most. */
	if (bw_precap && bw_state == 0 && (uint32_t)(now - bw_precap_ms) > 1000) {
		bw_precap = 0; apply_ceiling();
		if (bw_automatic) { bw_target = bw_code; }   /* decided again, not capped again at once */
	}
	if (bw_precap && (bw_target != 5 || bw_code == 5 || !dev_active || receiver.dev_state != 2)) {
		bw_precap = 0; apply_ceiling();
	}
	if (!bw_precap && bw_state == 0 && bw_target == 5 && bw_code < 5 && dev_mcs.current < dev_mcs.config.count &&
	    dev_mcs.config.rows[dev_mcs.current].mcs > CEILING_40) { bw_precap = 1; bw_precap_ms = now; apply_ceiling(); }
	const unsigned under_cap = !bw_precap || (!dev_mcs.pending && dev_mcs.current < dev_mcs.config.count &&
		dev_mcs.config.rows[dev_mcs.current].mcs <= CEILING_40);
#else
	const unsigned under_cap = 1;
#endif
	if (bw_state == 0) {
		if (bw_info.requests >= 4 && bw_code != bw_target && (uint32_t)(now - bw_retry_ms) > 10000) {
			/* Four quick tries went unanswered: keep asking every 10 s rather
			 * than leaving the link narrow until the next rebuild. */
			bw_info.requests = 3; bw_retry_ms = now;
		}
		/* Wider while the MCS still climbs, from 1 s into the link: an
		 * Open8030 AP keeps a width switch through MCS changes (it cancels
		 * only its own uplink-MCS command). A stock AP's MCS state machine
		 * (0x253412 from its 0x106 path) cancels a pending sync command, our
		 * switch included: unanswered, it is asked for again (state 1's
		 * deadline). Before, wider waited for 3 s into the link and 2 s on
		 * one MCS: ~15 s at 5 MHz after a link-up. Narrower at once. */
		(void)settled_ms;
		if (dev_active && receiver.dev_state == 2 && receiver.assigned_slot == 0 && bw_code != bw_target && !switching &&
		    (bw_target < bw_code || dev_mcs.acknowledgements) && under_cap &&
		    bw_info.requests < 4 && (uint32_t)(now - receiver.connected_elapsed_us / 1000) > 1000) {
			uint8_t code = (uint8_t)bw_target;
			(void)ar_dev_command_encode(dev_bw_request, sizeof dev_bw_request, 9, &code, 1);
			bw_request_pending = 1; bw_info.requests++;
			bw_state = 1; bw_deadline_ms = now + 2000;
			/* The AP can pause its producer before announcing the new width.
			 * Do not turn that pause into a concurrent coding downgrade. This
			 * guard expires even if the AP never answers the request. */
			bandwidth_mcs_settle(0);
		}
	} else if (bw_state == 1 && (int32_t)(now - bw_deadline_ms) >= 0) {
		/* No command 5 from the AP: ask again later (poll state 0 limits it). */
		bw_request_pending = 0; bw_state = 0;
		receiver.connected_elapsed_us = elapsed_us();
	}
}

static void sync_tx_mcs(const uint8_t control[7])
{
	/* 0x2300a2 -> 0x229bd4 -> PHY107: mode0 DEV accepts slot0;
	 * low five payload bits select TX MCS, irrespective of bandwidth bits
	 * when an MCS is present. 0x24d2a4 applies increases immediately. */
	unsigned mcs = control[4] & 31;
	if (mcs >= 25) {
		/* MCS "none": a bandwidth announcement from the AP (0x22b5b8). */
		unsigned code = control[4] >> 5;
		bw_info.announced++; bw_info.last = 0x8000u | control[3] | (uint32_t)control[4] << 8;
		if (code <= 5 && code != bw_code) { bandwidth_switch(code); bw_state = 2; }
		return;
	}
	tx_mcs_info.requests++; tx_mcs_info.requested = mcs;
	if (control[3] || mcs >= 16) { tx_mcs_info.rejected++; return; }
	struct ar_interleaver_user *u = &tx_interleaver.users[0];
	if (mcs == u->mcs) { tx_mcs_info.unchanged++; return; }
	if (u->active) { tx_mcs_info.busy_requests++; return; }
	uint32_t key = mcs_fch_keys[mcs], word = read32(BB + 0xb00);
	unsigned modulation = (key >> 16) & 255, code = (key >> 8) & 255, rate = key & 255;
	if ((key >> 24) || modulation > 4 || code > 2 || rate > 2) { tx_mcs_info.rejected++; return; }
	/* 0x221610: code length follows bandwidth/interleave and modulation.
	 * Normal keys leave a TX mode other than2 unchanged; mode2 becomes1. */
	static const uint8_t bits[] = {1,2,4,6,8};
	unsigned exponent = (word & 7) + 2 - rate + ((word >> 18) & 7);
	unsigned value = ((unsigned)bits[modulation] << exponent) >> 2;
	if (value != 2 && value != 4 && (!value || (value & 7))) { tx_mcs_info.rejected++; return; }
	tx_mcs_info.before = word; tx_mcs_info.key = key;
	if (ar_tx_mcs_control(&tx_interleaver,AR_TX_MCS_REQUEST,0,mcs,mcs_fch_keys,
	    information.chip_profile,1,&tx_mcs_ops,NULL)) { tx_mcs_info.rejected++; return; }
	if (u->active) { tx_mcs_info.downgrades++; }
	else { tx_mcs_applied(mcs); }
}
/* The AP's TX power as it announced it (0: not known, the link down):
 * event 14 on a change, and the power loop steps from it. */
static void dev_peer_power(uint8_t dbm)
{
	if (dbm) { ar_pwr_auto_peer(pwr_auto_context(), dbm); }
	if (dbm == dev_peer_dbm) { return; }
	const uint8_t data[] = {0, 0, dbm, dev_peer_dbm};
	dev_peer_dbm = dbm;
	publish_event(14, data, sizeof data);
}
void ar_rf_power_events(void)
{
	const unsigned own = power_level[ap_role ? 8 : 0];
	if (own != 0xff) {
		const uint8_t data[] = {0, 1, (uint8_t)own, (uint8_t)own};
		publish_event(14, data, sizeof data);
	}
	if (!ap_role && dev_peer_dbm) {
		const uint8_t data[] = {0, 0, dev_peer_dbm, dev_peer_dbm};
		publish_event(14, data, sizeof data);
	}
}
static void sync_apply(void)
{
	const uint8_t *control = sync_info.last_control;
	if (receive_operation >= 71) { traffic_info.commands[control[1] & 15].dispatched++; }
	if (receive_operation >= 72 && (control[1] & 15) == 3) { sync_tx_mcs(control); return; }
	if (receive_operation >= 72 && (control[1] & 15) == 5) { sync_bandwidth(control); return; }
	if (receive_operation >= 72 && (control[1] & 15) == 4) { sync_channel(control); return; }
	if (receive_operation >= 72 && (control[1] & 15) == 8) {
		/* The sync message: sub-type 0 the frame mode; others (6, a power
		 * request from an AP, which this DEV does not take) are no frame. */
		if ((control[3] & 15) == AR_PWR_AUTO_ANNOUNCE) {
			uint8_t dbm;
			uint32_t word = control[3] | (uint32_t)control[4] << 8 | (uint32_t)control[5] << 16 | (uint32_t)control[6] << 24;
			if (!ar_pwr_auto_parse_announce(word, &dbm)) { dev_peer_power(dbm); }
			return;
		}
		if (control[3] & 15) { sync_info.unsupported++; return; }
		frame_info.from_ap++; frame_pending = 0x80 | ((control[3] >> 4) & 7); return;
	}
	if ((control[1] & 15) != 6) { sync_info.unsupported++; return; }
	unsigned user = control[3] & 15, transmit = !!(control[3] & 16);
	if (user > 9) { sync_info.invalid_users++; return; }
	/* Original 0x236d9a: incoming bit4=0 sets RX identity, but not in
	 * DEV idle or flag0x40 state. Native admits only the supported normal
	 * mode0 link; unsupported remote-sync flags never enter state2. */
	if (!transmit && !receiver.dev_state) { sync_info.state_skips++; return; }
	uint32_t encoded = transmit ? (((user + 2) & ~3u) + 0xb7c) << 16 |
		((user + 2) & 3) << 11 | 8 : (user + 0x1b4) * 0x80000 | 8;
	uint32_t shared = transmit ? 0xb601008 : 0xdf00008;
	sync_info.user = user; sync_info.identity = control[4]; sync_info.transmit = transmit;
	sync_info.field_before = read32(BB + (encoded >> 16));
	sync_info.shared_before = read32(BB + (shared >> 16));
	field(encoded, control[4]);
	if (user == 0 || user == 8) { field(shared, control[4]); }
	sync_info.field_after = read32(BB + (encoded >> 16));
	sync_info.shared_after = read32(BB + (shared >> 16));
	sync_info.last_apply_us = elapsed_us();
	if (!sync_info.rx_writes && !sync_info.tx_writes) { sync_info.first_apply_us = sync_info.last_apply_us; }
	if (transmit) { sync_info.tx_writes++; } else { sync_info.rx_writes++; }
}
/* Polling serializes control handling; start_prepared_receive masks interrupt
 * delivery. These callbacks must gain real locks before an interrupt/thread port. */
static void sync_lock(enum ar_phy_receive_lock which, void *context) { (void)which; (void)context; }
static void sync_control_lock(void *context) { (void)context; }
static int sync_read(uint8_t packet[7], void *context) { memcpy(packet,context,7); return 0; }
static void sync_prepare(void *context) { (void)context; }
static void sync_field(uint32_t address, uint32_t value, void *context)
{
	(void)context; field(address,value);
}
static void sync_dispatch(unsigned command, uint8_t *payload, size_t length, void *context)
{
	(void)command; (void)payload; (void)length;
	memcpy(sync_info.last_control,phy_control.rx,7);
	if (context) { sync_info.immediate++; }
	sync_apply();
}
static void sync_expire_dispatch(unsigned outgoing, unsigned command, uint8_t *payload,
	size_t length, void *context)
{
	(void)outgoing; sync_dispatch(command,payload,length,context);
}
static void sync_stop_timer(void *context) { (void)context; sync_info.pending = 0; }
static void sync_start_timer(uint32_t delay, void *context)
{
	(void)context; sync_info.deadline_ms = elapsed_ms()+delay;
	sync_info.pending = 1; sync_info.armed++;
}
static int sync_admit(const uint8_t record[150], void *context)
{
	/* The original synthesized record requests mode3. Normal runtime still
	 * supports mode0 only; do not pass this internal record to the MAC decoder
	 * or pretend its required reconfiguration has been applied. */
	(void)record; (void)context;
	receiver.admission_attempts++; receiver.admission_rejects++;
	receiver.last_admission_result = (uint32_t)-95;
	return -95;
}
static uint32_t sync_now(void *context) { (void)context; return (uint32_t)(timer_ticks()/40000); }
/* Expiry never writes a TX packet or refreshes an AP idle packet. */
static void sync_unused_write(const uint8_t packet[7], void *context) { (void)packet; (void)context; }
static void sync_unused_idle(uint8_t packet[7], void *context) { (void)packet; (void)context; }
static const struct ar_phy_receive_ops sync_ops = {
	sync_lock,sync_lock,sync_read,sync_prepare,sync_field,sync_dispatch,
	sync_stop_timer,sync_start_timer,sync_admit,sync_now
};
static const struct ar_phy_control_ops sync_expire_ops = {
	sync_control_lock,sync_control_lock,sync_field,sync_unused_write,sync_expire_dispatch,sync_unused_idle
};
static void sync_receive(const uint8_t control[7])
{
	if ((control[1] & 15) == AR_PHY_STAMP) {
		/* Open8030's frame stamp: not a stock command (refused there, and never
		 * cached as the AP's last one). Checked like any block, and only on
		 * our link: a stamp from another AP on the channel (a second
		 * Open8030 air unit, or one heard during the search) would give the
		 * host a clock that is not this link's. */
		if (dev_active && receiver.dev_state == 2 &&
		    ((control[2] >> 1) & 7) == 4 && control[0] == ar_phy_control_checksum(control)) {
			const uint64_t us = control[3] | (uint64_t)control[4] << 8 | (uint64_t)control[5] << 16 |
				(uint64_t)(control[6] & 15) << 24 | (uint64_t)(control[1] >> 4) << 28;
			(void)ar_frame_stamp_receive(&fstamp, us);
		}
		return;
	}
	if ((control[1] & 15) == AR_PHY_RETRIES) {
		/* Open8030's resend map: the AP's last 12 frames (on our link, checked). */
		if (dev_active && receiver.dev_state == 2 && fstamp.frames &&
		    ((control[2] >> 1) & 7) == 4 && control[0] == ar_phy_control_checksum(control)) {
			const uint32_t map = control[3] | (uint32_t)control[4] << 8 | (uint32_t)control[5] << 16;
			const uint8_t index = (uint8_t)((control[6] & 15) | (control[1] >> 4) << 4);
			/* Its newest: the frame that ended at our IRQ 5 before the latest. */
			const uint32_t newest = fstamp.frames - 1;
			unsigned take = AR_RETRY_FRAMES;
			if (retry_link) {
				const uint8_t more = (uint8_t)(index - retry_last_ap);
				if (!more || more > 128) { return; }              /* the same map again, or an older one */
				if (more < take) { take = more; }
			}
			if (retry_have && (int32_t)(newest - retry_newest) <= 0) { return; }
			retry_last_ap = index; retry_link = 1;
			if (!retry_have) { memset(retry_ring, 0xff, sizeof retry_ring); }
			else {
				/* Frames no map covered (the channel busy longer): not known. */
				const uint32_t gap = newest - take - retry_newest;
				for (uint32_t f = 1; (int32_t)gap > 0 && f <= gap && f <= 256; f++) { retry_ring[(retry_newest + f) & 255] = 0xff; }
			}
			for (unsigned k = 0; k < take; k++) {
				const unsigned v = (map >> (2 * k)) & 3;
				retry_ring[(newest - k) & 255] = (uint8_t)v;
				retry_known_frames++;
				if (v == 3) { retry_stalls++; } else { retry_blocks += v; }
			}
			retry_newest = newest; retry_have = 1;
		}
		return;
	}
	/* Runtime provisioning currently admits only mode0 and local position0.
	 * Broader mode3/retransmission state ownership remains unintegrated. */
	struct ar_phy_receive receive = {.slot=receiver.assigned_slot,.dev_state=receiver.dev_state};
	memcpy(receive.mac,dev_config.local,4);
	phy_control.role = 1; phy_control.mode = dev_config.mode;
	phy_control.frame_us = receiver.search_frame_us; phy_control.callbacks = 0x17e;
	if (ar_phy_receive_process(&phy_control,&receive,&sync_ops,(void *)control)) {
		sync_info.rejected++; return;
	}
	unsigned command = control[1] & 15;
	if (command == 5 && control[3] == 0 && (control[4] >> 5) <= 5 &&
	    (control[4] >> 5) != bw_code) {
		unsigned life = (control[1] >> 4) | (control[2] & 1) << 4;
		if (life) { bandwidth_mcs_settle(life * receiver.search_frame_us / 1000 + 4); }
		unsigned code = control[4] >> 5;
		/* 5/10/20 on stock timing (the robust timer, the TTL-1 header going
		 * out on the new width) moved a frame or two after an Open8030 AP:
		 * its first frames' video lost. A stock AP's timing is not known. */
		if (life >= 2 && life < 31 && (code == 5 || bw_code == 5 || fstamp.valid) && fstamp.irq5[0]) {
			/* A header handled behind the next IRQ 5 (one late poll pass,
			 * IRQ 5 first): its frame began at the IRQ 5 before. */
			uint64_t base = fstamp.irq5[0];
			if (fstamp.irq5[1] && timer_ticks() - base < 40000) { base = fstamp.irq5[1]; }
			bw_due_code = 0xc0 | code;
			bw_due_ticks = base + (uint64_t)(life * receiver.search_frame_us - receiver.search_frame_us * 3 / 2) * 40;
			if ((bw_at_sweep & 7) == code) { bw_at_sweep = 0; }
		}
#ifdef AR_BW_TOGGLE_MS
		if (life && life < 31) {
			bw_heard = 0x80 | (control[4] >> 5); bw_sweeps_left = 0;
			bw_path = (bw_path & ~3u) | (life < 3 ? life : 3);
			bw_header_us = elapsed_us(); bw_header_phase = bw_header_us - bw_irq5_us; bw_irq5_gap = 0;
			bw_pre_valid = bw_pre_invalid = bw_pre_other = 0;
		}
#endif
	}
	sync_info.accepted++; memcpy(sync_info.last_control,phy_control.rx,7);
	if (receive_operation >= 71) {
		struct ar_rf_command_observation *item = &traffic_info.commands[command];
		uint32_t elapsed = elapsed_us();
		if (!item->accepted) { item->first_us = elapsed; memcpy(item->first, control, 7); }
		item->accepted++; item->last_us = elapsed; memcpy(item->last, control, 7);
	}
	if (command == 6 && !sync_info.identity_commands++) { memcpy(sync_info.first_identity, control, 7); }
}
static void sync_poll(uint32_t ms)
{
	if (!sync_info.pending || (int32_t)(ms - sync_info.deadline_ms) < 0) { return; }
	sync_info.pending = 0; sync_info.timer_fired++;
	(void)ar_phy_control_expire(&phy_control,&sync_expire_ops,NULL);
}
static void read_control_words(uint32_t *words)
{
	static const uint16_t offsets[] = {0xf04, 0xf08, 0xf0c, 0xf10, 0xf34, 0xf38, 0xf3c};
	for (unsigned word = 0; word < 7; word++) { words[word] = read32(BB + offsets[word]); }
}
static void read_control_header(uint32_t *words)
{
	/* Stock 0x22d30a reads this bank, restoring the caller's selector. */
	uint32_t old = read32(BB + 0xbb0);
	field(0xbb00701, 1);
	read_control_words(words);
	field(0xbb00701, (old >> 7) & 1);
}
static uint32_t read_header_status(void)
{
	uint32_t old = read32(BB + 0xbb0);
	field(0xbb00701, 0);
	uint32_t value = read32(BB + 0xf00);
	field(0xbb00701, (old >> 7) & 1);
	return value;
}
static int failure(int code)
{
	information.error = (uint32_t)code;
	return code;
}
static int wait_bits(uint32_t address, uint32_t mask, uint32_t expected, uint32_t us)
{
	uint32_t started = read32(TIMER);
	/* Both the timer and an iteration cap bound failed-clock/RTL cases. */
	for (unsigned iteration = 0; iteration < 1000000; iteration++) {
		if ((read32(address) & mask) == expected) { return 0; }
		if ((uint32_t)(read32(TIMER) - started) >= us * 40) { return -110; }
	}
	return -110;
}
static int delay_us(uint32_t us)
{
	uint32_t started = read32(TIMER);
	for (unsigned iteration = 0; iteration < 1000000; iteration++) {
		if ((uint32_t)(read32(TIMER) - started) >= us * 40) { return 0; }
	}
	return -110;
}

static int otp_word(unsigned offset, uint32_t *word)
{
	write32(OTP + 0x98, offset / 4);
	barrier();
	uint32_t started = read32(TIMER);
	for (unsigned iteration = 0; iteration < 100000; iteration++) {
		uint32_t status = read32(OTP + 0xa4);
		information.otp_status = status;
		if (status & 4) {
			*word = read32(OTP + 0x9c);
			write32(OTP + 0xa4, 0);
			return 0;
		}
		if (status & 3) { write32(OTP + 0xa4, 0); return failure(-5); }
		if ((uint32_t)(read32(TIMER) - started) >= 80000) { break; }
	}
	return failure(-110);
}

int ar_rf_probe(void)
{
	memset(&information, 0, sizeof information);
	commands_ready = 0;
	receive_operation = 0; host_prepared = 0;
	command_state.cached_bank = 3; command_state.board_band = -1;
	pins_configured = 0;
	/* Stock Ascent VRX application entry sets these two system-control bits
	 * before baseband initialization (0x20abba); preserve all other bits. */
	update(0x0a1060c8, 0, 3);
	/* Measured clock table: ID 12 is CLOCK+0x80 bit 1. */
	update(CLOCK + 0x80, 0, 2);
	for (uint32_t offset = 0x100; offset < 0x300; offset += 4) { write32(OTP + offset, 0); }
	barrier();
	uint32_t word;
	int result = otp_word(8, &word);
	if (!result) { information.chip_profile = word >> 16 & 255; information.phase = AR_RF_PROBED; }
	return result;
}

int ar_rf_board_identity(uint8_t mac[4])
{
	if (!mac || information.phase != AR_RF_PROBED) { return -22; }
	uint32_t first, low, high;
	int result = otp_word(12, &first);
	if (!result) { result = otp_word(24, &low); }
	if (!result) { result = otp_word(28, &high); }
	if (result) { return result; }
	/* Original 204208 consumes bytes 4 and 16..19 of the OTP block at 10.
	 * Assemble unaligned bytes explicitly; no OTP programming operation. */
	uint32_t wafer = first >> 16 & 255, serial = low >> 16 | high << 16;
	if (!wafer && !serial) { return -2; }
	uint32_t packed = (serial << 6) | (wafer & 63);
	for (unsigned i = 0; i < 4; i++) { mac[i] = packed >> (8 * i); }
	return 0;
}

static unsigned io_domains;
void ar_rf_set_io_domains(unsigned mask) { io_domains = mask & 6; }
int ar_rf_setup_pins(const uint8_t *records, size_t length)
{
	/* Records {bank, pin, mux, pad}: the stock boot pinmux (0x20540a ->
	 * 0x2196d0), banks 0-3. Mux nibbles at GPIO+0x2c (bank 0), 0x30 (1),
	 * 0x34 (2), 0x38/0x3c (3, pins 0-7/8-12). A non-zero pad byte is the
	 * stock 0x219b78 argument, applied for banks 2-3 (6-bit pad fields at
	 * GPIO+0x14/0x18 and 0x1c/0x20/0x24). Banks 2-3 carry the RF front-end
	 * control functions (bank 3 pins 4-9: functions 0x7f-0x84); without
	 * them the receiver ran ~57 AGC steps hotter than stock. */
	static const uint8_t pin_count[4] = {5, 5, 6, 13};
	if (information.phase != AR_RF_PROBED || !records || !length || length % 4 || length > 29 * 4) { return failure(-22); }
	uint32_t seen[4] = {0};
	for (size_t i = 0; i < length; i += 4) {
		unsigned bank = records[i], pin = records[i + 1];
		if (bank > 3 || pin >= pin_count[bank] || records[i + 2] > 15 || (records[i + 3] && bank < 2)) { return failure(-22); }
		if (seen[bank] & (1u << pin)) { return failure(-22); }
		seen[bank] |= 1u << pin;
	}
	update(CLOCK + 0x200, 0, 1u << 25); /* Measured GPIO clock ID 21. */
	/* Stock 0x24be84(bank, 0) after muxing banks 2 and 3: GPIO+0 bits 1
	 * and 2 (their I/O domain select). */
	if (io_domains) { update(0x0a10a000, 0, io_domains); }
	else {
		if (seen[2]) { update(0x0a10a000, 0, 2); }
		if (seen[3]) { update(0x0a10a000, 0, 4); }
	}
	for (size_t i = 0; i < length; i += 4) {
		unsigned bank = records[i], pin = records[i + 1], mux = records[i + 2], pad = records[i + 3];
		uint32_t reg = bank < 3 ? 0x0a10a02c + bank * 4 : pin < 8 ? 0x0a10a038 : 0x0a10a03c;
		unsigned shift = (bank == 3 && pin >= 8 ? pin - 8 : pin) * 4;
		update(reg, 15u << shift, (uint32_t)mux << shift);
		if (!bank) { update(0x0a090140, 3u << (pin * 2), 2u << (pin * 2)); }
		if (pad) {
			unsigned value = pad & 8 ? 0x30 : pad & 4 ? 0x20 : pad & 2 ? 0x10 : 0;
			value |= pad & 0x40 ? 0 : pad & 0x20 ? 2 : 1;
			uint32_t pad_reg = bank == 2 ? (pin < 4 ? 0x0a10a014 : 0x0a10a018) :
				(pin < 4 ? 0x0a10a01c : pin < 8 ? 0x0a10a020 : 0x0a10a024);
			unsigned pad_shift = (pin % 4) * 8;
			uint32_t old = (read32(pad_reg) >> pad_shift) & 0x0c;
			update(pad_reg, 0x3fu << pad_shift, (uint32_t)(value | old) << pad_shift);
		}
	}
	pins_configured = 1;
	return 0;
}

static void configure_clocks(void)
{
	/* Board startup's inactive-bank clock configuration, preserving other bits. */
	uint32_t value = read32(CLOCK + 4);
	write32(CLOCK + 4, value & 0x80000000 ? (value & 0x7ffff880) | 0x1101 : (value & 0xf880ffff) | 0x91010000);
	value = read32(CLOCK + 0x14);
	write32(CLOCK + 0x14, value & 0x80000000 ? (value & 0x7ffffc80) | 0x1100 : (value & 0xfc80ffff) | 0x91000000);
	update(CLOCK, 0, 1u << 12); /* Clock ID 0: BB root. */
	update(CLOCK + 8, 0, 1u << 12); /* Clock ID 1: PSRAM domain. */
}

int ar_rf_load_defaults(const uint8_t *records, size_t length)
{
	if ((information.phase != AR_RF_PROBED && information.phase != AR_RF_DEFAULTS) || information.chip_profile != 1) {
		return failure(-95);
	}
	if (!records || !length || length % 8 || information.default_writes + length / 8 > 704) { return failure(-22); }
	/* Validate the entire batch before issuing any peripheral write. */
	for (size_t i = 0; i < length; i += 8) {
		unsigned offset = records[i] | (unsigned)records[i + 1] << 8;
		if (offset % 4 || offset > 0xdfc || records[i + 2] || records[i + 3]) { return failure(-22); }
	}
	if (!information.default_writes) { configure_clocks(); }
	command_state.cached_bank = 3; command_state.board_band = -1;
	for (size_t i = 0; i < length; i += 8) {
		unsigned offset = records[i] | (unsigned)records[i + 1] << 8;
		write32(BB + offset, le32(records + i + 4));
		information.default_writes++;
	}
	barrier();
	information.phase = AR_RF_DEFAULTS;
	information.error = 0;
	return 0;
}

static int hold_engines(void)
{
	field(0x8000001, 1); field(0x8000301, 1);
	field(0x80c0201, 1); field(0x80c0301, 1);
	return wait_bits(BB + 0xfa8, 0x300, 0x300, 2000);
}
static void release_engines(void)
{
	field(0x80c0201, 0); field(0x80c0301, 0);
	field(0x8000301, 0); field(0x8000001, 0);
}
static int idle(void)
{
	int result = hold_engines();
	release_engines();
	return result;
}

int ar_rf_calibrate(void)
{
	if (information.phase != AR_RF_DEFAULTS || information.default_writes != 704) { return failure(-22); }
	command_state.cached_bank = 3; command_state.board_band = -1;
	for (unsigned user = 0; user < 11; user++) {
		uint32_t offset = ((user >> 1) + 0x2ca) * 0x40000;
		field(user == 10 ? 0xb600d01 : offset | ((user & 1) * 0x1000 + 0x700) | 1, 1);
		field(user == 10 ? 0xb701c04 : offset | ((user & 1) * 0x10 + 8) * 0x100 | 4, 12);
		field(user == 10 ? 0xb701804 : offset | (user & 1) << 12 | 4, 12);
		if (user != 10) { field((user * 0x40000 + 0xb000000) | 0x1808, 20); }
	}
	field(0x81701, 1); field(0x81701, 0);
	if (wait_bits(BB + 0xe0, 0xff, 0x6f, 100000)) { return failure(-110); }
	field(0x1c1901, 1); field(0x1c1901, 0);
	if (wait_bits(BB + 0xe0, 0xff000000, 0xff000000, 100000)) { return failure(-110); }
	/* Band 2 (5 GHz) selector and five-byte calibration control value. */
	field(0x4840802, 2); field(0x4900808, 10);
	static const uint8_t control[5] = {1, 0x13, 0, 0, 0};
	for (unsigned i = 0; i < sizeof control; i++) {
		unsigned offset = 0x4d0 + i, shift = (offset & 3) * 8;
		update(BB + (offset & ~3u), 0xffu << shift, (uint32_t)control[i] << shift);
	}
	if (idle()) { return failure(-110); }
	uint8_t history[3][4] = {{0}};
	for (unsigned round = 0; round < 100; round++) {
		field(0x3281008, 0xff); field(0x3001008, 0xa0); field(0x3281008, 0);
		uint32_t started = read32(TIMER), state = 0;
		for (unsigned iteration = 0; iteration < 1000000; iteration++) {
			state = read32(BB + 0x360) & 0xff;
			if (state == 6 || state == 7) { break; }
			if ((uint32_t)(read32(TIMER) - started) >= 4000000) { break; }
		}
		information.calibration_status = state;
		information.calibration_rounds = round + 1;
		if (state != 6 && state != 7) { field(0x8000001, 1); return failure(-110); }
		uint32_t left = read32(BB + 0x4f0), right = read32(BB + 0x4f4);
		uint8_t *current = history[round % 3];
		current[0] = left; current[1] = left >> 8; current[2] = right >> 8; current[3] = right >> 16;
		if (round >= 2) {
			unsigned maximum = 0;
			for (unsigned sample = 0; sample < 2; sample++) {
				for (unsigned component = 0; component < 4; component++) {
					int a = (int8_t)history[sample][component], b = (int8_t)history[sample + 1][component];
					unsigned difference = (unsigned)(a > b ? a - b : b - a);
					if (difference > maximum) { maximum = difference; }
				}
			}
			if (maximum < 3) {
				memcpy(calibration, current, 4);
				calibration[4] = (current[0] & 0x80) | ((current[1] >> 1) & 0x40) |
					((current[2] >> 4) & 8) | ((current[3] >> 5) & 4);
				field(0x8000001, 1);
				information.phase = AR_RF_CALIBRATED;
				information.error = 0;
				return 0;
			}
		}
	}
	field(0x8000001, 1);
	return failure(-5);
}

int ar_rf_setup_psram(void)
{
	if (information.phase != AR_RF_CALIBRATED) { return failure(-22); }
	update(CLOCK + 8, 0, 1u << 28);
	update(CLOCK + 8, ~UINT32_C(0xfc80ffff), 0x1010000);
	field(0x8081901, 0); field(0xcd00001, 0); field(0xcdc0801, 0);
	field(0x8000301, 1); field(0x8000001, 1); field(0x8000201, 1);
	field(0x8000301, 0); field(0x8000001, 0);
	write32(BB + 0xcd0, 0x80250700); write32(BB + 0xcd4, 0x2800200);
	write32(BB + 0xcd8, 0x0c030000); write32(BB + 0xcdc, 0xee999000);
	write32(BB + 0xce0, 0xaaaa); write32(BB + 0xce4, UINT32_MAX);
	write32(BB + 0xce8, 0x120); write32(BB + 0xcec, 0x40000000);
	barrier();
	if (delay_us(1000)) { return failure(-110); }
	field(0xcd00001, 1);
	if (delay_us(1000)) { return failure(-110); }
	field(0xcd00501, 1);
	if (delay_us(10000)) { return failure(-110); }
	uint32_t taps = read32(BB + 0xfa0);
	unsigned low = (taps >> 8) & 15, high = (taps >> 12) & 15;
	field(0xcd41f01, 1);
	field(0xcd41008, low <= high ? ((low + high) >> 1) | 0x80 : 0x89);
	for (unsigned attempt = 0; attempt < 4; attempt++) {
		if (delay_us(1000)) { return failure(-110); }
		field(0xcdc1010, 0xaa99);
		if (delay_us(1000)) { return failure(-110); }
		field(0xcdc0801, 1);
		if (delay_us(5000)) { return failure(-110); }
		information.psram_status = read32(BB + 0xfa4);
		if (information.psram_status == 0xaa991007) {
			field(0xcd00501, 0); field(0xcdc0801, 0);
			field(0x8000001, 1); field(0x8000201, 1); field(0x80c0301, 1);
			int idle_result = wait_bits(BB + 0xfa8, 0x200, 0x200, 2000);
			field(0x80c0301, 0); field(0x8000201, 0);
			if (idle_result) { return failure(idle_result); }
			information.phase = AR_RF_PSRAM;
			information.error = 0;
			return 0;
		}
		field(0xcd41008, 0x89);
	}
	return failure(-5);
}

const struct ar_rf_info *ar_rf_information(void) { return &information; }

int ar_rf_pll_word(uint32_t frequency, uint64_t *word)
{
	if (!word || frequency > 7500000) { return -22; }
	for (unsigned bucket = 0; bucket < 2; bucket++) {
		uint32_t low = bucket ? 4860000 : 3500000, high = bucket ? 7500000 : 4860000;
		for (unsigned divider = 0; divider < 8; divider++, low >>= 1, high >>= 1) {
			uint32_t vco = frequency << (divider + 1);
			if (frequency > low && frequency <= high && vco >= 7000000 && vco <= 15000000) {
				uint32_t integer = (uint32_t)(vco / 40000), remainder = (uint32_t)(vco % 40000);
				uint32_t control = ((bucket ? 0 : 1) << 7) | (divider << 4) | (divider << 1) | (integer >> 8);
				uint32_t fraction = 0;
				for (unsigned bit = 0; bit < 24; bit++) {
					remainder <<= 1; fraction <<= 1;
					if (remainder >= 40000) { remainder -= 40000; fraction |= 1; }
				}
				*word = (uint64_t)(control & 255) << 32 | (uint64_t)(integer & 255) << 24 |
					fraction;
				return 0;
			}
		}
	}
	return -22;
}

static void serial_word(unsigned offset, uint64_t word)
{
	uint32_t high = word >> 32, low = (uint32_t)word;
	uint8_t bytes[5] = {high, low >> 24, low >> 16, low >> 8, low};
	for (unsigned byte = 0; byte < 5; byte++) {
		unsigned address = offset + byte, shift = (address & 3) * 8;
		update(BB + (address & ~3u), 0xffu << shift, (uint32_t)bytes[byte] << shift);
	}
}
static void tune_context_plls(uint64_t pll, uint32_t frequency)
{
	/* The 40-bit values use nonuniform subword placement, not a five-byte
	 * stride. Decode the offset AND bit shift of the corroborated field map. */
	static const uint16_t shadows[12] = {0x93a,0x940,0x945,0x94a,0x950,0x955,
		0x95a,0x960,0x905,0x96a,0x90a,0x90a};
	static const uint16_t other_shadows[12] = {0x910,0x915,0x91a,0x920,0x925,0x92a,
		0x930,0x935,0x900,0x965,0x90a,0x90a};
	/* Like stock (linked dump, 2026-10-16): only the broadcast receive bank
	 * (user 8's first word) and the uplink slot's bank (user 0's other word)
	 * follow the channel, with the search users 10/11; every other bank is
	 * parked at 2100 MHz. Tuning all 24 banks onto the receive channel cost
	 * the receiver most of its gain margin (AGC index ~75 vs stock's ~18,
	 * raw SNR 1500 vs 9000 on the same link). */
	uint64_t park;
	if (ar_rf_pll_word(2100000, &park)) { park = pll; }
	for (unsigned user = 0; user < 12; user++) {
		serial_word(shadows[user], user == 8 || user >= 10 ? pll : park);
		serial_word(other_shadows[user], user == 0 || user >= 10 ? pll : park);
		/* Search starts both active directions together; linked commands
		 * subsequently change only the bank named by the peer. */
		tuned_frequency[user][1] = user == 8 || user >= 10 ? frequency : 2100000;
		tuned_frequency[user][0] = user == 0 || user >= 10 ? frequency : 2100000;
	}
	if (commands_ready && receive_operation == 76) { tune_measurement(); }
}
static void reapply_power(unsigned user);
static void configure_receive_context(uint64_t pll)
{
	/* Both direction banks must follow the selected channel, including the
	 * slot banks selected after admission. Keep tuning separate from profile
	 * setup so search cannot overwrite PHY state or DMA ownership. */
	tune_context_plls(pll, receiver.frequency_khz);
	for (unsigned user = 0; user < 12; user++) {
		uint32_t phy = (0x9a0 + user * 8) << 16;
		unsigned broadcast = user == 8, search = user >= 10;
		field(phy | 3, search ? 0 : broadcast ? 5 : 3);
		field(phy | 0x803, broadcast || user == 11 ? 4 : user == 10 ? 3 : 1);
		field(phy | 0xd03, search ? 0 : broadcast ? 6 : 5);
		field(phy | 0x301, 1);
		field(phy | 0x401, 0);
		field(phy | 0x501, !search);
		field(phy | 0x602, 2);
		field(phy | 0x1808, broadcast ? 0x20 : 0x3c);
		field(phy | 0x1401, 1); field(phy | 0x1701, 1);
		uint32_t band = (0x9a4 + user * 8) << 16;
		/* Band of each bank (1 = below 4.3 GHz, 4 = 5 GHz), matching where
		 * tune_context_plls() puts it: bits 25-27 the first word, 29-31 the
		 * other. */
		field(band | 0x1903, user == 8 || user >= 10 ? 4 : 1); field(band | 0x1201, 1); field(band | 0x1601, 1);
		field(band | 0x1d03, user == 0 || user >= 10 ? 4 : 1); field(band | 0x1301, 1); field(band | 0x1701, 1);
		/* Per-user frequency field (MHz / 2): stock parks all but user 8. */
		if (user < 10 && user != 8) { field((uint32_t)(0xa80 + (user / 2) * 4) << 16 | (user & 1 ? 0x1010 : 0x0010), 0x041a); }
		field(band | 8, broadcast ? 0 : user == 0 ? 0x27 : 0x4e);
		field(band | 0x806, broadcast ? 15 : user == 0 ? 34 : 23);
		uint32_t power = (0xda0 + user * 8) << 16;
		field(power | 0x1808, broadcast ? 0 : user == 0 ? 0x27 : 0x4e);
		field((power + 0x40000) | 6, broadcast ? 14 : user == 0 ? 34 : 22);
		reapply_power(user);
		if (user < 10) {
			/* Corroborated static fields in the other per-user PHY bank, not
			 * packet contents, live gain status or DMA descriptors. */
			uint32_t other_phy = (0xb00 + user * 4) << 16;
			field(other_phy | 3, broadcast ? 2 : 1);
			field(other_phy | 0x301, 0); field(other_phy | 0x401, broadcast);
			field(other_phy | 0x501, 1); field(other_phy | 0x1501, 0);
			field(other_phy | 0x1203, 2); field(other_phy | 0x1602, user != 0);
			field(other_phy | 0x602, 0); field(other_phy | 0x1e01, broadcast);
			field(other_phy | 0x803, 0); field(other_phy | 0xc04, user == 0 || broadcast ? 6 : 3);
			field(other_phy | 0x1002, user != 0);
			field(((user >> 1) + 0x2ca) * 0x40000 | (user & 1) << 12 | 0x601, 1);
		}
	}
	field(0xa681e02, 0); field(0xc040e02, 0);
}
static unsigned irq_vector(unsigned index) { return index + (index < 4 ? 72 : 76); }
static void irq_ack(unsigned index)
{
	field(0xaf40001 | index << 8, 1); field(0xaf40001 | index << 8, 0);
	uint32_t address = 0xe0801000u + irq_vector(index) * 4;
	write8(address, read8(address) & ~1u);
}

static void phy_trace_begin(struct ar_rf_phy_event *event, unsigned index)
{
	event->begin_us = elapsed_us();
	event->flags = index | (phy_eop_seen[8] ? 1u << 8 : 0) |
		(phy_header_valid[8] ? 1u << 9 : 0) | 1u << 10;
	event->fbc = read32(BB + 0xfbc);
	uint32_t old = read32(BB + 0xbb0);
	field(0xbb00701, 0);
	event->fch_before = read32(BB + 0xf00); event->errors = read32(BB + 0xf28);
	field(0xbb00701, (old >> 7) & 1);
	old = read32(BB + 0xc00); field(0xc000701, 0);
	event->snr = read32(BB + 0xee0); field(0xc000701, (old >> 7) & 1);
	event->command = read32(BB + 0xe6c);
	event->pending = (read8(0xe0801000u + irq_vector(12) * 4) & 1) << 12 |
		(read8(0xe0801000u + irq_vector(14) * 4) & 1) << 14;
	event->samples = phy.samples; event->mac_valid = receiver.mac_valid_packets;
}

static void phy_trace_bucket(unsigned bucket, const struct ar_rf_phy_event *event)
{
	struct ar_rf_phy_bucket *b = &phy_trace.buckets[bucket];
	uint16_t snr = event->snr, errors = event->errors;
	if (!b->count || snr < b->snr_min) { b->snr_min = snr; }
	if (!b->count || snr > b->snr_max) { b->snr_max = snr; }
	if (!b->count) { b->first_us = event->begin_us; }
	b->count++; b->snr_sum += snr; b->error_sum += errors;
	b->zero_errors += !errors; b->last_us = event->begin_us;
}

static void phy_trace_finish(struct ar_rf_phy_event *event, unsigned index)
{
	uint32_t old = read32(BB + 0xbb0); field(0xbb00701, 0);
	event->fch_after = read32(BB + 0xf00); field(0xbb00701, (old >> 7) & 1);
	if (event->fch_before & 0x80) { event->flags |= 1u << 11; }
	if (receiver.mac_valid_packets != event->mac_valid) { event->flags |= 1u << 12; }
	if (phy.samples != event->samples) { event->flags |= 1u << 13; }
	event->samples = phy.samples; event->mac_valid = receiver.mac_valid_packets;
	event->end_us = elapsed_us();
	uint32_t elapsed = event->end_us - event->begin_us;
	if (elapsed > phy_trace.max_handler_us) { phy_trace.max_handler_us = elapsed; }
	phy_trace.header_changed += event->fch_before != event->fch_after;
	phy_trace.simultaneous_pending += event->pending == ((1u << 12) | (1u << 14));
	if (index == 12) {
		phy_trace_bucket(event->flags & (1u << 8) ? 1 : 0, event);
		if (event->flags & (1u << 9)) { phy_trace_bucket(2, event); }
	} else { phy_trace_bucket(index == 13 ? 5 : event->fch_before & 0x80 ? 3 : 4, event); }
	if (phy_trace.first_count < 32) { phy_trace.first[phy_trace.first_count++] = *event; }
	phy_trace.last[phy_trace.last_head] = *event;
	phy_trace.last_head = (phy_trace.last_head + 1) & 31;
	if (phy_trace.last_count < 32) { phy_trace.last_count++; }
	phy_trace.events++;
}

static void receive_event(void)
{
	receiver.receive_events++;
	uint32_t bitmap = read32(BB + 0xf88) & 0x3ff;
	unsigned user = 0;
	while (user < 10 && !(bitmap & (1u << user))) { user++; }
	/* No user bit: stock logs "wrong user" and takes its role's receive
	 * user, 8 on a DEV, 0 (the slot) on an AP - where it is always so. */
	if (user == 10) {
		if (!ap_role) { receiver.invalid_packets++; return; }
		user = 0;
	}
	field(0xcec0004, user);
	if (user < 8) {
		uint32_t bits = read32(BB + 0xcf0) >> 24, bit = 1u << user;
		field(0xcf01808, bits | bit); field(0xcf01808, bits & ~bit);
	} else {
		uint32_t bits = read32(BB + 0xcf4) & 3, bit = 1u << (user - 8);
		field(0xcf40002, bits | bit); field(0xcf40002, bits & ~bit);
	}
	uint32_t metadata = read32(BB + 0xf90), length = metadata & 0xffff;
	if ((metadata & (1u << 29)) || !length || length > sizeof receive_buffers[0] || receive_pending >= 4) {
		receiver.invalid_packets++;
		if (metadata & (1u << 29)) { receiver.drop_flagged++; }
		else if (receive_pending >= 4) { receiver.drop_busy++; }
		else { receiver.drop_other++; }
		if (!ap_role) {
			/* Kind 6: a dropped receive event - metadata, user bitmap, user, pending. */
			uint8_t d[12]; memcpy(d, &metadata, 4); memcpy(d + 4, &bitmap, 4);
			d[8] = (uint8_t)user; d[9] = (uint8_t)receive_pending; d[10] = d[11] = 0;
			capture_add(6, d, 12, read32(BB + 0xf90));
		}
		field(0xd540001 | (user + 8) << 8, 1); field(0xd540001 | (user + 8) << 8, 0);
		return;
	}
	unsigned slot = (receive_head + receive_pending) & 3;
	receive_queue[slot].length = length; receive_queue[slot].user = user; receive_queue[slot].metadata = metadata;
	receive_length = length; receive_user = user; receive_metadata = metadata; receive_pending++;
	ar_cache_flush(receive_buffers[slot], length);
	barrier();
	field(0xbec0020, (uint32_t)(uintptr_t)receive_buffers[slot]);
	field(0xbe81801, 1); field(0xbe81801, 0);
#ifdef AR_RF_SIMULATED
	ar_rf_test_start_dma(receive_buffers[slot], receive_length);
#endif
}

static void dev_admit(const struct ar_dev_control *control)
{
	if (receive_operation < 54 || dev_active || receiver.acquisition_ready) { return; }
	struct ar_dev_acquisition plan;
	receiver.admission_attempts++;
	int result = ar_dev_idle_evaluate(&dev_config, control, &plan);
	/* The AP advertises the return channel independently of its downlink. */
	if (!result && plan.transmit_channel != dev_transmit_channel) {
		result = retune_user(plan.transmit_channel, 0, 0);
	}
	receiver.last_admission_result = (uint32_t)result;
	if (result) { receiver.admission_rejects++; return; }
	receiver.acquisition_ready = 1;
	memcpy(receiver.accepted_peer, plan.peer, 4);
	receiver.assigned_slot = 8; /* AP login has not assigned a slot. */
	link_event(1, receiver.dev_state); receiver.dev_state = 1;
#ifdef AR_DEV_TRACE
	if (!ap_trace.freeze_at) { ap_trace.freeze_at = ap_trace.head + 140; }
#endif
	ar_dev_tx_init(&dev_tx);
	/* Mode-0 idle -> lock prerequisites (0x22d996/0x22d71c). This is a
	 * bounded first transition, not the subsequent login/MCS state machine. */
	field(0xb001602, plan.control_field_b00);
	field(0xbf00002, 1); /* 0x222bde(8,2) maps to physical user 0. */
	receiver.search_suspended = 1;
	/* Both configured TX profiles are below bandwidth code 4. This is the
	 * corroborated A48 value before those TX commands, not the RX8 profile. */
	field(0xa480020, 0);
	field(0xb7c1008, plan.peer_hash); field(0xb601008, plan.peer_hash);
	field(0x8000201, 1); field(0x80c0301, 1);
	result = wait_bits(BB + 0xfa8, 0x200, 0x200, 2000);
	field(0x80c0301, 0);
	receiver.transition_elapsed_us = elapsed_us();
	if (result) {
		receiver.transition_status = (uint32_t)result;
		return; /* TX remains held; no retry can falsely claim activation. */
	}
	dev_active = 1;
	if (automatic_recovery() && !(channel_monitor.users[8].flags & 1)) {
		channel_monitor.users[8].flags = 3; channel_monitor.users[8].missed = 0;
	}
	field(0x8000201, 0);
	receiver.transition_status = 1;
#ifdef AR_DEV_FORCE_SHORT
	apply_frame_mode(2);                 /* test: the short frame from admission */
#endif
}

static void dev_tx_finish_segments(void)
{
	field(0xd401808, 255); field(0xd401808, 0);
}

static void phy_write(const uint8_t packet[7], void *context)
{
	(void)context;
	uint8_t padded[8] = {0}; memcpy(padded,packet,7);
	mcs_info.fch_words[0] = le32(padded); mcs_info.fch_words[1] = le32(padded+4);
	write32(BB+0xb8c,mcs_info.fch_words[0]); write32(BB+0xb90,mcs_info.fch_words[1]);
	barrier(); mcs_info.fch_writes++;
}
static void phy_dispatch(unsigned outgoing, unsigned command, uint8_t *payload,
	size_t length, void *context)
{
	if (!outgoing) { sync_dispatch(command,payload,length,context); return; }
	if (ap_role) { ap_command_done(command, payload); return; }
	/* MCS requests use TTL31 and are cancelled on observation; no local
	 * completion is dispatched. Ours (channel 4, frame mode 8) apply here. */
	if (command == 4 || command == 8) { dev_command_done(command, payload); return; }
	sync_info.unsupported++;
}
static const struct ar_phy_control_ops phy_ops = {
	sync_control_lock,sync_control_lock,sync_field,phy_write,phy_dispatch,sync_unused_idle
};
static void phy_configure(void)
{
	phy_control.role = 1; phy_control.mode = dev_config.mode;
	phy_control.wake = mcs_sync_mmio; phy_control.callbacks = 0x17e;
	phy_control.frame_us = receiver.search_frame_us;
}
static struct ar_dev_mcs *dev_mcs_context(void)
{
	/* The pure MCS policy consumes a snapshot and produces an intent. The
	 * shared transport owns the pending command, including non-MCS requests.
	 * Polling serializes snapshot, policy evaluation and intent application. */
	memcpy(dev_mcs.fch,phy_control.tx,7);
	return &dev_mcs;
}
static void dev_mcs_reset_rx(void)
{
	if (!mcs_info.reset_pending || receive_pending) { return; }
	mcs_info.reset_pending = 0;
	/* A timed-out coding request must not destroy the producer while it
	 * still owes the CPU a DMA completion. Coalesce requests until drained. */
	field(0x8000101, 1); field(0x80c0201, 1);
	if (wait_bits(BB + 0xfa8, 0x100, 0x100, 2000)) { mcs_info.reset_errors++; }
	field(0x80c0201, 0); field(0x8000101, 0); mcs_info.rx_resets++;
}
static void dev_mcs_apply(unsigned action)
{
	if (action & AR_DEV_MCS_RESET_RX) {
		mcs_info.reset_pending = 1;
		mcs_info.reset_deferred += !!receive_pending;
		dev_mcs_reset_rx();
	}
	phy_configure();
	if (action & AR_DEV_MCS_SYNC_ON) {
		/* Preencode checked the authoritative slot through dev_mcs_context.
		 * Enqueue wakes synchronization but does not write the FCH registers. */
		(void)ar_phy_control_enqueue(&phy_control,3,31,dev_mcs.fch+3,2,&phy_ops,NULL);
	}
	/* Only our MCS request: the slot may hold another command of ours (a
	 * frame switch, a hop), whose sender would wait for it forever. */
	if ((action & AR_DEV_MCS_SYNC_OFF) && (phy_control.tx[1] & 15) == 3) {
		(void)ar_phy_control_cancel(&phy_control,&phy_ops,NULL);
	}
	memcpy(dev_mcs.fch,phy_control.tx,7);
	if (action & AR_DEV_MCS_APPLY_BUFFER) {
		/* PHY106 -> 222c0a; mode0 logical RX8 maps to physical buffer0. */
		field(0xbf00002, 1); field(0xd081001, 0); mcs_info.buffer_applies++;
		mcs_event(1,
			dev_mcs.current < dev_mcs.config.count ? dev_mcs.config.rows[dev_mcs.current].mcs : 25,
			dev_mcs.previous < dev_mcs.config.count ? dev_mcs.config.rows[dev_mcs.previous].mcs : 25);
		if (!mcs_info.first_ack_ms && dev_mcs.acknowledgements) {
			mcs_info.first_ack_ms = elapsed_ms();
		}
	}
}

static void dev_measure_eop(uint32_t status)
{
	/* 0x2210cc maps raw user9 to logical8; each FCH clears the EOP guard.
	 * This mode-0 path reports only the configured broadcast receiver8. */
	unsigned user = (status >> 20) & 15;
	if (user == 9) { user = 8; }
	if (user != 8) { phy.other_users++; return; }
	if (phy_eop_seen[user]) { phy.duplicates++; return; }
	phy_eop_seen[user] = 1;
	phy.last_fbc = status;
	uint32_t old = read32(BB + 0xbb0);
	field(0xbb00701, 0);
	phy.raw_errors = read32(BB + 0xf08 + user * 4);
	field(0xbb00701, (old >> 7) & 1);
	old = read32(BB + 0xc00);
	field(0xc000701, 0);
	phy.raw_snr = read32(BB + 0xea0 + user * 8);
	phy.raw_aux = read32(BB + 0xea4 + user * 8);
	field(0xc000701, (old >> 7) & 1);
	/* 0x22d158 and 0x22d22a select the gain/offset bank independently. */
	old = read32(BB + 0x808);
	field(0x8081b01, 0);
	phy.raw_gain[0] = read32(BB + 0xe30); phy.raw_gain[1] = read32(BB + 0xe34);
	field(0x8081b01, 1);
	phy.raw_offset[0] = read32(BB + 0xe30); phy.raw_offset[1] = read32(BB + 0xe34);
	field(0x8081b01, (old >> 27) & 1);
	uint16_t errors = phy.raw_errors;
	if (!errors && !phy_header_valid[user]) { errors = 1; phy.forced_errors++; }
	/* Stock leaves sample bytes9..11 as stack residue. Native initializes
	 * those padding bytes to zero, without inventing measurement values. */
	memset(phy.sample, 0, sizeof phy.sample);
	phy.sample[0] = phy.raw_snr; phy.sample[1] = phy.raw_snr >> 8;
	phy.sample[2] = errors; phy.sample[3] = errors >> 8;
	phy.sample[4] = phy.raw_errors >> 16; phy.sample[5] = phy.raw_errors >> 24;
	phy.sample[6] = (phy.raw_gain[0] >> 16) + (phy.raw_gain[0] >> 24);
	phy.sample[7] = phy.raw_gain[1] + (phy.raw_gain[1] >> 8);
	phy.sample[8] = phy.raw_aux;
	/* Exact 0x8204140b at 0x22d28a matched this byte reversal for 1024
	 * random hardware inputs in trial109. No other encoding is inferred. */
	uint32_t offset = __builtin_bswap32((phy.raw_offset[0] >> 16) | (phy.raw_offset[1] << 16));
	for (unsigned i = 0; i < 4; i++) { phy.sample[12 + i] = offset >> (i * 8); }
	uint64_t sample_ticks = elapsed_ticks();
	phy.sample_elapsed_us = (uint32_t)(sample_ticks / 40);
	if (!phy.samples) { memcpy(phy.first_sample, phy.sample, sizeof phy.sample); }
	phy.samples++;
	quality_ring[quality_head] = (struct quality_sample){(uint16_t)phy.raw_snr, phy.sample[6], phy.sample[7],
		(uint8_t)(errors != 0), (uint32_t)(sample_ticks / 40000)};
	quality_head = (quality_head + 1) & 7; if (quality_count < 8) { quality_count++; }
	{
		/* The frame this header is in ends at the next IRQ 5 (frame n ends
		 * at IRQ 5 number n). Frames between with no header: not known. */
		const uint32_t n = fstamp.frames + 1;
		if (fq_have && (int32_t)(n - fq_newest) > 0) {
			for (uint32_t f = fq_newest + 1; f != n && f - fq_newest <= 256; f++) { fq_word[f & 255] = 0; }
		}
		if (!fq_have || (int32_t)(n - fq_newest) >= 0) {
			int db10 = ar_pwr_auto_db10((uint16_t)phy.raw_snr); if (db10 < 0) { db10 = 0; }
			const unsigned q = (unsigned)db10 * 4 / 10 > 254 ? 254u : (unsigned)db10 * 4 / 10;
			const unsigned mcs = dev_mcs.current < dev_mcs.config.count ? dev_mcs.config.rows[dev_mcs.current].mcs & 15 : 0;
			fq_word[n & 255] = q | (uint32_t)phy.sample[6] << 8 | (uint32_t)phy.sample[7] << 16 | mcs << 24 |
				(uint32_t)((bw_code - 2) & 3) << 28 | 1u << 30 | (uint32_t)(errors != 0) << 31;
			fq_newest = n; fq_have = 1;
		}
	}
	if (receive_operation >= 67) {
		/* Policy inputs. SNR: the EA0 halfword as it is (stock's self.snr reads
		 * twice this on the bench, so the stock rows' thresholds hold ~3 dB more
		 * here). Both receive chains run and combine as stock (0x000 [6:3],
		 * 2T2R_STBC, path weights; checked live 2026-10-19) - "64-QAM 3/4
		 * fails here", written before that and the sensitivity fix, is gone.
		 * Errors: the F08 per-packet error word (non-zero on 1.4% of packets
		 * at 5 MHz, 12% at 20 MHz MCS 10, 64% at MCS 11) plus packets refused
		 * since the previous sample. Stock's per-chain snr_chan path keys on
		 * E6C user 12/14 events this receiver never produces (only user 8). */
		uint16_t policy_snr = (uint16_t)phy.raw_snr;
		uint32_t refused = (receiver.invalid_packets - phy_last_invalid) +
			(receiver.mac_invalid_packets - phy_last_mac_invalid);
		phy_last_invalid = receiver.invalid_packets; phy_last_mac_invalid = receiver.mac_invalid_packets;
		uint32_t total = (uint32_t)errors + refused;
		uint16_t policy_errors = total > 65535 ? 65535 : (uint16_t)total;
		phy.sample[9] = policy_snr; phy.sample[10] = policy_snr >> 8; phy.sample[11] = (uint8_t)policy_errors;
		uint32_t now_ms = (uint32_t)(sample_ticks / 40000);
		if (bw_mcs_settling && (int32_t)(now_ms - bw_mcs_resume_ms) >= 0) { bw_mcs_settling = 0; }
		if (bw_at_sweep || bw_settle_step || bw_mcs_settling) {
			/* Muting/restarting RX during a width change produces errors that
			 * do not describe the settled link. Discard that sample window and
			 * its pending direction; preserve any already-issued MCS request.
			 * Resume with eight fresh samples after the width settling time. */
			dev_mcs.ready = dev_mcs.index = dev_mcs.direction = 0;
		} else {
			dev_mcs_apply(ar_dev_mcs_sample(dev_mcs_context(), now_ms, policy_snr, policy_errors));
			/* The air unit's power, from the same record. A busy control
			 * slot (an MCS request, a hop, a frame switch): the next record
			 * asks again. */
			uint8_t target, direction;
			if (receiver.dev_state == 2 && ar_pwr_auto_record(pwr_auto_context(), now_ms, policy_errors != 0,
			    policy_snr, phy.sample[6], phy.sample[7], &target, &direction)) {
				uint32_t word = ar_pwr_auto_word(target, direction, (uint8_t)(receiver.assigned_slot & 15));
				const uint8_t bytes[4] = {(uint8_t)word, (uint8_t)(word >> 8), (uint8_t)(word >> 16), (uint8_t)(word >> 24)};
				if (!dev_command_request(8, 2, bytes, 4)) { ar_pwr_auto_sent(&pwr_auto, target, direction); }
			}
		}
		if (!mcs_info.first_request_ms && dev_mcs.requests) { mcs_info.first_request_ms = dev_mcs.request_ms; }
	}
}

static void request_recovery(unsigned reason)
{
	if (recovery.pending) { return; }
	recovery.pending = reason;
	/* Stop new submissions immediately. Rebuild only outside packet parsing,
	 * after the current owned RX view has been released. */
	field(0x8000201, 1);
}

static void monitor_reset(unsigned per_user, unsigned user, void *context)
{
	/* Normal mode0 enables logical RX8 only. Defer its global RX reset until
	 * the current DMA view is released, as in the existing recovery path. */
	(void)per_user; (void)user; (void)context; recovery_reset_pending = 1;
}
static void monitor_lost(unsigned user, struct ar_channel_monitor_user *entry, void *context)
{
	(void)user; (void)entry; (void)context;
	if (!recovery.pending) { recovery.fch_losses++; request_recovery(1); }
}
static int monitor_post(unsigned destination, unsigned message, const uint8_t *data,
	size_t length, uint32_t timeout, void *context)
{
	/* No channel-change producer arms a completion yet. Its worker delivery
	 * must be connected before enabling PHY command4/channel policy. */
	(void)destination; (void)message; (void)data; (void)length; (void)timeout; (void)context;
	return -95;
}
static const struct ar_channel_monitor_ops monitor_ops = {
	.reset=monitor_reset,.field=sync_field,.post=monitor_post,.lost=monitor_lost
};
static void monitor_poll(uint64_t now_ms)
{
	if (!monitor_interval_ms || now_ms-monitor_last_ms < monitor_interval_ms) { return; }
	/* Match one timer rearm per processed callback; no catch-up bursts. */
	monitor_last_ms = now_ms;
	(void)ar_channel_monitor_tick(&channel_monitor,&monitor_ops,NULL);
}
static void dev_invalidate(void)
{
	if (automatic_recovery()) {
		recovery.invalidations++; request_recovery(2); return;
	}
	if (receive_operation >= 67) { dev_mcs_apply(ar_dev_mcs_connect(dev_mcs_context(), 0)); }
	ar_pwr_auto_reset(pwr_auto_context());
	dev_peer_power(0);
	ar_frame_stamp_reset(&fstamp); retry_link = 0;
	field(0x8000201, 1); field(0xcf41501, 0);
	link_event(0, receiver.dev_state);
	dev_active = 0; receiver.dev_state = 0; receiver.assigned_slot = 8; bw_state = 0; bw_info.requests = 0;
	bw_due_code = 0;
	receiver.sync_ack_pending = 0; receiver.link_invalidations++;
}

static void dev_command_observed(const uint8_t *data, size_t length, void *context)
{
	(void)context;
	if (data[1] != 2 && !receiver.first_nonstats_length) {
		receiver.first_nonstats_length = length;
		memcpy(receiver.first_nonstats, data, length);
	}
}

static void dev_observe_compact(const struct ar_mac_frame *frame)
{
	struct ar_dev_compact compact;
	int result = ar_dev_compact_decode(frame->outband, frame->outband_length, &compact);
	receiver.last_dev_result = result;
	if (result) { receiver.compact_invalid++; receiver.dev_control_invalid++; return; }
	receiver.compact_valid++; receiver.dev_control_valid++;
	memcpy(receiver.last_compact, frame->outband, 6);
	/* No address/channel fields exist in this form. Never use it for idle
	 * admission or fabricate a full broadcast with an invented peer address. */
	if (!dev_active || receiver.dev_state != 2 || receiver.assigned_slot != 0) { return; }
	if (compact.token != receiver.dev_token || compact.state != 2) { dev_invalidate(); }
}

static void dev_observe_payloads(const struct ar_mac_frame *frame)
{
	if (receive_operation < 59 || !frame->descriptor_count) { return; }
	receiver.payload_packets++;
	for (unsigned i = 0; i < frame->descriptor_count; i++) {
		struct ar_mac_payload payload;
		if (ar_mac_payload_at(frame, i, &payload)) { continue; }
		if (receive_operation >= 71 && payload.port < 5) {
			struct ar_rf_port_observation *item = &traffic_info.ports[payload.port];
			uint64_t packet_ticks = elapsed_ticks();
			uint32_t elapsed = (uint32_t)(packet_ticks / 40);
			if (!item->packets) { item->first_us = elapsed; item->first_sequence = frame->sequence; }
			item->packets++; item->bytes += payload.length;
			item->last_us = elapsed; item->last_sequence = frame->sequence;
			uint64_t second = packet_ticks / 40000000;
			if (second < 16) { traffic_info.port_bytes_by_second[second][payload.port] += payload.length; }
		}
		if (receive_operation >= 60) {
			if (payload.port < 5) {
				receiver.port_packets[payload.port]++; receiver.port_bytes[payload.port] += payload.length;
			} else { receiver.other_port_packets++; receiver.other_port_bytes += payload.length; }
			if (payload.port && !receiver.first_data_length && payload.length) {
				receiver.first_data_port = payload.port; receiver.first_data_length = payload.length;
				memcpy(receiver.first_data, payload.data, payload.length < 128 ? payload.length : 128);
			}
		}
		if (payload.port) { continue; }
		if (receive_operation >= 62 && dev_active) {
			(void)ar_dev_stream_feed(&dev_stream, payload.data, payload.length, dev_command_observed, NULL);
			memcpy(receiver.stream_commands, dev_stream.commands, sizeof receiver.stream_commands);
			receiver.stream_checksum_errors = dev_stream.checksum_errors;
			receiver.stream_length_errors = dev_stream.length_errors;
			receiver.stream_discarded = dev_stream.discarded; receiver.stream_buffered = dev_stream.used;
		}
		size_t copied = payload.length < 132 ? payload.length : 132;
		if (!receiver.port0_packets) {
			receiver.first_port0_length = payload.length;
			memcpy(receiver.first_port0, payload.data, copied);
		}
		receiver.port0_packets++; receiver.port0_bytes += payload.length;
		receiver.last_port0_length = payload.length;
		memset(receiver.last_port0, 0, sizeof receiver.last_port0);
		memcpy(receiver.last_port0, payload.data, copied);
		if (receive_operation < 60) { continue; }
		/* Diagnostic envelope recognition only. A partial command may span
		 * MAC frames; count it as unframed here, not as corrupt stream data. */
		size_t offset = 0;
		while (offset < payload.length) {
			const uint8_t *data = payload.data + offset;
			size_t left = payload.length - offset;
			if (left < 4 || data[0] != 0x4a || data[2] > 128 || (size_t)data[2] + 4 > left) {
				receiver.port0_unframed++; break;
			}
			uint8_t check = 0; size_t length = data[2] + 4;
			for (size_t j = 0; j < length; j++) { check ^= data[j]; }
			if (check) { receiver.port0_unframed++; break; }
			if (data[1] < 8) { receiver.port0_commands[data[1]]++; }
			if (data[1] != 2 && !receiver.first_nonstats_length) {
				receiver.first_nonstats_length = length;
				memcpy(receiver.first_nonstats, data, length);
			}
			offset += length;
		}
	}
}

static void dev_observe_lock(const struct ar_dev_control *control)
{
	if (receive_operation < 57 || !dev_active) { return; }
	uint32_t elapsed = elapsed_us();
	receiver.lock_messages++;
	receiver.lock_slot_states[(control->slots[0] >> 1) & 3]++;
	receiver.lock_observed_extensions |= control->extensions;
	if (control->extensions & 1) {
		if (!receiver.login_extension_packets) {
			receiver.first_login_elapsed_us = elapsed;
			receiver.first_login_length = control->length;
			memcpy(receiver.first_login_control, control->data, control->length);
		}
		receiver.login_extension_packets++;
		receiver.last_login_elapsed_us = elapsed;
	}
	struct ar_dev_lock_context lock = {.selected_slot = 0,
		.assigned_slot = receiver.assigned_slot, .token = receiver.dev_token,
		.transmit_channel = dev_transmit_channel};
	memcpy(lock.peer, receiver.accepted_peer, 4);
	/* A normal state-2 broadcast need not carry another login extension.
	 * A changed token/profile stops TX before delivery. Normal continuous
	 * sessions rebuild after releasing this packet's owned view. */
	if (receive_operation >= 59 && receiver.dev_state == 2) {
		if (memcmp(control->peer, lock.peer, 4)) { return; }
		if (control->mode || control->slot_bitmap != dev_config.slot_bitmap ||
		    control->receive_channel != dev_config.receive_channel ||
		    control->transmit_channel != dev_transmit_channel ||
		    (control->slots[receiver.assigned_slot] >> 8) != receiver.dev_token ||
		    ((control->slots[receiver.assigned_slot] >> 1) & 3) != 2) {
			dev_invalidate();
		}
		return;
	}
	struct ar_dev_login login;
	enum ar_dev_lock_result result = ar_dev_lock_evaluate(&dev_config, &lock, control, &login);
	receiver.last_lock_result = result;
	receiver.lock_results[result]++;
	if (result != AR_DEV_LOCK_ASSIGNMENT) { return; }
	if (!receiver.login_matching_packets) {
		receiver.first_matching_elapsed_us = elapsed;
		receiver.first_matching_length = control->length;
		memcpy(receiver.first_matching_control, control->data, control->length);
	}
	receiver.login_matching_packets++;
	receiver.last_matching_elapsed_us = elapsed;
	receiver.last_login_slot = login.slot; receiver.last_login_token = login.token;
	receiver.last_login_value = login.value; receiver.last_login_ap_state = login.ap_state;
	memcpy(&receiver.last_login_sync, login.sync, 4);
	if (receive_operation < 59) { return; }
	/* This first connected path supports physical user0 / assigned slot0.
	 * 0x232928 requires every remote-sync flag to have been applied before
	 * setting completion. A zero flags byte needs only the four-byte CRC ACK;
	 * unknown/nonzero requests must not be acknowledged as executed. */
	if (login.slot || login.sync[0]) { receiver.sync_unsupported++; return; }
	if (login.ap_state != 2) { return; }
	uint8_t check = 0;
	for (unsigned i = 0; i < 4; i++) { check = ar_mac_crc8_step(check ^ login.sync[i]); }
	(void)ar_dev_command_encode(dev_sync_ack, sizeof dev_sync_ack, 4, &check, 1);
	receiver.assigned_slot = login.slot; receiver.dev_token = login.token;
	receiver.sync_ack_pending = 1;
	/* 0x22ef52: local state2, reset MCS observation, DEV B4C request field
	 * via 0x2236e0 and CF4 bit21 via 0x22b98e(1). Operation75 publishes the
	 * corresponding link transition to registered host observers. */
	field(0xb4c0305, 1); field(0xcf41501, 1);
	link_event(2, receiver.dev_state);
	receiver.dev_state = 2; receiver.connected_elapsed_us = elapsed;
	if (dev_config.pairing) {
		/* Original 0x22ef.. snapshots the peer and a fresh logical-user-8
		 * measurement only after the pairing login reaches state 2. */
		const uint8_t *sample = phy.samples &&
			(uint32_t)(elapsed_us() - phy.sample_elapsed_us) <= 200000 ? phy.sample : NULL;
		ar_binding_complete(&binding, login.slot, receiver.accepted_peer, sample);
	}
	if (receive_operation >= 67) { dev_mcs_apply(ar_dev_mcs_connect(dev_mcs_context(), 1)); }
	ar_pwr_auto_reset(pwr_auto_context());
	ar_frame_stamp_reset(&fstamp); retry_link = 0;
}

static void dev_preencode(void)
{
	if (!dev_active || recovery.pending) { dev_hop_abort(); return; }
	receiver.tx_preencode++;
	receiver.last_tx_user = (read32(BB + 0xf70) >> 12) & 15;
	receiver.last_tx_budget = read32(BB + 0xf5c) >> 16;
	receiver.last_tx_request = (read32(BB + 0xf58) >> 4) & 15;
	if (receive_operation >= 72) {
		/* Original 0x221556 samples the user again for attempt history.
		 * Record before packet construction; this is not an ACK or TX-end. */
		unsigned user = (read32(BB + 0xf70) >> 12) & 15;
		tx_history_info.samples++; tx_history_info.last_user = user;
		if (user) { tx_history_info.other_users++; }
		else {
			const struct ar_interleaver_user *u = &tx_interleaver.users[0];
			if (u->active && u->retries) { tx_history_info.eligible++; }
			(void)ar_tx_irq_history(&tx_interleaver,0,&tx_history_ops,NULL);
		}
	}
	if (receiver.last_tx_user != 0) {
		receiver.tx_errors++; dev_tx_finish_segments(); return;
	}
	(void)ar_dev_tx_retire(&dev_tx, 1);
	if (receive_operation >= 67) {
		dev_mcs_apply(ar_dev_mcs_preencode(dev_mcs_context(),
			elapsed_ms(), receiver.assigned_slot));
	}
	dev_hop_poll();
	struct ar_dev_report report = {.state = receiver.dev_state, .assigned_slot = receiver.assigned_slot,
		.receive_channel = dev_config.receive_channel,
		.slot_value = dev_power_index,
		.status_a = dev_status_a, .status_b = dev_status_b, .pairing = binding.enabled};
	memcpy(report.local, dev_config.local, 4);
	const struct ar_dev_tx_packet *packet;
	unsigned ack = receiver.sync_ack_pending && (receiver.last_tx_budget & ~7u) >= 33;
	unsigned request = !receiver.sync_ack_pending && bw_request_pending && (receiver.last_tx_budget & ~7u) >= 33;
	uint8_t stream[132]; unsigned port = 0;
	size_t length = ack || request ? sizeof dev_sync_ack : 0;
	size_t budget = receiver.last_tx_budget & ~7u;
	/* Keep remote-sync ACK ahead of host streams even when its budget is
	 * temporarily too small. The endpoint retains bytes on every skipped TX. */
	if (!receiver.sync_ack_pending && !request && receive_operation >= 66 && receiver.dev_state == 2 &&
	    receiver.assigned_slot == 0 && budget > 28 && tx_peek_callback && tx_commit_callback) {
		size_t capacity = budget - 28;
		if (capacity > sizeof stream) { capacity = sizeof stream; }
		length = tx_peek_callback(stream, capacity, &port, tx_context);
		if (!length) { port = 0; }
		if (length > capacity || (length && (!port || port > 4))) {
			receiver.tx_errors++; dev_tx_finish_segments(); return;
		}
	}
	int result = ar_dev_tx_prepare_payload(&dev_tx, &report, port,
		ack ? dev_sync_ack : request ? dev_bw_request : stream,
		length, receiver.last_tx_budget, &packet);
	if (result) {
		receiver.tx_errors++; receiver.tx_owned = dev_tx.owned;
		dev_tx_finish_segments(); return;
	}
	/* Original 0x237ca6 submits header then outband as distinct segments,
	 * followed by its terminal mask pulse. Caches remain disabled. */
	ar_cache_clean(packet->bytes, packet->length);
	barrier();
	field(0xd440020, (uint32_t)(uintptr_t)packet->bytes);
	field(0xd400010, packet->header_length); field(0xd401808, 1);
	field(0xd440020, (uint32_t)(uintptr_t)(packet->bytes + packet->header_length));
	field(0xd400010, 16); field(0xd401808, 2);
	if (ack) {
		if (!receiver.sync_ack_submitted) {
			memcpy(receiver.first_sync_ack_packet, packet->bytes, packet->length);
		}
		receiver.sync_ack_pending = 0; receiver.sync_ack_submitted++;
	}
	if (request) { bw_request_pending = 0; bw_info.sent++; }
	if (length) {
		field(0xd440020, (uint32_t)(uintptr_t)(packet->bytes + packet->header_length + 16));
		field(0xd400010, length); field(0xd401808, 4);
	}
	dev_tx_finish_segments();
	if (length && !ack && !request) {
		if (!tx_payload_info.packets) {
			tx_payload_info.first_length = packet->length;
			memcpy(tx_payload_info.first_packet, packet->bytes, packet->length);
		}
		tx_payload_info.packets++; tx_payload_info.bytes += length;
		tx_payload_info.last_length = packet->length; tx_payload_info.last_port = port;
		memset(tx_payload_info.last_packet, 0, sizeof tx_payload_info.last_packet);
		memcpy(tx_payload_info.last_packet, packet->bytes, packet->length);
		tx_commit_callback(port, length, tx_context);
	}
	memcpy(receiver.last_tx_packet, packet->bytes, sizeof receiver.last_tx_packet);
	receiver.last_tx_length = packet->length;
	memset(receiver.last_tx_extended, 0, sizeof receiver.last_tx_extended);
	memcpy(receiver.last_tx_extended, packet->bytes, packet->length < sizeof receiver.last_tx_extended ?
		packet->length : sizeof receiver.last_tx_extended);
	receiver.last_tx_elapsed_us = elapsed_us();
	if (!receiver.tx_packets) { receiver.first_tx_elapsed_us = receiver.last_tx_elapsed_us; }
	receiver.tx_packets++; receiver.tx_owned = dev_tx.owned;
}

static void receive_complete(void)
{
	receiver.dma_completions++;
	if (!receive_pending) { receiver.invalid_packets++; return; }
	receive_length = receive_queue[receive_head].length; receive_user = receive_queue[receive_head].user;
	receive_metadata = receive_queue[receive_head].metadata;
	ar_cache_invalidate(receive_buffers[receive_head], receive_length);
	barrier();
	receiver.last_packet_elapsed_us = elapsed_us();
	if (!receiver.packets) { receiver.first_packet_elapsed_us = receiver.last_packet_elapsed_us; }
	receiver.packets++;
	receiver.bytes += receive_length;
	if (receive_length > capture.rx_max_length) { capture.rx_max_length = receive_length; }
	if (receive_length > 4096) { capture.rx_over_4k++; }
	receiver.last_length = receive_length; receiver.last_user = receive_user;
	receiver.last_rx_metadata = receive_metadata;
	memset(receiver.first_bytes, 0, sizeof receiver.first_bytes);
	memcpy(receiver.first_bytes, receive_buffers[receive_head], receive_length < 64 ? receive_length : 64);
	/* Parse synchronously while this slot is owned by the receive consumer.
	 * Views must not survive advancing the ring. */
	struct ar_mac_frame frame;
	int result = ar_mac_decode(receive_buffers[receive_head], receive_length, &frame);
	receiver.last_mac_result = (uint32_t)result;
	if (result) { receiver.mac_invalid_packets++; }
	else {
		int forwarding_context_valid = 1;
		receiver.mac_valid_packets++;
		capture_packet(receive_buffers[receive_head], receive_length, &frame);
		if (ap_role) {
			ap_receive(&frame);
			receive_head = (receive_head + 1) & 3; receive_pending--;
			return;
		}
		receiver.last_mac_sequence = frame.sequence;
#ifdef AR_BW_TOGGLE_MS
		{
			uint8_t len = (uint8_t)(receive_length / 256 > 255 ? 255 : receive_length / 256);
			bw_seq_head = (bw_seq_head + 1) & 3; bw_seq_ring[bw_seq_head] = (uint8_t)frame.sequence; bw_len_ring[bw_seq_head] = len;
			if (bw_seq_after) {
				const uint8_t seq[] = {0xb4, (uint8_t)(7 - bw_seq_after), (uint8_t)frame.sequence, len};
				publish_event(2, seq, sizeof seq); bw_seq_after--;
			}
		}
#endif
		receiver.last_mac_descriptors = frame.descriptor_count;
		receiver.last_outband_length = frame.outband_length;
		if (listener_capture) {
			/* Explicit bench capture of video only. No control callbacks or
			 * admission: the caller supplied a fixed, trusted RF context.
			 * The short PHY identity is not authentication. */
			if (payload_callback && (receive_user == 0 || receive_user == 8)) {
				for (unsigned i = 0; i < frame.descriptor_count; i++) {
					struct ar_mac_payload payload;
					if (!ar_mac_payload_at(&frame, i, &payload) && payload.port == 4) {
						payload_callback(0, 4, payload.data, payload.length, payload_context);
					}
				}
			}
			receive_head = (receive_head + 1) & 3; receive_pending--;
			return;
		}
		dev_observe_payloads(&frame);
		if (receive_operation >= 60 && frame.outband_length == 6) {
			dev_observe_compact(&frame);
			result = (int)receiver.last_dev_result;
		} else if (frame.outband_length) {
			struct ar_dev_control control;
			result = ar_dev_control_decode(frame.outband, frame.outband_length, &control);
			receiver.last_dev_result = (uint32_t)result;
			if (result == AR_MAC_UNSUPPORTED) { receiver.dev_control_unsupported++; }
			else if (result) { receiver.dev_control_invalid++; }
			else {
				receiver.dev_control_valid++;
				receiver.last_dev_length = control.length;
				receiver.last_dev_sequence = control.sequence;
				receiver.last_dev_channels = control.receive_channel | (uint32_t)control.transmit_channel << 8;
				receiver.last_dev_extensions = control.extensions;
				memcpy(receiver.last_dev_peer, control.peer, 4);
				memset(receiver.last_dev_control, 0, sizeof receiver.last_dev_control);
				memcpy(receiver.last_dev_control, control.data, control.length);
				dev_admit(&control);
				dev_observe_lock(&control);
				forwarding_context_valid = !memcmp(control.peer, receiver.accepted_peer, 4);
			}
		}
		/* Copy external payloads only after this frame's control can invalidate
		 * the admitted context. Internal port 0 has its own command stream. */
		if (receive_operation >= 63 && !recovery.pending && !result && forwarding_context_valid && dev_active && receiver.dev_state == 2 &&
		    receiver.assigned_slot == 0 && payload_callback) {
			for (unsigned i = 0; i < frame.descriptor_count; i++) {
				struct ar_mac_payload payload;
				if (!ar_mac_payload_at(&frame, i, &payload) && payload.port && payload.port < 5) {
					payload_callback(0, payload.port, payload.data, payload.length, payload_context);
				}
			}
		}
	}
	receive_head = (receive_head + 1) & 3;
	receive_pending--;
	dev_mcs_reset_rx();
}

/* A width switch across 40 MHz has no later frame to wait for: the frame
 * changes with it, and an end that moves a frame after the other runs a
 * frame the other does not (the ground's link drops, ~1 s). A receive still
 * owned at that IRQ 5 is taken to its end first: most often its DMA is done
 * and its IRQ 13 only waits for this poll pass (5 is polled before 13 and
 * 16), else a short wait. What is still pending after limit_us is left.
 * Short: the ends re-arm their frames where they switch, and a wait here
 * starts ours that much later than the AP's. */
#define RECEIVE_SETTLE_US 100u
static unsigned receive_settle(uint32_t limit_us)
{
	static const uint8_t order[2] = {13, 16};
	uint32_t started = read32(TIMER);
	for (;;) {
		unsigned took = 0;
		for (unsigned i = 0; i < 2; i++) {
			unsigned index = order[i];
			if (!(read8(0xe0801000u + irq_vector(index) * 4) & 1)) { continue; }
			receiver.irq_counts[index]++; receiver.irq_total++;
			if (TRACE_ON) { ap_trace_irq(index); }
			irq_ack(index);
			if (index == 13) { receive_complete(); } else { receive_event(); }
			took = 1;
		}
		if (!receive_pending && !took) { return 0; }
		if ((uint32_t)(read32(TIMER) - started) >= limit_us * 40) { return receive_pending; }
	}
}

/* ---- AP role (AP04 plan): the air unit's side of the link ------------------
 * The same frame as the DEV with the directions swapped (user 8 transmits the
 * broadcast, user 0 receives the ground's slot); ap.c decides what goes in
 * the control, the radio here sends it and feeds back what the ground sends. */
static struct ar_ap ap_link;
static struct ar_rf_ap_info ap_info = {.version = 1};
static uint8_t ap_mac_sequence, ap_identity_hash, ap_init_mcs = 2;
#ifdef AR_ROLE_AP
#define AP_TX_BUFFERS 6
#define AP_TX_BYTES 12288
#else
#define AP_TX_BUFFERS 2
#define AP_TX_BYTES 1024
#endif
static _Alignas(8) uint8_t ap_tx_buffers[AP_TX_BUFFERS][AP_TX_BYTES];
static unsigned ap_tx_next;
/* The next block's stream bytes, copied into its buffer from the main loop
 * ahead of the pre-encode (at a fixed offset behind the header and the
 * largest control: header, control and stream are separate DMA segments).
 * Built at the pre-encode, a full block (~10 KB at 20 MHz MCS 11) took
 * ~200 us to copy before the encoder could start, and the second block's
 * pre-encode comes only ~0.66 ms before it goes out: with the main loop's
 * own wait, ~2.5% of frames went out broken at MCS 11 (stock: none). The
 * staged bytes stay in the socket until the block takes them; the
 * pre-encode adds what came since, or sends fewer if the budget shrank. */
#define AP_PAYLOAD_AT 216u
#define AP_STAGE_PIECE 1024u
static struct { uint32_t length, budget; unsigned port, buffer; uint8_t valid; } ap_stage;
static void ap_stage_poll(void)
{
	if (ap_link.state != AR_AP_CONNECTED || !tx_peek_at_callback || !tx_available_callback || ap_bw_quiet || ap_bw_prequiet) {
		ap_stage.valid = 0; return;
	}
	if (!ap_stage.valid || ap_stage.buffer != ap_tx_next) {
		ap_stage.valid = 1; ap_stage.buffer = ap_tx_next; ap_stage.length = 0; ap_stage.port = 0;
	}
	/* As much as the last block's budget could carry (a smaller one sends a
	 * prefix), a piece at a time. */
	uint32_t limit = ap_info.budget_last > 12 ? ap_info.budget_last - 12 : 0;
	if (limit > AP_TX_BYTES - AP_PAYLOAD_AT) { limit = AP_TX_BYTES - AP_PAYLOAD_AT; }
	if (ap_stage.length >= limit) { return; }
	uint32_t room = limit - ap_stage.length;
	if (room > AP_STAGE_PIECE) { room = AP_STAGE_PIECE; }
	uint8_t *at = ap_tx_buffers[ap_tx_next] + AP_PAYLOAD_AT + ap_stage.length;
	size_t n = tx_peek_at_callback(&ap_stage.port, ap_stage.length, at, room, tx_context);
	if (!n) { if (!ap_stage.length) { ap_stage.port = 0; } return; }
	ar_cache_clean(at, n);
	ap_stage.length += (uint32_t)n; ap_info.staged_bytes += (uint32_t)n;
}
void ar_rf_note_host_stage(unsigned stage, uint32_t us)
{
	if (stage >= 4) { return; }
	if (us > ap_info.host_max[stage]) { ap_info.host_max[stage] = us; }
	if (us > 200) { ap_info.host_slow[stage]++; }
}
const struct ar_rf_ap_info *ar_rf_ap_information(void)
{
	ap_info.state = ap_link.state; ap_info.token = ap_link.token; ap_info.channel = ap_link.config.channel;
	ap_info.frame_us = ap_frame_us;
	ap_info.gpio_d[0] = read32(0x0a10a038); ap_info.gpio_d[1] = read32(0x0a10a064); ap_info.gpio_d[2] = read32(0x0a10a068);
	ap_info.adc_samples = adc.samples; ap_info.adc_raw = read32(ADC_BASE + 0x10);
	for (unsigned i = 0; i < 8; i++) { ap_info.adc_mv[i] = adc.mv[i]; }
	ap_info.reports = ap_link.reports; ap_info.bad_reports = ap_link.bad_reports;
	ap_info.foreign_reports = ap_link.foreign_reports; ap_info.logins = ap_link.logins;
	ap_info.connects = ap_link.connects; ap_info.drops = ap_link.drops;
	ap_info.acks = ap_link.acks; ap_info.bad_acks = ap_link.bad_acks;
	memcpy(ap_info.ground, ap_link.ground, 4);
	ap_info.phase = information.phase; ap_info.error = information.error;
	ap_info.irq_total = receiver.irq_total;
	memcpy(ap_info.irq_counts, receiver.irq_counts, sizeof ap_info.irq_counts);
	ap_info.reg800 = read32(BB + 0x800); ap_info.reg80c = read32(BB + 0x80c);
	ap_info.regfa8 = read32(BB + 0xfa8); ap_info.rege6c = read32(BB + 0xe6c); ap_info.regf70 = read32(BB + 0xf70);
	ap_info.error_last = receiver.error_last; ap_info.fch_last = receiver.fch_last;
	ap_info.fch_valid = receiver.fch_locked; ap_info.power_last = receiver.power_last;
	ap_info.regf00 = read32(BB + 0xf00); ap_info.regf04 = read32(BB + 0xf04);
	ap_info.regf08 = read32(BB + 0xf08); ap_info.regf0c = read32(BB + 0xf0c);
	ap_info.receive_events = receiver.receive_events; ap_info.invalid_packets = receiver.invalid_packets;
	ap_info.dma_completions = receiver.dma_completions; ap_info.rx_pending = receive_pending;
	ap_info.rx_metadata = read32(BB + 0xf90); ap_info.regf88 = read32(BB + 0xf88);
	ap_info.regf90 = receive_metadata; ap_info.regbe8 = read32(BB + 0xbe8);
	return &ap_info;
}
/* The AP's transmit identity for user 8 (the PHY scrambler): 0 while idle,
 * as a searching ground listens with 0; its own hash once command 6 has told
 * the ground to switch. Byte (8 + 2) & 3 of 0xb84 and the shared 0xb60 field. */
static void ap_set_identity(uint8_t value)
{
	field(0xb841008, value); field(0xb601008, value);
	ap_info.identity = value;
}
static void ap_link_changed(unsigned previous)
{
	unsigned now = ap_link.state;
	if (now == previous) { return; }
	if (now == AR_AP_LOGIN && !ap_trace.freeze_at) { ap_trace.freeze_at = ap_trace.head + 120; }
	if (now == AR_AP_CONNECTED) {
		ap_link_stamps = phy_control.stamps;
		ap_pwr_announce = 1;   /* the ground learns our TX power */
		/* Binding: the ground that logged in is the pairing result (GET 1). */
		if (binding.enabled) { ar_binding_complete(&binding, 0, ap_link.ground, NULL); }
		/* Stock after login: command 3 (TTL 31) asks for the ground's
		 * uplink MCS, then command 6 (TTL 5) moves user 8's identity to the
		 * AP's hash on both ends. One command at a time: 6 goes first here,
		 * it decides whether the ground keeps decoding us. */
		const uint8_t identity[2] = {0x88, ap_identity_hash};
		/* The block may hold the frame stamp (one frame in 100 ms): then
		 * from ap_poll as soon as it is free. Left unsent, the identity
		 * stayed 0 on both ends: the uplink budget halved, and the short
		 * frame then lost the link. */
		if (ar_phy_control_enqueue(&phy_control, 6, 5, identity, 2, &phy_ops, NULL)) { ap_identity_pending = 1; }
		else { ap_identity_pending = 0; ap_info.phy_sent++; }
		/* Stock on a connected slot: broadcast retransmission on (0xb20 bit
		 * 30 clear) and its requests taken (0xcf4 bit 21, "retx req
		 * enabled"). On with no ground connected, it stops the TX engine. */
		field(0xb201e01, 0); field(0xcf41501, 1);
		ar_dev_retx_init(&ap_retx, dev_retx.threshold ? dev_retx.threshold : AP_RETX_THRESHOLD);
		ap_info.retx_threshold = ap_retx.threshold;
		/* The AP time (command 7) goes once the PHY channel is free, not a
		 * minute after boot: until then the ground has no AP clock. */
		phy_control.time_value = (uint32_t)(timer_ticks() / 40000) - 60001; phy_control.time_valid = 0;
		/* And then every second, not every minute: one lost header (a frame or
		 * bandwidth switch right after the login) left the ground without an
		 * AP clock for a minute - the host's latency figures fell back to an
		 * estimate. */
		phy_control.time_period_ms = 1000;
		link_event(2, 0);
	} else if (previous == AR_AP_CONNECTED || now == AR_AP_IDLE) {
		(void)ar_phy_control_cancel(&phy_control, &phy_ops, NULL);
		ap_uplink_mcs_pending = 0; ap_chan_pending = 0; ap_identity_pending = 0; ap_chan_host_pending = 0;
		ap_up_connect(0);
		/* A hop the host asked for still happens; one of ours is dropped. */
		if (ap_chan_hop && !ap_chan_hop_auto) { ap_chan_at_sweep = ap_chan_hop; }
		ap_chan_hop = 0;
		/* Lost soon after our own hop: back to where the ground was. */
		if (ap_auto_hopping && elapsed_ms() - ap_auto_hop_ms < 2000) {
			ap_chan_at_sweep = 0x10000 | ap_auto_from; ap_info.hop_rollbacks++;
		}
		ap_auto_hopping = 0; ar_hop_measure_reset(&ap_measure);
		ap_set_identity(0);
		field(0xb201e01, 1); field(0xcf41501, 0);   /* no retransmission without a ground */
		/* ... and no watchdog phase left over: a switch's re-seed would turn
		 * it back on (a link lost at 20/40 MHz, the switch to 5 after). */
		ar_dev_retx_init(&ap_retx, ap_retx.threshold);
		ap_stall_frames = 0; ap_stall_off = 0;
		/* A new link starts at 5 MHz and the first MCS: at IRQ 5, as every
		 * width change (an engine hold from here lands mid-TX: the broadcast
		 * stopped for good, with no ground to re-link to). */
		ap_bw_pending = 0; ap_bw_at_sweep = 0; ap_bw_quiet = 0; ap_bw_prequiet = 0;
		if (ap_bw_code != 2) { ap_bw_at_sweep = 0x80 | 2; }
		ap_frame_pending = 0;
		if (ap_frame_mode) { ap_frame_at_sweep = 0x80; }   /* a new link starts on the standard frame */
		/* Automatic power: back to the host's maximum for the next link. */
		ap_pwr_pending = 0;
		if (ap_pwr_min < ap_pwr_max && power_level[8] != 0xff && power_level[8] != ap_pwr_max) { (void)apply_power(8, ap_pwr_max); }
		ap_set_mcs(ap_init_mcs);
		if (previous == AR_AP_CONNECTED) { link_event(0, 2); }
	}
}
static void ap_command_received(const uint8_t *envelope, size_t length, void *context)
{
	(void)context;
	unsigned previous = ap_link.state, request = ap_link.bandwidth_request;
	ap_info.commands++;
	ar_ap_command(&ap_link, envelope, length, elapsed_ms());
	if (ap_link.bandwidth_request != request) { ap_info.bandwidth_requests++; }
	ap_link_changed(previous);
}
/* One block's packet: header, the control, and while connected the host's
 * stream bytes, built in place in a TX buffer the hardware reads by DMA. */
static void ap_preencode(void)
{
	ap_info.preencodes++;
#ifdef AR_BW_TRACE
	if (ap_trace_after && !ap_trace_pre) { ap_trace_pre = 1; ap_trace_item(3, timer_ticks() - ap_trace_switch); }
#endif
	ap_info.preencode_requests[(read32(BB + 0xf58) >> 4) & 15]++;
	if (read8(0xe0801000u + irq_vector(4) * 4) & 1) { ap_info.preencode_late++; }
	unsigned user = (read32(BB + 0xf70) >> 12) & 15;
	uint32_t budget = (read32(BB + 0xf5c) >> 16) & ~7u;
	ap_info.last_user = user; ap_info.budget_last = budget;
	if (budget > ap_info.budget_max) { ap_info.budget_max = budget; }
	if (!ap_info.budget_min || budget < ap_info.budget_min) { ap_info.budget_min = budget; }
	if (user != 8) { ap_info.other_users++; dev_tx_finish_segments(); return; }
	uint8_t *packet = ap_tx_buffers[ap_tx_next];
	uint8_t control[200];
	size_t control_length = ar_ap_control(&ap_link, control, sizeof control);
	size_t aligned = (control_length + 3) & ~(size_t)3, header = 12;
	if (!control_length || budget < header + aligned || budget > AP_TX_BYTES) {
		ap_info.errors++; dev_tx_finish_segments(); return;
	}
	/* Stream bytes: the ones staged in this buffer (ap_stage_poll) and what
	 * came since, behind a one-descriptor header. */
	size_t length = 0; unsigned port = 0;
	unsigned quiet = ap_bw_quiet || ap_bw_prequiet;
	uint8_t *payload = packet + AP_PAYLOAD_AT;
	if (ap_link.state == AR_AP_CONNECTED && tx_peek_callback && tx_commit_callback && !quiet) {
		size_t room = budget - header - aligned;
		if (room > AP_TX_BYTES - AP_PAYLOAD_AT) { room = AP_TX_BYTES - AP_PAYLOAD_AT; }
		size_t staged = 0;
		if (ap_stage.valid && ap_stage.buffer == ap_tx_next && ap_stage.length && tx_available_callback &&
		    tx_available_callback(ap_stage.port, tx_context) >= ap_stage.length) {
			staged = ap_stage.length < room ? ap_stage.length : room; port = ap_stage.port;
			ap_info.staged_used += (uint32_t)staged;
		}
		if (staged < room && staged && tx_peek_at_callback) {
			size_t more = tx_peek_at_callback(&port, staged, payload + staged, room - staged, tx_context);
			ar_cache_clean(payload + staged, more);
			length = staged + more;
		} else if (staged) {
			length = staged;
		} else if (room) {
			length = tx_peek_callback(payload, room, &port, tx_context);
			ar_cache_clean(payload, length);
		}
		if (length) { ap_info.payload_packets++; } else { ap_info.empty_packets++; }
		if (length > room || (length && (!port || port > 4))) { ap_info.errors++; length = 0; }
	}
	ap_stage.valid = 0;
	if (!length) { header = 8; }
	memset(packet, 0, header + aligned);
	packet[0] = 0x58; packet[4] = ap_mac_sequence++; packet[6] = (uint8_t)control_length;
	if (length) {
		uint32_t word = (uint32_t)length << 12 | port;
		packet[7] = 1;
		for (unsigned b = 0; b < 4; b++) { packet[8 + b] = word >> (8 * b); }
	}
	memcpy(packet + header, control, control_length);
	uint16_t check = ar_mac_checksum16(packet, header);
	packet[2] = (uint8_t)check; packet[3] = check >> 8;
	ar_cache_clean(packet, header + aligned);   /* the stream bytes were cleaned as they came in */
	barrier();
	field(0xd440020, (uint32_t)(uintptr_t)packet);
	field(0xd400010, header); field(0xd401808, 1);
	field(0xd440020, (uint32_t)(uintptr_t)(packet + header));
	field(0xd400010, aligned); field(0xd401808, 2);
	if (length) {
		/* Segments longer than 4 KiB come out empty at the ground (stock's
		 * stream arrives in socket-buffer pieces): at most 4000 bytes each,
		 * on the next segment bits (at most eight segments a packet). */
		const uint8_t *data = payload; size_t left = length; unsigned bit = 2;
		while (left && bit < 8) {
			size_t piece = left > 4000 && bit < 7 ? 4000 : left;
			field(0xd440020, (uint32_t)(uintptr_t)data);
			field(0xd400010, piece); field(0xd401808, 1u << bit);
			data += piece; left -= piece; bit++;
		}
	}
	dev_tx_finish_segments();
	if (length) { tx_commit_callback(port, length, tx_context); ap_info.payload_bytes += length; }
#ifdef AR_BW_TRACE
	ap_trace_pre_at[ap_trace_pre_head] = timer_ticks(); ap_trace_pre_stream[ap_trace_pre_head] = !!length;
	{
		uint16_t pkt = (uint16_t)((uint8_t)(ap_mac_sequence - 1) | (length / 256 > 255 ? 255 : length / 256) << 8);
		ap_trace_pkt[ap_trace_pre_head] = pkt;
		if (ap_trace_pkt_after) { ap_trace_raw(33 - ap_trace_pkt_after, pkt); ap_trace_pkt_after--; }
	}
	ap_trace_pre_head = (ap_trace_pre_head + 1) & 3;
#endif
	ap_tx_next = (ap_tx_next + 1) % AP_TX_BUFFERS;
	ap_info.packets++; ap_info.bytes += header + aligned + length;
	memcpy(ap_info.last_packet, packet, sizeof ap_info.last_packet);
}
/* What the ground sent up in its slot: its report, port-0 commands, and its
 * stream bytes for the host once connected. */
static void ap_receive(const struct ar_mac_frame *frame)
{
	ap_info.uplink_packets++;
	unsigned previous = ap_link.state;
	if (frame->outband_length >= 16) {
		memcpy(ap_info.last_report, frame->outband, 16);
		(void)ar_ap_report(&ap_link, frame->outband, frame->outband_length, elapsed_ms());
	}
	ap_link_changed(previous);
	for (unsigned i = 0; i < frame->descriptor_count; i++) {
		struct ar_mac_payload payload;
		if (ar_mac_payload_at(frame, i, &payload)) { continue; }
		if (!payload.port) {
			(void)ar_dev_stream_feed(&dev_stream, payload.data, payload.length, ap_command_received, NULL);
		} else if (payload.port < 5 && ap_link.state == AR_AP_CONNECTED && payload_callback) {
			payload_callback(0, payload.port, payload.data, payload.length, payload_context);
		}
	}
}
/* The AP's radio set-up over what the DEV path left: values a stock 18.21.10
 * AP holds (register dumps of a linked stock AP on an air unit), less
 * anything live (gain state, calibration results, holds, DMA). */
static void ap_tune(uint64_t pll, uint32_t frequency)
{
	uint64_t park;
	if (ar_rf_pll_word(2100000, &park)) { park = pll; }
	static const uint16_t first[10] = {0x93a,0x940,0x945,0x94a,0x950,0x955,0x95a,0x960,0x905,0x96a};
	static const uint16_t other[10] = {0x910,0x915,0x91a,0x920,0x925,0x92a,0x930,0x935,0x900,0x965};
	for (unsigned user = 0; user < 10; user++) {
		/* Stock AP: users 0-8 receive on the channel, user 8 transmits on
		 * it, only user 9 is parked. */
		serial_word(first[user], user < 9 ? pll : park);
		serial_word(other[user], user == 8 ? pll : park);
		tuned_frequency[user][1] = user < 9 ? frequency : 2100000;
		tuned_frequency[user][0] = user == 8 ? frequency : 2100000;
		field((uint32_t)(0xa80 + (user / 2) * 4) << 16 | (user & 1 ? 0x1010 : 0x0010),
			user < 9 ? frequency / 2000 : 0x041a);
	}
	serial_word(0x90a, pll);
}
/* A hop: every receive bank and the broadcast's TX bank to the new channel,
 * and the broadcast offers its index (a ground's report must carry it). */
static void ap_retune(unsigned index)
{
	uint64_t pll;
	if (index >= receiver.channel_count || ar_rf_pll_word(receiver.channels[index].frequency_khz, &pll)) {
		ap_info.channel_errors++; return;
	}
	unsigned previous = receiver.channel_index;
	ap_tune(pll, receiver.channels[index].frequency_khz);
	power_select_band(receiver.channels[index].frequency_khz);
	receiver.channel_index = index; receiver.frequency_khz = receiver.channels[index].frequency_khz;
	receiver.channels[index].visits++;
	struct ar_ap_config config = ap_link.config; config.channel = (uint8_t)index;
	ar_ap_configure(&ap_link, &config);
	ap_info.channel_moves++;
	const uint8_t data[] = {0, 1, (uint8_t)index, (uint8_t)previous};
	publish_event(2, data, sizeof data);
}
/* SET 0x1b {slot, min, max}: the range of the slot's uplink MCS (stock
 * 0x220210 bounds the slot's MCS policy rows). An air host: {0, 2, 2} (its
 * video strategy 2: {0, 1, 2}). No uplink policy here: the ground is asked
 * for the top of the range at each link-up. */
int ar_rf_host_uplink_mcs_range(const uint8_t *in, size_t length)
{
	if (!in || length < 3) { return -22; }
	if (in[0] > 7 || in[1] > in[2]) { return -6; }
	if (!ap_role || in[0]) { return 0; }
	ap_uplink_mcs_max = in[2] < 16 ? in[2] : 0xff;
	ap_up_min = in[1]; ap_up_max = in[2];
	ap_up_build();
	return 0;
}
/* Our command 4 [0x70, index] (TTL 16): the ground hops as it receives the
 * last TTL, we at the IRQ 5 after the expiry. From an AP the flags say what
 * the DEV moves (sync_channel): 0x40 its slot TX, 0x10 its broadcast TX,
 * 0x20 its broadcast RX - all three, the whole link. 0x80 (what a DEV sends
 * for its own hop) moves nothing on a mode-0 DEV: with it the ground stayed
 * and lost the link at every host or automatic hop. */
static int ap_hop_start(unsigned index, unsigned automatic)
{
	const uint8_t hop[3] = {0x70, (uint8_t)index, (uint8_t)(index >> 8)};
	if (ar_phy_control_enqueue(&phy_control, 4, 16, hop, 3, &phy_ops, NULL)) { return -16; }
	ap_chan_hop = 0x10000 | index; ap_chan_hop_auto = (uint8_t)automatic; ap_info.phy_sent++;
	return 0;
}
static int ap_host_channel(const uint8_t *payload, size_t length)
{
	/* The host's channel (the air host: {1, index}, at every link-up and
	 * when it finds the radio off its channel). Linked: command 4, TTL
	 * 16, both ends hop as it expires (stock "set slot tx chan"). */
	if (length < 2) { return -22; }
	unsigned index = payload[1];
	if (index >= receiver.channel_count) { return -6; }
	if (index == receiver.channel_index && !ap_chan_hop && !ap_chan_at_sweep) { ap_chan_host_pending = 0; return 0; }
	if (ap_link.state != AR_AP_CONNECTED) { ap_chan_at_sweep = 0x10000 | index; ap_chan_host_pending = 0; return 0; }
	if (ap_chan_hop) { return (ap_chan_hop & 0xffff) == index ? 0 : -16; }
	/* The block busy (the frame stamp holds it one frame in 100 ms): the
	 * request is kept and goes from ap_poll, instead of a -16 the host had
	 * to come back from. */
	if (ap_hop_start(index, 0)) { ap_chan_host_pending = 0x10000 | index; }
	else { ap_chan_host_pending = 0; }
	return 0;
}

/* ---- The ground's uplink MCS (stock's slot policy, ap.mcs.slot) --------
 * The engine of a DEV's broadcast policy (ar_dev_mcs), the other way round
 * (stock PHY 0x103 / 0x106): samples are the ground's packets' SNR (0xea0)
 * and LDPC error word (0xf08) at their end-of-packet interrupt; the MCS it
 * sends is read from its FCH key; a change goes to it as command 3 [slot 0,
 * mcs | 0xc0] (TTL 31) until its FCH shows it. SET 0x1b's range keeps the
 * rows inside it (stock 0x220210); with the policy off, the range's top is
 * asked once per link. */
static struct ar_dev_mcs ap_up;
static struct ar_dev_mcs_config ap_up_config;
static unsigned ap_up_active(void) { return ap_up_ready && ap_up.config.enabled; }
static struct ar_dev_mcs *ap_up_context(void) { memcpy(ap_up.fch, phy_control.tx, 7); return &ap_up; }
static void ap_up_apply(unsigned action)
{
	if (action & AR_DEV_MCS_SYNC_ON) {
		/* Taken when the block is free; else re-advertised a few frames on. */
		if (!ar_phy_control_enqueue(&phy_control, 3, 31, ap_up.fch + 3, 2, &phy_ops, NULL)) { ap_info.uplink_requests++; }
	}
	/* Only our own request: the block may hold another command by now. */
	if ((action & AR_DEV_MCS_SYNC_OFF) && (phy_control.tx[1] & 15) == 3) {
		(void)ar_phy_control_cancel(&phy_control, &phy_ops, NULL);
	}
	if (action & AR_DEV_MCS_APPLY_BUFFER) {
		ap_info.uplink_changes++;
		ap_info.uplink_mcs = ap_up.current < ap_up.config.count ? ap_up.config.rows[ap_up.current].mcs : 25;
	}
	if (action & AR_DEV_MCS_RESET_RX) { ap_info.uplink_timeouts++; }
	memcpy(ap_up.fch, phy_control.tx, 7);
}
static void ap_up_build(void)
{
	if (!ap_up_config.count) { ap_up_ready = 0; return; }
	struct ar_dev_mcs_config c = ap_up_config;
	unsigned n = 0;
	memset(c.rows, 0, sizeof c.rows);
	for (unsigned i = 0; i < ap_up_config.count; i++) {
		const struct ar_dev_mcs_row *row = &ap_up_config.rows[i];
		if (row->mcs >= ap_up_min && row->mcs <= ap_up_max) { c.rows[n++] = *row; }
	}
	if (!n) {   /* none inside: the highest row not above it, else the first */
		unsigned pick = 0;
		for (unsigned i = 0; i < ap_up_config.count; i++) { if (ap_up_config.rows[i].mcs <= ap_up_max) { pick = i; } }
		c.rows[n++] = ap_up_config.rows[pick];
	}
	c.count = (uint8_t)n;
	unsigned connected = ap_up.connected;
	if ((phy_control.tx[1] & 15) == 3) { (void)ar_phy_control_cancel(&phy_control, &phy_ops, NULL); }
	ap_up_ready = !ar_dev_mcs_init(&ap_up, &c);
	if (ap_up_ready && connected) { ap_up_apply(ar_dev_mcs_connect(ap_up_context(), 1)); }
}
static void ap_up_measure(uint32_t status)
{
	unsigned user = (status >> 20) & 15;
	if (user == 8 || user == 9) { return; }   /* the AP receives only the ground's slot */
	uint32_t old = read32(BB + 0xbb0);
	field(0xbb00701, 0); uint32_t errors = read32(BB + 0xf08); field(0xbb00701, (old >> 7) & 1);
	old = read32(BB + 0xc00);
	field(0xc000701, 0); uint32_t snr = read32(BB + 0xea0); field(0xc000701, (old >> 7) & 1);
	ap_info.uplink_samples++; ap_info.uplink_snr = (uint16_t)snr;
	if ((uint16_t)errors) { ap_info.uplink_errors++; }
	if (ap_up_active()) {
		ap_up_apply(ar_dev_mcs_sample(ap_up_context(), elapsed_ms(), (uint16_t)snr, (uint16_t)errors));
	}
}
static void ap_up_observe(unsigned mcs)
{
	ap_info.uplink_observed = mcs;
	if (ap_up_active()) { ap_up_apply(ar_dev_mcs_observe(ap_up_context(), (uint8_t)mcs)); }
}
static void ap_up_connect(unsigned connected)
{
	if (ap_up_ready) { ap_up_apply(ar_dev_mcs_connect(ap_up_context(), connected)); }
}
static void ap_up_tick(void)
{
	if (ap_up_active()) { ap_up_apply(ar_dev_mcs_preencode(ap_up_context(), elapsed_ms(), 0)); }
}

/* ---- Channels by the AP itself (stock bb_phy_chan_policy_pre_proc) -----
 * In the host's auto mode (SET 5 {1}; the configuration's "manu" starts it
 * off) with a ground connected. Each frame's broadcast HARQ request says
 * whether a block went again; the measure is the longer of the run of such
 * frames (less one) and their count in the last 16. Stock's tiers
 * (hop_para {retx_cnt, power_diff}): the first tier whose count is not
 * below the measure needs the quietest work channel (the scan's levels) to
 * be power_diff dB under ours; a measure past every tier moves to it
 * regardless. From blind_errors on, at most every blind_ms, a blind hop:
 * to that channel, else the next one on the work list. A hop is command 4
 * like the host's; the link lost within 2 s of it goes back ("safe hop").
 * init_chan -1: the start channel is the quietest work channel, once the
 * scan has measured them all, if no ground has logged in by then. */
static struct ar_hop_config ap_hop = {.blind_ms = 3000, .blind_errors = 16, .tier_count = 3,
	.tiers = {{3, 20}, {6, 10}, {9, 5}}};
void ar_rf_set_hop_config(const struct ar_hop_config *hop)
{
	if (!hop) { return; }
	ap_hop = *hop;
	if (ap_hop.tier_count > AR_HOP_TIERS) { ap_hop.tier_count = AR_HOP_TIERS; }
}
static void ap_hop_feed(uint8_t first, uint8_t second)
{
	/* The frame's count for the resend map: each id that is not one or two
	 * past the newest seen is a block sent again (as the measure counts). */
	if (ap_measure.newest != 255) {
		ap_frame_resent += ((unsigned)(first - ap_measure.newest) & 15u) - 1u >= 2u;
		if (second != 255) { ap_frame_resent += ((unsigned)(second - ap_measure.newest) & 15u) - 1u >= 2u; }
	}
	ap_frame_fed = 1;
	if (ar_hop_measure_feed(&ap_measure, first, second)) { ap_info.hop_retx_frames++; }
	ap_hop_evaluate = 1;
}
static int ap_channel_index(uint32_t frequency_khz)
{
	for (unsigned i = 0; i < receiver.channel_count; i++) {
		if (receiver.channels[i].frequency_khz == frequency_khz) { return (int)i; }
	}
	return -1;
}
static void ap_auto_channel(void)
{
	uint32_t now = elapsed_ms();
	if (ap_auto_hopping && now - ap_auto_hop_ms > 2000) { ap_auto_hopping = 0; ap_info.hop_kept++; }
	/* The scan has ten rounds of every channel: the quietest work channel
	 * (the scan's own start-up choice needs a band set only a DEV has). */
	if (ap_hop.measure_start && !ap_measured_start && ap_link.state == AR_AP_IDLE && !ap_link.logins &&
	    scan_scores.ready) {
		ap_measured_start = 1;
		int quietest = -1; int32_t level = 0;
		for (unsigned r = 0; r < scan_info.count && r < AR_RF_MAX_CHANNELS; r++) {
			if (!(scan_rows[r] & (UINT32_C(1) << 24)) || !scan_history[r].score) { continue; }
			int index = ap_channel_index(scan_rows[r] & 0xffffff);
			if (index >= 0 && (quietest < 0 || scan_history[r].score < level)) { quietest = index; level = scan_history[r].score; }
		}
		if (quietest >= 0 && (unsigned)quietest != receiver.channel_index && !ap_chan_at_sweep) {
			ap_chan_at_sweep = 0x10000 | (unsigned)quietest; ap_info.measured_starts++;
		}
	}
	if (!ap_hop_evaluate) { return; }
	ap_hop_evaluate = 0;
	unsigned measure = ar_hop_measure_value(&ap_measure);
	ap_info.hop_measure = measure;
	if (!chan_auto || ap_link.state != AR_AP_CONNECTED || !measure || ap_chan_hop || ap_chan_at_sweep ||
	    ap_chan_pending || (ap_auto_hop_ms && now - ap_auto_hop_ms < 500)) { return; }
	int best = -1, own = -1;
	int32_t best_level = 0, own_level = 0;
	for (unsigned r = 0; r < scan_info.count && r < AR_RF_MAX_CHANNELS; r++) {
		if (!(scan_rows[r] & (UINT32_C(1) << 24)) || !scan_history[r].score) { continue; }   /* work list, measured */
		int index = ap_channel_index(scan_rows[r] & 0xffffff);
		if (index < 0) { continue; }
		if ((unsigned)index == receiver.channel_index) { own = index; own_level = scan_history[r].score; continue; }
		if (best < 0 || scan_history[r].score < best_level) { best = index; best_level = scan_history[r].score; }
	}
	uint32_t blind_before = ap_blind_ms;
	enum ar_hop_decision decision = ar_hop_decide(&ap_hop, measure, own >= 0, own_level, best >= 0, best_level,
		now, &ap_blind_ms);
	if (decision == AR_HOP_STAY) { return; }
	int target = best;
	if (decision == AR_HOP_BLIND) {
		ap_info.hop_blind++;
		/* Nothing measured: the next channel on the work list. */
		for (unsigned step = 1; step <= receiver.channel_count && target < 0; step++) {
			unsigned index = (receiver.channel_index + step) % receiver.channel_count;
			int r = scan_row_of(receiver.channels[index].frequency_khz);
			if (r >= 0 && (scan_rows[r] & (UINT32_C(1) << 24))) { target = (int)index; }
		}
		if (target < 0) { return; }
	} else { ap_info.hop_by_power++; }
	if ((unsigned)target == receiver.channel_index || ap_hop_start((unsigned)target, 1)) {
		ap_blind_ms = blind_before;   /* not sent: a blind hop is still due */
		return;
	}
	ap_auto_from = (uint8_t)receiver.channel_index; ap_auto_hop_ms = now; ap_auto_hopping = 1;
	ar_hop_measure_reset(&ap_measure);
	ap_info.hop_last = receiver.channel_index | (uint32_t)target << 8 | measure << 16;
}
static void ap_configure_radio(uint64_t pll, uint32_t frequency)
{
	ap_tune(pll, frequency);
	/* Per-user receive profiles; keep the gain bits [13:0] power set. */
	for (unsigned user = 0; user < 10; user++) {
		/* User 0 receives the ground's slot. A stock AP on the standard
		 * frame holds 0x3cb0a12b (selector 3, interleave 5), on the short
		 * one 0x3cb0012a (2, 0): the profile follows the frame mode. */
#ifdef AR_AP_SHORT_FRAME
		uint32_t rx = user == 0 ? 0x3cb0012a : user == 8 ? 0x3cb0a2ac : 0x3cb0a12b;
#else
		uint32_t rx = user == 8 ? 0x3cb0a2ac : 0x3cb0a12b;
#endif
		uint32_t band = user == 8 ? 0x88cc0000 : user == 9 ? 0x22cc0000 : 0x28cc0000;
		write32(BB + 0x9a0 + user * 8, rx);
		update(BB + 0x9a4 + user * 8, 0xffffc000, band);
		/* TX profiles: user 8 starts at 5 MHz, MCS 0 (stock's first link
		 * word); the others are not sent from but carry stock's fields. */
		write32(BB + 0xb00 + user * 4, user == 8 ? 0x14486032 : 0x14093021);
	}
	/* Broadcast HARQ retransmission: off until a ground connects, as stock
	 * builds its frame; ap_link_changed() turns it on with its requests. */
	field(0xb201e01, 1); field(0xcf41501, 0);
	write32(BB + 0x9f0, 0x3cb00488); update(BB + 0x9f4, 0xffffc000, 0); update(BB + 0x9fc, 0xffffc000, 0x08550000);
	field(0x9f80803, 1);                 /* user 11 receives at 2.5 MHz on a stock AP (the DEV path left 20) */
	write32(BB + 0x980, 0x068e009c); write32(BB + 0x98c, 0x000f0000);
	/* Windows and durations of every user as stock leaves them; the DEV path
	 * left the second 0x970 window open (0xffff, its monitor setting). */
	write32(BB + 0x970, 0x12501250); write32(BB + 0x988, 0x000009c4); write32(BB + 0x99c, 0x0b320b32);
	for (unsigned offset = 0xa04; offset <= 0xa1c; offset += 4) {
		if (offset != 0xa10) { write32(BB + offset, 0x0b320b32); }
	}
	/* 0xa10 [15:0] is the AP's slot duration: 0x0da3 makes the standard
	 * 5634 us frame (mode 0) a ground starts on; 0x07a3 the short one. */
#ifdef AR_AP_SHORT_FRAME
	write32(BB + 0xa00, 0x0b320da3); write32(BB + 0xa10, 0x0b3207a3);
#else
	write32(BB + 0xa00, 0x0b320da3); write32(BB + 0xa10, 0x0b320da3);
#endif
	write32(BB + 0xa20, 0x400300ca); write32(BB + 0xa2c, 0x00000fff);
	write32(BB + 0xa48, 0); write32(BB + 0xa68, 0x100a0931); write32(BB + 0xafc, 0x40020008);
	update(BB + 0x80c, 0x00980000, 0x00980000); update(BB + 0x808, 0x4000, 0x4000);
	write32(BB + 0x010, 0x40800000); write32(BB + 0x04c, 0xb0000000);
	update(BB + 0x800, 0x80, 0x80);      /* set on every stock AP, never on a DEV */
	write32(BB + 0x08c, 0x01000080); write32(BB + 0x090, 0);
	update(BB + 0xb54, 0x40000000, 0x40000000); write32(BB + 0xb70, 0xcc200000);
	write32(BB + 0xb7c, 0x00000400); write32(BB + 0xb84, 0); write32(BB + 0xb60, 0x0c003c00);
	write32(BB + 0xd20, 0); write32(BB + 0xd24, 0); write32(BB + 0xd2c, 0);
	write32(BB + 0xd30, 0xff00ffff);     /* ranging: AP role on every slot */
	/* Receive buffer map: the DEV maps its broadcast user 8 to buffer 0
	 * (0); the stock AP holds 1, its receive user being the slot's. */
	field(0xbf00002, 1);
	write32(BB + 0xaf4, 0x0002870b);     /* stock's interrupt source masks */
	/* Path B off, as the air unit's host asks at start (PRJ_CMD_RF_PATH_CTRL
	 * path 1, TX and RX, 0): its TX/RX enables, then path A's weights alone. */
	field(0x0000301, 0); field(0x0000501, 0);
	field(0xc000101, 0); field(0xa200701, 1);
	field(0xa2c000c, 0xfff); field(0xa2c100c, 0); field(0xc001701, 0);
	/* The AP receives the ground's slot with its own identity (the ground
	 * switches to the AP's hash at admission); the DEV path cleared these. */
	field(0xde00008, ap_identity_hash); field(0xdf00008, ap_identity_hash);
	ap_set_identity(0);
	(void)ar_tx_mcs_control(&tx_interleaver, AR_TX_MCS_INITIALIZE, 8, 0, mcs_fch_keys,
		information.chip_profile, 0, &tx_mcs_ops, NULL);
	ap_init_mcs = tx_interleaver.users[8].mcs; ap_info.broadcast_mcs = ap_init_mcs;
	/* PHY control as the AP sends it. "wake" pauses ranging (0xd30[23:16])
	 * while a command is queued: ranging shares the header field, so without
	 * it the block never goes on air (grounds see ranging bytes instead). */
	phy_control.role = 0; phy_control.mode = 0; phy_control.wake = 1; phy_control.callbacks = 0x17e;
	field(0x8000201, 0);                 /* the AP transmits from the start */
	ap_tx_next = 0; ap_mac_sequence = 0;
	memset(&ap_info, 0, sizeof ap_info); ap_info.version = 1;
	ap_info.calibration_rounds = information.calibration_rounds;
	ap_info.calibration_status = information.calibration_status;
	ap_info.configured_us = elapsed_us();
	ap_rearm_pending = 0; ap_rx_late = 0; ap_fch_bad = 0; ap_sweep_us = 0;
	/* The broadcast transmits at the configured first level (the host's
	 * SET 8 {8, dBm} moves it); user 8 holds a DEV's receive values here. */
	if (dev_power_index >= 9) { (void)apply_power(8, dev_power_index); }
	ap_bw_code = 2; ap_bw_pending = 0; ap_info.bandwidth = 2; ap_info.broadcast_mcs = ap_init_mcs;
}
int ar_rf_ap_decoder_reset(void)
{
	if (!ap_role) { return -95; }
	/* Hold the baseband (0x800 bits 0 and 3), request both engine flushes
	 * (0x80c bits 2 and 3, acknowledged in 0xfa8 bits 8 and 9), release. */
	field(0x8000001, 1); field(0x8000301, 1);
	field(0x80c0201, 1); field(0x80c0301, 1);
	int result = wait_bits(BB + 0xfa8, 0x300, 0x300, 200);
	field(0x80c0201, 0); field(0x80c0301, 0);
	field(0x8000301, 0); field(0x8000001, 0);
	ap_info.decoder_resets++; ap_info.decoder_reset_result = (uint32_t)result;
	return result;
}
static int ap_rearm_rx(void)
{
	field(0x8000101, 1); field(0x80c0201, 1);
	int result = wait_bits(BB + 0xfa8, 0x100, 0x100, 200);
	field(0x80c0201, 0); field(0x8000101, 0);
	return result;
}
static int ap_rearm_tx(void)
{
	field(0x8000201, 1); field(0x80c0301, 1);
	int result = wait_bits(BB + 0xfa8, 0x200, 0x200, 200);
	field(0x80c0301, 0); field(0x8000201, 0);
	return result;
}
int ar_rf_ap_rearm(unsigned variant)
{
	if (!ap_role) { return -95; }
	if (variant == 1) { return ap_rearm_rx(); }
	if (variant == 2) { return ap_rearm_tx(); }
	if (variant == 3) { ap_rearm_pending = 3; return 0; }
	return -22;
}
/* The broadcast (user 8) TX bandwidth: code 2 = 5 MHz (a new link), 4 = 20,
 * 5 = 40. The MCS fields depend on it, so the current MCS is programmed again.
 * 0xa48 is what stock writes before each TX entry (IRQ 7) for the code. 40 MHz
 * also shortens the broadcast window [15:0] from 3000 to 1600 us: the frame
 * changes, so the running sequence is re-armed and the frame time taken again,
 * as for a frame-mode change (stock tears the link down here instead). */
static void ap_apply_bandwidth(unsigned code)
{
	unsigned previous = ap_bw_code;
	field(0xb200003, code);
	write32(BB + 0xa48, code == 5 ? 0x80808080 : code == 4 ? 0x40404040 : 0);
	field(0x9700010, code == 5 ? 0x09c4 : 0x1250);
	{
#ifdef AR_BW_TRACE
		uint64_t hold_from = timer_ticks();
#endif
		int held = hold_engines();
#ifdef AR_BW_TRACE
		uint64_t held_at = timer_ticks();
#endif
		release_engines();
#ifdef AR_BW_TRACE
		ap_trace_item(0, held_at - fstamp.irq5[0]); ap_trace_item(1, held_at - hold_from);
		for (unsigned i = 0; i < 4; i++) {
			unsigned k = (ap_trace_pre_head + i) & 3;   /* oldest first */
			uint64_t at = ap_trace_pre_at[k] && fstamp.irq5[0] > ap_trace_pre_at[k] ? fstamp.irq5[0] - ap_trace_pre_at[k] : 0;
			uint32_t v = (uint32_t)(at / 320); if (v > 0x7fff) { v = 0x7fff; }
			ap_trace_item(21 + i, (uint64_t)(v | (ap_trace_pre_stream[k] ? 0x8000u : 0)) * 320);
		}
		ap_trace_item(2, ap_trace_irq17 && fstamp.irq5[0] > ap_trace_irq17 ? fstamp.irq5[0] - ap_trace_irq17 : 0);
		for (unsigned i = 0; i < 4; i++) { ap_trace_raw(25 + i, ap_trace_pkt[(ap_trace_pre_head + i) & 3]); }
		ap_trace_pkt_after = 4;
		ap_trace_switch = fstamp.irq5[0]; ap_trace_after = 1; ap_trace_pre = 0; ap_trace_irq17 = 0; ap_trace_17 = 1;
		ap_trace_i7_frames = 8; ap_trace_i7_user8 = ap_trace_i7_all = ap_trace_i7_seen = ap_trace_i7_preset = 0;
		ap_tf_since = 0;
#endif
		ap_frame_us = dev_frame_microseconds(); phy_control.frame_us = ap_frame_us;
		ap_info.frame_us = ap_frame_us;
		ap_trace_mark(40 + code, read32(BB + 0x970), read32(BB + 0xb20), (uint32_t)held);
		/* Diagnostics: with the stall freeze off (unfreeze 1), keep the frames
		 * around the first switch across 40 MHz in the trace. */
		if (!ap_trace.frozen && ap_trace.freeze_at == UINT32_MAX) { ap_trace.freeze_at = ap_trace.head + 300; }
	}
	(void)ar_tx_mcs_control(&tx_interleaver, AR_TX_MCS_PROGRAM, 8, tx_interleaver.users[8].mcs,
		mcs_fch_keys, information.chip_profile, 0, &tx_mcs_ops, NULL);
	ap_bw_code = code; ap_info.bandwidth = code; ap_info.bandwidth_switches++; ap_bw_prequiet = 0;
	/* The re-arm starts the engine's block ids over: the HARQ watchdog would
	 * read them as stuck and turn retransmission off (stock re-seeds its
	 * tracker after every sequencer restart). One that had just turned it
	 * off (0xb20 bit 30, two frames) is turned back on here: re-seeded, it
	 * never would, and the broadcast stayed paused - the ground heard the
	 * first frame on 40 MHz and nothing after, the link lost (bench,
	 * 2026-10-26, 1 switch in ~45, the AP's TX power reading 0). */
#ifdef AR_BW_TRACE
	ap_trace_raw(33, (uint16_t)(ap_retx.phase | (read32(BB + 0xb20) >> 30 & 1) << 8));   /* item 33: watchdog phase, bit 30 */
#endif
	if (ap_retx.phase && ap_link.state == AR_AP_CONNECTED) { field(0xb201e01, 0); ap_info.retx_enables++; }
	ar_dev_retx_init(&ap_retx, ap_retx.threshold);
	/* Stock event 12 [slot, received, new, old]: our own change (the air host
	 * takes its throughput again on it). */
	const uint8_t data[] = {0, 0, (uint8_t)code, (uint8_t)previous};
	publish_event(12, data, sizeof data);
}
static void ap_set_mcs(unsigned mcs)
{
	unsigned previous = tx_interleaver.users[8].mcs;
	if (mcs >= 25 || mcs == previous) { return; }
	(void)ar_tx_mcs_control(&tx_interleaver, AR_TX_MCS_PROGRAM, 8, mcs, mcs_fch_keys,
		information.chip_profile, 0, &tx_mcs_ops, NULL);
	ap_info.broadcast_mcs = tx_interleaver.users[8].mcs;
	mcs_event(0, tx_interleaver.users[8].mcs, previous);   /* our TX MCS changed */
}
/* TDD frame mode on the AP (stock bb_link_do_frame_change, AP role): user
 * 0's receive profile and its slot follow the ground's shorter uplink (stock
 * AP dumps: 0x9a0 0x3cb0a12b standard / 0x3cb0012a short, 0xa10 [15:0]
 * 0x0da3 / 0x07a3, i.e. 2234 / 1251 us), then both engines are held and
 * flushed so the running sequence takes the new slot. The frame goes from
 * 5633.92 to 4650.88 us; the broadcast window stays. */
static void ap_apply_frame_mode(unsigned mode)
{
	ap_frame_pending = 0;
	if ((mode != 0 && mode != 2) || mode == ap_frame_mode) { return; }
	ap_trace_mark(30 + mode, read32(BB + 0xa10), read32(BB + 0x9a0), read32(BB + 0xb20));
	write32(BB + 0x9a0, mode == 2 ? 0x3cb0012a : 0x3cb0a12b);
	field(0xa100010, mode == 2 ? 0x07a3 : 0x0da3);
	int held = hold_engines(); release_engines();
	ap_trace_mark(33, read32(BB + 0xa10), read32(BB + 0x9a0), (uint32_t)held);
	ap_frame_mode = mode;
	ap_frame_us = dev_frame_microseconds(); phy_control.frame_us = ap_frame_us;
	ap_rx_end_us = (mode == 2 ? 1251 : 2234) + 66;
	ap_info.frame_mode = mode; ap_info.frame_switches++; ap_info.frame_us = ap_frame_us;
}
/* A power request from the ground (command 8 sub-type 6), inside the
 * host's range; none without one. */
static void ap_power_request(uint32_t word)
{
	uint8_t target, direction, slot;
	if (ar_pwr_auto_parse(word, &target, &direction, &slot) || slot) { return; }
	uint8_t current = power_level[8] != 0xff ? power_level[8] : ap_pwr_max;
	uint8_t next = ar_pwr_auto_apply(current, target, direction, ap_pwr_min, ap_pwr_max);
	/* Nothing to change (at an end of our range): say where we are, so the
	 * ground steps from there and not from what it asked for. */
	if (next) { (void)apply_power(8, next); } else { ap_pwr_announce = 1; }
}
/* A PHY message from the ground's slot header. Command 3 asks for the
 * broadcast MCS ([slot, mcs | 0xc0], TTL 31); the ground stops asking once it
 * sees that MCS in our headers. */
static void ap_phy_received(const uint8_t control[7])
{
	unsigned command = control[1] & 15, length = (control[2] >> 1) & 7;
	ap_info.uplink_phy++;
	if (command == 3 && length >= 2 && !control[3]) {
		ap_info.mcs_requests++;
		ap_set_mcs(control[4] & 0x3f);
	}
	/* Command 8 [mode << 4, 0, 0, 0], TTL 5: the ground switches its frame
	 * as the TTL runs out; the AP as it receives the last one (TTL 1), as a
	 * stock receiver dispatches, or by a timer if that one is lost. */
	/* Command 4 [flags | slot 0, index lo, index hi], TTL 16: the ground
	 * hops as its TTL runs out (at its next uplink header); the AP at the
	 * IRQ 5 after the last one, or by a timer if that one is lost. */
	if (command == 4 && length == 3 && (control[3] & 0xf0) && !(control[3] & 15)) {
		unsigned index = control[4] | (unsigned)control[5] << 8;
		unsigned life = control[1] >> 4 | (control[2] & 1) << 4;
		ap_info.channel_requests++;
		if (life <= 1) { ap_chan_at_sweep = 0x10000 | index; ap_chan_pending = 0; }
		else if (!ap_chan_pending) {
			ap_chan_pending = 0x10000 | index;
			ap_chan_deadline_ms = elapsed_ms() + (life - 1) * ap_frame_us / 1000 + 4;
		}
	}
	/* Command 8 sub-type 6: the ground asks for our TX power (ar_pwr_auto). */
	if (command == 8 && length == 4 && (control[3] & 15) == AR_PWR_AUTO_SUBTYPE) {
		uint32_t word = control[3] | (uint32_t)control[4] << 8 | (uint32_t)control[5] << 16 | (uint32_t)control[6] << 24;
		unsigned life = control[1] >> 4 | (control[2] & 1) << 4;
		if (life <= 1) { ap_pwr_pending = 0; ap_power_request(word); }
		else if (!ap_pwr_pending) {
			ap_pwr_pending = word;
			ap_pwr_deadline_ms = elapsed_ms() + (life - 1) * ap_frame_us / 1000 + 4;
		}
	}
	if (command == 8 && length == 4 && !(control[6] & 12) && !(control[3] & 15)) {
		unsigned mode = control[3] >> 4 | (control[4] & 15) << 4;
		unsigned life = control[1] >> 4 | (control[2] & 1) << 4;
		ap_info.frame_requests++;
		ap_trace_mark(34, life, mode, read32(BB + 0xa10));
		if (life <= 1) { ap_apply_frame_mode(mode); }
		else if (!ap_frame_pending) {
			ap_frame_pending = 0x80 | (mode & 7);
			ap_frame_deadline_ms = elapsed_ms() + (life - 1) * ap_frame_us / 1000 + 4;
		}
	}
}
static void ap_command_done(unsigned command, const uint8_t *payload)
{
	(void)payload;
	ap_info.phy_expired++;
	/* Command 6 reached the ground at this TTL: transmit with our hash. */
	if (command == 6 && ap_link.state == AR_AP_CONNECTED) {
		ap_set_identity(ap_identity_hash);
		/* The uplink MCS: the policy from here on (its requests go from the
		 * frame tick), or the range's top once, from ap_poll (this runs
		 * inside the PHY tick, which clears the block after it). */
		if (ap_up_active()) { ap_up_connect(1); } else { ap_uplink_mcs_pending = 1; }
	}
	/* Our command 4 expired: the ground hopped on its last TTL, we follow at IRQ 5. */
	if (command == 4 && ap_chan_hop) { ap_chan_at_sweep = ap_chan_hop; ap_chan_hop = 0; }
	/* Command 5 expired on both ends: the ground's receiver moves now. A
	 * switch across 40 MHz has moved already, at the IRQ 5 after the TTL-1
	 * header; if that was missed, at the next one. */
	if (command == 5 && ap_bw_pending) {
		if (ap_link.state == AR_AP_CONNECTED && ap_bw_pending != ap_bw_code) {
			ap_bw_at_sweep = 0x80 | ap_bw_pending;
		} else if (!ap_bw_at_sweep) { ap_bw_prequiet = 0; }   /* no switch to come */
		ap_bw_pending = 0;
	}
}
const struct ar_rf_ap_trace *ar_rf_ap_trace_information(void) { return &ap_trace; }
void ar_rf_ap_trace_unfreeze(unsigned mode)
{
	/* UINT32_MAX: no freeze at all; UINT32_MAX - 1: none at a count (never
	 * reached), only the stall check in ap_poll. */
	ap_trace.freeze_at = mode == 2 ? UINT32_MAX - 1 : UINT32_MAX; ap_trace.frozen = 0;
}
/* Bench: the broadcast's pre-encode (the one that fills the next frame's
 * first block) moved in the 0x8a0 frame program - positional, by hardware
 * entry (0 the switch before the broadcast, 1 the broadcast, 2-4 the
 * switches and the sweep after it, 5 the slot), byte 0x80 | at << 4 | raw
 * user 8 (stock: 0xe8 at entry 1). Placed at the next IRQ 5 with the engines
 * re-armed, as stock re-places it on a frame change; trace item 50 holds the
 * table before and after. */
static unsigned ap_trigger_pending;   /* 0x10000 | position << 8 | byte */
void ar_rf_ap_trigger(unsigned position, unsigned byte)
{
	/* position bit 7: no re-arm (the table taken as it runs). */
	if ((position & 0x7f) < 6 && (byte & 0x80)) { ap_trigger_pending = 0x10000 | (position & 0x7f) << 8 | (position & 0x80) << 8 | (byte & 0xff); }
}
static void ap_place_trigger(void)
{
	unsigned position = (ap_trigger_pending >> 8) & 7, byte = ap_trigger_pending & 0xff;
	unsigned rearm = !(ap_trigger_pending & 0x8000);
	ap_trigger_pending = 0;
	ap_trace_mark(50, read32(BB + 0x8a0), read32(BB + 0x8a4), position << 8 | byte);
	for (unsigned offset = 0x8a0; offset < 0x900; offset += 4) { write32(BB + offset, 0); }
	update(BB + 0x8a0 + position / 4 * 4, 0xffu << (position % 4 * 8), (uint32_t)byte << (position % 4 * 8));
	if (rearm) { (void)hold_engines(); release_engines(); }
	ap_trace_mark(50, read32(BB + 0x8a0), read32(BB + 0x8a4), position << 8 | byte);
	if (!ap_trace.frozen) { ap_trace.freeze_at = ap_trace.head + 200; }   /* keep the frames after it */
}
static void ap_trace_irq(unsigned index)
{
	uint32_t now = elapsed_us();
	if (index == 19) { ap_trace.last_pre_us = now; }
	if (ap_trace.frozen) { return; }
	if (ap_trace.freeze_at && ap_trace.head == ap_trace.freeze_at) {
		ap_trace.frozen = 2; ap_trace.frozen_us = now; return;
	}
	uint32_t *e = ap_trace.entries[ap_trace.head % AR_RF_AP_TRACE_ENTRIES];
	/* pre-encode: its request word; frame control: the packet IDs asked for */
	e[5] = index == 19 ? read32(BB + 0xf58) : index == 18 ? read32(BB + 0xf64) : 0;
	e[0] = now; e[1] = index | (read32(BB + 0xfa8) & 0xffffff) << 8;
	e[2] = read32(BB + 0xf70); e[3] = read32(BB + (index == 19 ? 0xf5c : 0xf88)); e[4] = read32(BB + 0xf90);   /* 19: its budget word */
	ap_trace.head++;
}
/* A trace entry of our own (source 30 and up): three words of state. */
static void ap_trace_mark(unsigned source, uint32_t a, uint32_t b, uint32_t c)
{
	if (ap_trace.frozen) { return; }
	uint32_t *e = ap_trace.entries[ap_trace.head % AR_RF_AP_TRACE_ENTRIES];
	e[0] = elapsed_us(); e[1] = source | (read32(BB + 0xfa8) & 0xffffff) << 8;
	e[2] = a; e[3] = b; e[4] = c; e[5] = read32(BB + 0xf70);
	ap_trace.head++;
}
static void ap_poll(void)
{
	if (!ap_trace.frozen && ap_trace.freeze_at != UINT32_MAX && ap_info.preencodes > 200 &&
	    elapsed_us() - ap_trace.last_pre_us > 20000) {
		ap_trace.frozen = 1; ap_trace.frozen_us = elapsed_us();
	}
	unsigned previous = ap_link.state;
	if (ar_ap_poll(&ap_link, elapsed_ms())) { ap_link_changed(previous); }
	/* Command 6 the login could not send (block busy): now that it is free. */
	if (ap_identity_pending && ap_link.state == AR_AP_CONNECTED) {
		const uint8_t identity[2] = {0x88, ap_identity_hash};
		if (!ar_phy_control_enqueue(&phy_control, 6, 5, identity, 2, &phy_ops, NULL)) {
			ap_identity_pending = 0; ap_info.phy_sent++;
		}
	}
	/* Our TX power to the ground (at link-up and each change), when the
	 * block is free. */
	if (ap_pwr_announce && ap_link.state == AR_AP_CONNECTED && power_level[8] != 0xff) {
		uint32_t word = ar_pwr_auto_announce_word(power_level[8]);
		const uint8_t bytes[4] = {(uint8_t)word, (uint8_t)(word >> 8), 0, 0};
		if (!ar_phy_control_enqueue(&phy_control, 8, 5, bytes, 4, &phy_ops, NULL)) { ap_pwr_announce = 0; ap_info.phy_sent++; }
	}
	/* The host's channel its SET 6 could not send (block busy): the same. */
	if (ap_chan_host_pending && ap_link.state == AR_AP_CONNECTED && !ap_chan_hop &&
	    !ap_hop_start(ap_chan_host_pending & 0xffff, 0)) {
		ap_chan_host_pending = 0;
	}
	if (ap_frame_pending && (int32_t)(elapsed_ms() - ap_frame_deadline_ms) >= 0) {
		ap_frame_at_sweep = ap_frame_pending; ap_frame_pending = 0;
	}
	if (ap_pwr_pending && (int32_t)(elapsed_ms() - ap_pwr_deadline_ms) >= 0) {
		uint32_t word = ap_pwr_pending; ap_pwr_pending = 0; ap_power_request(word);
	}
	if (ap_chan_pending && (int32_t)(elapsed_ms() - ap_chan_deadline_ms) >= 0) {
		ap_chan_at_sweep = ap_chan_pending; ap_chan_pending = 0;
	}
	ap_auto_channel();
	/* 1: to send (the MCS whose key is 0, internal index 2), 2: out, held. */
	if (ap_uplink_mcs_pending == 1 && ap_link.state == AR_AP_CONNECTED) {
		unsigned mcs = 0;
		while (mcs < 16 && mcs_fch_keys[mcs]) { mcs++; }
		if (ap_uplink_mcs_max < 16) { mcs = ap_uplink_mcs_max; }
		const uint8_t request[2] = {0, (uint8_t)(0xc0 | mcs)};
		for (unsigned i = 0; i < 4; i++) { ap_info.mcs_keys[i] = mcs_fch_keys[i]; }
		if (mcs >= 16) { ap_uplink_mcs_pending = 0; }
		else if (!ar_phy_control_enqueue(&phy_control, 3, 31, request, 2, &phy_ops, NULL)) {
			ap_uplink_mcs_pending = 2; ap_uplink_mcs_ms = elapsed_ms();
			ap_info.uplink_mcs = mcs; ap_info.phy_sent++;
		}
	}
	if (ap_uplink_mcs_pending == 2 && elapsed_ms() - ap_uplink_mcs_ms >= AP_UPLINK_MCS_MS) {
		(void)ar_phy_control_cancel(&phy_control, &phy_ops, NULL);
		ap_uplink_mcs_pending = 0;
	}
	/* The ground's port-0 command 9 asks for a broadcast bandwidth: stock
	 * answers with command 5 [type 0, slot 8 | code << 5, 0], TTL 3, and both
	 * ends switch as it expires (the ground reads the code from bits 7:5; a
	 * fixed 0x88 there read as 20 MHz whatever was asked). */
	/* Not while a switch still waits for its IRQ 5 (put off over a receive):
	 * the same code again, its quiet armed with nothing to switch, kept the
	 * broadcast without video. */
	if (ap_link.state == AR_AP_CONNECTED && ap_link.bandwidth_request && !ap_bw_pending && !ap_bw_at_sweep) {
		unsigned code = ap_link.bandwidth_request - 1u;
		if (code >= 2 && code <= 5 && code != ap_bw_code) {
			const uint8_t request[3] = {0, (uint8_t)(8 | code << 5), 0};
			if (!ar_phy_control_enqueue(&phy_control, 5, 3, request, 3, &phy_ops, NULL)) {
				ap_bw_pending = code; ap_info.phy_sent++;
			}
		}
	}
}

int ar_rf_host_mcs_ceiling(const uint8_t *payload, size_t length)
{
	if (!payload || length != 1) { return -22; }
	if (payload[0] == 0xff) { host_ceiling = 0; apply_ceiling(); return 0; }
	const int mcs = (int8_t)payload[0];
	/* Stock -1..13 (internal 1..15). -2 would be internal 0, which reads as
	 * no ceiling at all: refused rather than lifting it. */
	if (mcs < -1 || mcs > 13) { return -22; }
	host_ceiling = (unsigned)(mcs + 2); apply_ceiling();   /* internal = stock + 2 */
	return 0;
}

/* Duration of this monitor's six-step DEV frame only. The stock frame reader
 * (0x21acda) rounds each 640 ns duration separately before summing (0x21b008).
 * Prepare commands use 0x984; RX8, special user13 and TX0 use the other fields.
 * This does not implement the general stock frame interpreter. */
static uint32_t dev_frame_microseconds(void)
{
	uint32_t prepare = ((read32(BB + 0x984) & 0xffff) * 640 + 500) / 1000;
	/* The broadcast window: [31:16] receives (DEV), [15:0] transmits (AP). */
	uint32_t window = read32(BB + 0x970), broadcast = (((ap_role ? window : window >> 16) & 0xffff) * 640 + 500) / 1000;
	uint32_t special = ((read32(BB + 0x980) & 0xffff) * 640 + 500) / 1000;
	/* The ground's uplink slot: 0xa00 on a DEV, 0xa10 on an AP. */
	uint32_t transmit = ((read32(BB + (ap_role ? 0xa10 : 0xa00)) & 0xffff) * 640 + 500) / 1000;
	return prepare * 3 + broadcast + special + transmit;
}

const struct ar_rf_frame_info *ar_rf_frame_information(void)
{
	struct ar_rf_frame_info *info = &frame_stamp_info;
	uint64_t us = 0;
	const int known = ar_frame_stamp_ap_us(&fstamp, ap_role, &us) == 0;
	info->version = 1;
	info->flags = (ap_role ? 1u : 0u) | (known ? 2u : 0u) | (!ap_role && fstamp.valid ? 4u : 0u);
	info->frames = fstamp.frames;
	/* The frame in 0.64 us ticks, its parts as dev_frame_microseconds adds them. */
	const uint32_t window = read32(BB + 0x970);
	info->frame_ticks = (read32(BB + 0x984) & 0xffff) * 3 + ((ap_role ? window : window >> 16) & 0xffff) +
		(read32(BB + 0x980) & 0xffff) + (read32(BB + (ap_role ? 0xa10 : 0xa00)) & 0xffff);
	info->since_us = fstamp.irq5[0] ? (uint32_t)((timer_ticks() - fstamp.irq5[0]) / 40) : 0;
	info->ap_us_lo = (uint32_t)us; info->ap_us_hi = 0;   /* modulo 2^32 */
	info->taken = fstamp.taken; info->rejected = fstamp.rejected; info->residual_us = fstamp.residual_us;
	info->width = ap_role ? ap_bw_code : bw_code;
	info->frame_mode = ap_role ? ap_frame_mode : frame_mode;
	info->sent = phy_control.stamps;
	info->retries = ap_role ? ap_info.hop_retx_frames : retry_blocks;
	info->retry_frame = retry_have ? retry_newest : 0;
	info->retry_known_frames = retry_known_frames; info->retry_stalls = retry_stalls;
	memset(info->retry_map, 0, sizeof info->retry_map); memset(info->retry_known, 0, sizeof info->retry_known);
	info->q_frame = fq_have ? fq_newest : 0;
	for (unsigned k = 0; k < 64; k++) { info->q_word[k] = fq_have ? fq_word[(fq_newest - k) & 255] : 0; }
	for (unsigned k = 0; retry_have && k < 64; k++) {
		const uint8_t v = retry_ring[(retry_newest - k) & 255];
		if (v == 0xff) { continue; }
		info->retry_known[k / 32] |= 1u << (k % 32);
		info->retry_map[k / 16] |= (uint32_t)v << (2 * (k % 16));
	}
	return info;
}

uint8_t ar_rf_control_checksum(const uint8_t control[7])
{
	return ar_phy_control_checksum(control);
}

uint32_t ar_rf_host_ap_time(void)
{
	/* The AP's clock is its own milliseconds, what its command 7 carries. */
	if (ap_role) { return (uint32_t)(timer_ticks() / 40000); }
	const struct ar_ap_time time = {phy_control.time_value,phy_control.time_valid};
	return ar_ap_time_get(&time, (uint32_t)(timer_ticks() / 40000));
}

int ar_rf_host_status(uint16_t users, uint8_t output[300])
{
	if (!output) { return -22; }
	if (receive_operation == 76 && !host_initialized) { return -11; }
	if (receive_operation < 67 || information.phase < AR_RF_PSRAM || information.error ||
	    !receiver.channel_count || dev_config.mode || dev_config.slot_bitmap != 1) { return -11; }
	/* GET_STATUS 0x245de8: DEV role, mode0, no external sync, slot0.
	 * Report the programmed frequency separately for each direction. */
	memset(output, 0, 300);
	output[0] = 1; output[3] = 1; output[4] = output[5] = 1;
	memcpy(output + 6, dev_config.local, 4);
	for (unsigned user = 0; user < 10; user++) {
		if (!(users & (1u << user))) { continue; }
		uint8_t *tx = output + 12 + user * 24, *rx = tx + 12;
		uint32_t tw = read32(BB + 0xb00 + user * 4);
		uint32_t rw = read32(BB + 0x9a0 + user * 8);
		uint32_t key = ((tw >> 8) & 7) << 16 | (((tw >> 12) & 15) % 3) << 8 |
			((tw >> 16) & 3) | (uint32_t)(((tw >> 22) & 3) == 2) << 24;
		tx[0] = 25;
		for (unsigned mcs = 0; mcs < 25; mcs++) {
			if (mcs_fch_keys[mcs] == key) { tx[0] = mcs; break; }
		}
		tx[1] = (tw >> 22) & 3; tx[2] = (tw >> 5) & 1;
		tx[3] = (tw >> 4) & 1; tx[4] = (tw >> 18) & 7; tx[5] = tw & 7;
		/* The original per-user RX getter deliberately returns MCS25;
		 * negotiated RX MCS is in the separate link entry below. */
		rx[0] = 25; rx[1] = (rw >> 6) & 3; rx[5] = (rw >> 8) & 7;
		unsigned selector = rw & 7, interleave = (rw >> 13) & 7;
		unsigned base = interleave & ~4u;
		if (selector == 1 || selector == 2) {
			rx[4] = selector - 1; rx[2] = interleave != 0;
		} else if (selector >= 3 && selector <= 6) {
			/* Inverse selector/interleaver mapping, 0x24d058. */
			if (base < 1 || base > 3 || selector < base + 2 || selector > base + 3 ||
			    (selector == 3 && base != 1) || (selector == 6 && base != 3)) { return -95; }
			rx[4] = base + 1; rx[3] = selector == base + 3; rx[2] = interleave != base;
		} else if (selector != 0) { return -95; }
		for (unsigned b = 0; b < 4; b++) {
			tx[8+b] = tuned_frequency[user][0] >> (8*b);
			rx[8+b] = tuned_frequency[user][1] >> (8*b);
		}
	}
	/* Deterministic unavailable entries replace original unused stack bytes.
	 * 0x2327f4 supplies DEV's configured peer, not the observed AP address. */
	for (unsigned slot = 0; slot < 8; slot++) {
		output[252 + slot * 6] = 3; output[253 + slot * 6] = 25;
	}
	output[252] = information.phase == AR_RF_RECEIVING ? receiver.dev_state : 0;
	if (information.phase == AR_RF_RECEIVING && dev_active && receiver.dev_state == 2 &&
	    dev_mcs.current < dev_mcs.config.count) {
		output[253] = dev_mcs.config.rows[dev_mcs.current].mcs;
	}
	memcpy(output + 254, dev_config.configured_peer, 4);
	/* Open8030: linked, the AP it took (its address from its broadcast), so
	 * the host can tell air units apart; stock reports the configured peer. */
	if (output[252] == 2) { memcpy(output + 254, receiver.accepted_peer, 4); }
	if (ap_role) {
		/* The AP's view: role 0, the link state of its slot-0 ground. */
		output[0] = 0;
		output[252] = information.phase == AR_RF_RECEIVING ? ap_link.state : 0;
		output[253] = tx_interleaver.users[8].mcs;
		memcpy(output + 254, ap_link.ground, 4);
	}
	return 0;
}
int ar_rf_ap_role(void) { return (int)ap_role; }

int ar_rf_monitor(uint32_t frequency, uint32_t milliseconds, uint8_t identity_hash, unsigned operation)
{
	return ar_rf_monitor_plan(frequency, milliseconds, identity_hash, operation, NULL, 0);
}

int ar_rf_monitor_plan(uint32_t frequency, uint32_t milliseconds, uint8_t identity_hash,
	unsigned operation, const uint8_t *channels, size_t length)
{
	/* Operations 77/78 are bounded/continuous fixed-channel listeners. Keep the
	 * underlying operation below the admission/PHY-command/TX thresholds;
	 * merely inventing a connected DEV state would enable acknowledgements.
	 * LST1: bandwidth, selector, interleave, buffer-mode; RX duration counts.
	 * No automatic session tracking; only receive-only video delivery. */
	if (operation == 77 || operation == 78) {
		if (!channels || length != 12 || (memcmp(channels, "LST1", 4) && memcmp(channels, "LST2", 4)) ||
		    channels[4] > 5 || channels[5] < 4 || channels[5] > 5 ||
		    channels[6] < 5 || channels[6] > 6 || channels[7] > 1 ||
		    !le32(channels + 8) || le32(channels + 8) > 65535 ||
		    (operation == 77 ? !milliseconds || milliseconds > 10000 : milliseconds != 0)) { return -22; }
        unsigned follow=!memcmp(channels,"LST2",4), initial=0;
        if (follow) {
            if (!host_config.valid || host_config.length<AR_RF_DEV4_PREFIX+4 ||
                memcmp(host_config.channels,"DEV4",4) || channels[4]<2) { return -95; }
            unsigned count=(host_config.length-AR_RF_DEV4_PREFIX)/4;
            while (initial<count && le32(host_config.channels+AR_RF_DEV4_PREFIX+initial*4)!=frequency) { initial++; }
            if (initial==count) { return -22; }
        }
		/* This diagnostic is DEV-only; do not reuse an AP's saved context. */
		if (ap_role) { return -95; }
		uint8_t channel[4];
		for (unsigned i = 0; i < 4; i++) { channel[i] = frequency >> (8 * i); }
		int result = configure_monitor(frequency, milliseconds, identity_hash,
			53, channel, sizeof channel, operation == 78 ? 2 : 1, NULL);
		if (result) { return result; }
		host_prepared = 0; listener_capture = 1;
        listener_follow=follow;
        memset(&listener_change,0,sizeof listener_change);
        if (follow) {
            receiver.channel_count=(host_config.length-AR_RF_DEV4_PREFIX)/4;
            for (unsigned i=0;i<receiver.channel_count;i++) {
                receiver.channels[i].frequency_khz=le32(host_config.channels+AR_RF_DEV4_PREFIX+i*4);
                receiver.channels[i].visits=0;
            }
            receiver.channel_index=initial; receiver.channels[initial].visits=1;
        }
        bw_code=channels[4];
		field(0xde00008, identity_hash); field(0xdf00008, identity_hash);
		field(0x9e00803, channels[4]); field(0x9e00003, channels[5]);
		field(0x9e00d03, channels[6]); field(0x9701010, le32(channels + 8));
		field(0xbf00002, channels[7]); field(0xd081001, 0);
        receiver.search_frame_us=dev_frame_microseconds();
		/* Search retuning clears the buffer map. Observe this fixed context
		 * for the entire bounded trial, including when every payload fails. */
		receiver.search_suspended = 1;
		field(0x8000201, 1);
		return start_prepared_receive();
	}
	return configure_monitor(frequency, milliseconds, identity_hash, operation, channels, length, 0, NULL);
}

static int prepare_host(uint32_t frequency, uint32_t milliseconds, uint8_t identity_hash,
	const uint8_t *channels, size_t length, unsigned deferred, const struct ar_power_config *power)
{
	int result = configure_monitor(frequency, milliseconds, identity_hash, 76, channels, length, deferred, power);
	if (result) { return result; }
	memset(&binding, 0, sizeof binding);
	memmove(host_config.channels, channels, length);
	host_config.length = length; host_config.frequency = frequency;
	host_config.milliseconds = milliseconds; host_config.identity = identity_hash;
	host_config.deferred = deferred; host_config.valid = host_initialized = 1;
	host_config.has_power = power != NULL;
	if (power) { host_config.power = *power; }
	host_start_required = 0;
	return 0;
}
int ar_rf_prepare_host(uint32_t frequency, uint32_t milliseconds, uint8_t identity_hash,
	const uint8_t *channels, size_t length)
{
	return prepare_host(frequency, milliseconds, identity_hash, channels, length, 1, NULL);
}

/* Firmware-owned continuous mode is not exposed by the bounded diagnostic
 * monitor command. The backend still supplies the actual start candidates. */
int ar_rf_prepare_host_continuous(uint32_t frequency, uint8_t identity_hash,
	const uint8_t *channels, size_t length)
{
	return prepare_host(frequency, 0, identity_hash, channels, length, 2, NULL);
}

int ar_rf_prepare_config(uint32_t frequency, uint8_t identity_hash,
	const uint8_t *channels, size_t length, unsigned search, const struct ar_power_config *power)
{
	if (search > 1 || !power) { return -22; }
	return prepare_host(frequency, 0, identity_hash, channels, length, search ? 3 : 2, power);
}

static void binding_identity(void)
{
	uint8_t hash = 0;
	if (!binding.enabled) {
		for (unsigned i = 0; i < 4; i++) { hash ^= dev_config.configured_peer[i]; }
	}
	/* Original DEV pairing setter: 0x22b92c(8, hash), then
	 * 0x222cac(8, hash). Pairing uses the shared zero identity. */
	field(0xb841008, hash); field(0xb601008, hash);
	field(0xde00008, hash); field(0xdf00008, hash);
}

/* AP binding, the air unit's bind button: the air host (as the stock air app)
 * sends SET 2 on, reads GET 1 every 20 ms until the same ground has been
 * there a few times, sends SET 2 off, then SET 4 with that ground as the
 * candidate. Meanwhile the idle broadcast offers slot 0 for pairing and any
 * ground is served; the ground that completes its login is the result. A
 * ground bound this way stays served when pairing ends, so the link it formed
 * is not dropped between SET 2 off and SET 4. */
static int ap_host_pairing(const uint8_t *data, size_t length)
{
	uint8_t bound[4];
	const unsigned was_bound = binding.enabled && binding.complete;
	if (was_bound) { memcpy(bound, binding.peer, 4); }
	int result = ar_binding_set(&binding, data, length);
	if (result) { return result; }
	struct ar_ap_config config = ap_link.config;
	config.pairing = (uint8_t)binding.enabled;
	if (!binding.enabled && was_bound) {
		unsigned known = 0;
		for (unsigned i = 0; i < config.ground_count; i++) { known |= !memcmp(config.grounds[i], bound, 4); }
		if (!known && config.ground_count < AR_AP_MAX_GROUNDS) {
			memcpy(config.grounds[config.ground_count++], bound, 4);
		}
	}
	unsigned previous = ap_link.state;
	ar_ap_configure(&ap_link, &config);
	ap_link_changed(previous);
	return 0;
}

int ar_rf_host_pairing(const uint8_t *data, size_t length)
{
	if (!data || length != 14) { return -22; }
	if (receive_operation != 76 || !host_initialized || information.error) { return -11; }
	if (ap_role) { return ap_host_pairing(data, length); }
	unsigned was_pairing = binding.enabled;
	/* A completed bind ends with the bound AP as the peer, as on stock:
	 * set before the identity below is taken from the peer - left at the
	 * previous unit, the identity no longer matched the AP just bound and
	 * the link it had formed dropped. */
	uint8_t bound[4];
	const unsigned bound_ok = binding.enabled && binding.complete;
	if (bound_ok) { memcpy(bound, binding.peer, 4); }
	int result = ar_binding_set(&binding, data, length);
	if (result) { return result; }
	if (!binding.enabled && bound_ok) {
		memcpy(dev_config.configured_peer, bound, 4);
		memcpy(host_config.channels + 8, bound, 4);
	}
	dev_config.pairing = binding.enabled;
	memcpy(dev_config.pairing_skip, binding.skip, sizeof binding.skip);
	if (information.phase == AR_RF_RECEIVING &&
	    (binding.enabled || (was_pairing && receiver.dev_state != 2))) {
		/* Reacquire with pairing admission. Defer teardown until borrowed
		 * RX views/DMA are released, as for normal peer replacement. An
		 * unfinished pairing must restart normal search on cancellation:
		 * restoring the configured-peer filter alone can strand acquisition
		 * when the previously active peer came from the host candidate list. */
		request_recovery(4);
	} else if (binding.enabled || was_pairing) { binding_identity(); }
	return 0;
}

int ar_rf_host_pairing_result(uint8_t output[164])
{
	if (receive_operation != 76 || !host_initialized || information.error) { return -11; }
	return ar_binding_result(&binding, output);
}


/* DEV binding (ported from upstream mh1412/Open8030 718c31d): SET 2 starts
 * or cancels pairing, GET 1 returns the completed peer, SET 3 selects the
 * peer to admit. Pairing uses the shared zero identity; a cancelled,
 * unfinished pairing restarts normal search through recovery. */
static unsigned peer_ignored;
static uint8_t peer_requested[4];
int ar_rf_host_peer(const uint8_t *data, size_t length)
{
	if (!data || length != 4) { return -22; }
	if (receive_operation != 76 || !host_initialized || information.error) { return -11; }
	/* Stock (DEV) stores the peer - the AP to associate with, admitted as
	 * the configured peer whatever the candidate list holds - and
	 * retargets the user-8 identity through its own hash (0x2218f4). The
	 * peer is stored the same way here. The identity is not retargeted:
	 * that hash is not derived yet, an XOR one stopped the link with a
	 * stock AP, and the AP's own command 6 sets it once linked. */
	memcpy(peer_requested, data, 4); peer_ignored++;
	if (!ap_role) {
		memcpy(dev_config.configured_peer, data, 4);
		/* Recovery and stop/start rebuild from the saved loader plan. */
		memcpy(host_config.channels + 8, data, 4);
	}
	return 0;
}
int ar_rf_host_candidates(const uint8_t *data, size_t length)
{
	/* Original 0x221dca copies count four-byte addresses; 0x21b3e2
	 * additionally checks the active peer against the configured/list peers.
	 * Continuous sessions rebuild after accepting active-peer removal. */
	if (!data || length != 402 || data[0] || data[1] > AR_DEV_MAX_CANDIDATES) { return -22; }
	if (receive_operation != 76 || !host_initialized || information.error ||
	    (!host_prepared && information.phase != AR_RF_RECEIVING && !host_start_required)) { return -11; }
	unsigned remove_peer = 0;
	if (!binding.enabled && dev_active && memcmp(receiver.accepted_peer, dev_config.configured_peer, 4)) {
		unsigned match = 0;
		for (unsigned i = 0; i < data[1]; i++) {
			if (!memcmp(receiver.accepted_peer, data + 2 + i * 4, 4)) { match = 1; break; }
		}
		if (!match && !automatic_recovery()) { return -95; }
		remove_peer = !match;
	}
	memcpy(dev_candidates, data + 2, data[1] * 4);
	dev_config.candidate_count = data[1];
	if (ap_role) {
		struct ar_ap_config grounds = ap_link.config;
		grounds.ground_count = data[1] < AR_AP_MAX_GROUNDS ? data[1] : AR_AP_MAX_GROUNDS;
		memcpy(grounds.grounds, data + 2, grounds.ground_count * 4u);
		unsigned previous = ap_link.state;
		ar_ap_configure(&ap_link, &grounds); ap_link_changed(previous);
		return 0;
	}
	if (remove_peer) {
		recovery.candidate_removals++;
		if ((channel_monitor.users[8].flags & 3) == 3) { channel_monitor.users[8].flags |= 4; }
		/* No old-peer TX while waiting for the next monitor interval. */
		request_recovery(3);
	}
	if (!host_prepared || host_start_required) { return 0; }
	host_prepared = 0;
	return start_prepared_receive();
}

static void command_write(uint32_t encoded, uint32_t value, void *context)
{
	(void)context;
	field(encoded, value);
}
static void command_byte(uint32_t offset, uint32_t value, void *context)
{
	(void)context;
	unsigned shift = (offset & 3) * 8;
	update(BB + (offset & ~3u),255u << shift,value << shift);
}
static void tune_metadata(unsigned user, unsigned direction, unsigned property,
	uint32_t value, void *context)
{
	(void)context;
	(void)ar_command_update(&command_program,1,user,direction,property,value,0);
}
static void tune_measurement(void)
{
	unsigned channel = scan_info.channel;
	if (channel >= scan_info.count) { scan_info.errors++; scan_info.phase = 0; return; }
	uint32_t frequency = scan_rows[channel] & 0xffffff;
	scan_info.phase = 0;
	/* Supported loader uses band_single. Original user11 skips the ordinary
	 * users' gain hints and frequency/2000 register fields. */
	const struct ar_tune_config config = {0};
	const struct ar_tune_ops ops = {.field = command_write, .byte = command_byte,
		.metadata = tune_metadata};
	if (ar_tune_apply(11,1,frequency,scan_plls[channel],0,&config,NULL,&ops,NULL)) {
		scan_info.errors++; return;
	}
	tuned_frequency[11][1] = frequency;
	scan_info.frequency_khz = frequency; scan_info.tunes++;
}
static void scan_drop_pending(void)
{
	if (read8(0xe0801000u + irq_vector(5) * 4) & 1) { irq_ack(5); scan_info.stale++; }
}
static void command_prepare(uint32_t observed)
{
	int next = ar_command_step(&command_program,observed,&command_state,!ap_role,0,0,
		read32(BB + 0x48c),command_write,command_byte,NULL);
	if (next < 0) { scan_info.phase = 0; return; }
	unsigned mode = observed >> 20 & 3, raw = observed >> 16 & 15;
	if (mode == 0 || mode == 3) {
		if (raw == 13) {
			scan_drop_pending();
			if (scan_info.phase == 1) { scan_info.phase = 2; scan_info.armed++; }
			else { scan_info.phase = 0; }
		}
		return;
	}
	if ((command_program.entries[next].bits >> 4 & 15) != 13) { return; }
	/* Original ISR ordering cannot be assumed by a polling runtime. A stale
	 * cursor or a changing hardware command cannot justify a tune/sample. */
	if ((command_program.entries[command_program.cursor].bits >> 4 & 15) != raw ||
	    ((read32(BB + 0xe6c) ^ observed) & 0x3f0000)) {
		scan_info.errors++; scan_info.phase = 0; return;
	}
	uint16_t channel = scan_info.channel;
	(void)ar_scan_advance(scan_rows,scan_info.count,&channel,0,0);
	scan_drop_pending(); scan_info.channel = channel;
	tune_measurement(); scan_drop_pending();
	if ((read32(BB + 0xe6c) ^ observed) & 0x3f0000) { scan_info.errors++; }
	else { scan_info.phase = 1; }
}
static int build_commands(void)
{
	/* Mode-0 DEV frame from 0x22c774: RX8, independent measurement RX11,
	 * TX0 with pre-command mode6. RX8/TX0 begin on the link frequency;
	 * RX11 is tuned independently below. Retained TX ownership remains open. */
	static const struct ar_command_descriptor frame[] = {{0x3621,0},{0x358,0},{0x3413,0}};
	/* The AP's broadcast pre-encode (the one that fills the next frame's
	 * first block): at the start of the slot (entry 5: displacement 4, at 0)
	 * instead of stock's 6 into the broadcast. The block was filled 2.05 ms
	 * before it went out at 40 MHz (short frame), now 1.35 (1.0 ms is about
	 * the least: at 0.9 ms more blocks are sent again, at 0.57 they miss).
	 * Bench 2026-11-02, 40 MHz MCS 8, ~20 Mbps video, 4 x 45 s each:
	 * per slice radio->ground 5.13 -> 4.81 ms, capture->picture 19.55 ->
	 * 18.82, blocks sent again 7.3% -> 4.7%. */
	static const struct ar_command_descriptor ap_frame[] = {{0x0621,4},{0x358,0},{0x3413,0}};
	struct ar_command_profile profiles[AR_COMMAND_USERS];
	for (unsigned user = 0; user < AR_COMMAND_USERS; user++) {
		uint32_t rx = read32(BB + 0x9a0 + user * 8);
		uint32_t tx = read32(BB + (user == 10 ? 0xb60 : 0xb00 + user * 4));
		unsigned type = rx & 7, mode = rx >> 6 & 3;
		profiles[user] = (struct ar_command_profile){
			.frequency = {receiver.frequency_khz,receiver.frequency_khz},
			.bandwidth = {tx >> (user == 10 ? 24 : 0) & 7,rx >> 8 & 7},
			.rx_flag = (type == 1 && mode < 2) || (type == 2 && mode >= 2)};
	}
	memcpy(command_state.calibration[2],calibration,sizeof calibration);
	command_state.valid_banks = 4; /* Only bank 2 is calibrated by this runtime. */
	int result = ar_command_build(&command_program,ap_role ? ap_frame : frame,3,!ap_role,profiles,command_write,NULL);
	commands_ready = !result;
	if (!result) { tune_measurement(); }
	return result;
}

static int configure_monitor(uint32_t frequency, uint32_t milliseconds, uint8_t identity_hash,
	unsigned operation, const uint8_t *channels, size_t length, unsigned deferred,
	const struct ar_power_config *power)
{
	progress(0xa000, frequency, milliseconds, operation);
	uint64_t pll;
	unsigned maximum_ms = deferred ? 60000 : operation >= 70 ? 15000 : operation >= 53 ? 10000 : 1500;
	if (information.phase != AR_RF_PSRAM || !pins_configured || operation > (deferred ? 76u : 75u) || milliseconds > maximum_ms || (!milliseconds && deferred < 2) || ar_rf_pll_word(frequency, &pll)) {
		return failure(-22);
	}
	struct ar_power_control prepared_power = {0};
	if (power) {
		/* Only the loader carries complete resolved calibration. Validate it
		 * against the legacy DEV4 gains before any register or state change. */
		if (operation != 76 || !channels || length < AR_RF_DEV4_PREFIX+4 ||
			power->closed[0] != 1 || power->closed[1] != 1 ||
			power->high_clock != 1 || power->dev_role != (memcmp(channels, "AP04", 4) != 0)) { return -22; }
		struct ar_power_plan initial;
		int result = ar_power_plan(power,0,channels[28],&initial);
		if (result) { return result; }
		for (unsigned i = 0; i < 4; i++) {
			if (initial.writes[i].value != channels[24+i]) { return -22; }
		}
		prepared_power.config = *power;
	}
	uint8_t fixed_channel[4], tx_gain[4] = {0};
	const uint8_t *scan_channels = NULL; unsigned scan_count = 0;
	struct ar_dev_mcs_config policy = {0};
	if (operation >= 54) {
		unsigned prefix = operation >= 67 ? AR_RF_DEV4_PREFIX : operation >= 59 ? 32 : operation >= 58 ? 28 : 24;
		if (!channels || length < prefix + 4 || (length - prefix) % 4 ||
		    (length - prefix) / 4 > AR_RF_MAX_CHANNELS ||
		    (memcmp(channels, operation >= 67 ? "DEV4" : operation >= 61 ? "DEV3" : operation >= 59 ? "DEV2" : operation >= 58 ? "DEV1" : "DEV0", 4) &&
		     (operation < 67 || memcmp(channels, "AP04", 4))) || channels[20] > 2 ||
		    channels[21] >= (length - prefix) / 4) { return failure(-22); }
		if (operation >= 58 && (channels[24] > 63 || channels[26] > 63)) { return failure(-22); }
		if (operation >= 59 && (channels[28] > 63 || channels[29] > (operation >= 61 ? 127 : 0) || channels[30] || channels[31])) { return failure(-22); }
		if (operation >= 67) {
			if (ar_dev_mcs_decode_config(&policy, channels + 32, AR_DEV_MCS_CONFIG_BYTES)) { return failure(-22); }
			for (unsigned i = 0; i < 25; i++) {
				uint32_t key = le32(channels + 168 + i * 4);
				if (key & ~UINT32_C(0x01070303)) { return failure(-22); }
				for (unsigned j = 0; j < i; j++) {
					if (key == le32(channels + 168 + j * 4)) { return failure(-22); }
				}
			}
		}
		unsigned count = (length - prefix) / 4, index = channels[21];
		uint8_t hash = 0;
		for (unsigned i = 0; i < 4; i++) { hash ^= channels[4 + i]; }
		if (hash != identity_hash || le32(channels + prefix + index * 4) != frequency) { return failure(-22); }
		for (unsigned i = 0; i < count; i++) {
			uint64_t checked;
			uint32_t value = le32(channels + prefix + i * 4);
			if (value <= 4300000 || ar_rf_pll_word(value, &checked)) { return failure(-22); }
			for (unsigned j = 0; j < i; j++) {
				if (value == le32(channels + prefix + j * 4)) { return failure(-22); }
			}
		}
		if (operation == 76) { scan_channels = channels + prefix; scan_count = count; }
		ap_role = operation >= 67 && !memcmp(channels, "AP04", 4);
		memset(&dev_config, 0, sizeof dev_config);
		memcpy(dev_config.local, channels + 4, 4);
		memcpy(dev_config.configured_peer, channels + 8, 4);
		memcpy(dev_candidates, channels + 12, 8);
		dev_config.candidates = dev_candidates; dev_config.candidate_count = channels[20];
		dev_config.receive_channel = index; dev_transmit_channel = index; dev_config.channel_count = count;
		dev_config.slot_bitmap = 1;
		dev_status_a = channels[22]; dev_status_b = channels[23];
		if (operation >= 58) { memcpy(tx_gain, channels + 24, 4); }
		dev_power_index = operation >= 59 ? channels[28] : 0;
		ar_dev_retx_init(&dev_retx, operation >= 61 ? channels[29] : 0);
		if (operation >= 67) {
			(void)ar_dev_mcs_init(&dev_mcs, &policy);
			bw_policy_rows();
			/* An AP's own policy is the ground's uplink one; its channel
			 * mode starts as the configuration's channel.mode. */
			if (ap_role) { ap_up_config = policy; ap_up_build(); chan_auto = ap_hop.auto_start; }
			mcs_sync_mmio = channels[42];
			/* Remote sync as stock's DEV runs it (0x22511e): 0xd30[5:0] 0x3f,
			 * [7:6] 3, [15:8] one per user, and [23:16] 0xff while one of the
			 * DEV's own commands is queued in its control block. */
			mcs_sync_mmio = 1; field(0xd300010, 0xffff);
			for (unsigned i = 0; i < 25; i++) { mcs_fch_keys[i] = le32(channels + 168 + i * 4); }
		}
		if (deferred == 3) {
			/* Normal loader init_chan=-1 searches its configured list. */
			if (index) { return failure(-22); }
			channels += prefix; length -= prefix;
		} else {
			if (ap_role) { ap_channel_count = count; ap_channel_start = index; memcpy(ap_channels, channels + prefix, count * 4); }
			memcpy(fixed_channel, channels + prefix + index * 4, 4);
			channels = fixed_channel; length = 4;
		}
	}
	if (operation >= 39) {
		if (!channels || !length || length % 4 || length > AR_RF_MAX_CHANNELS * 4 || le32(channels) != frequency) { return failure(-22); }
		for (size_t i = 0; i < length / 4; i++) {
			uint32_t next = le32(channels + i * 4);
			/* This bring-up has calibration and pin setup only for band 2. */
			if (next <= 4300000 || ar_rf_pll_word(next, &channel_plls[i])) { return failure(-22); }
			for (size_t j = 0; j < i; j++) {
				if (next == le32(channels + j * 4)) { return failure(-22); }
			}
		}
	} else if (length) { return failure(-22); }
	commands_ready = 0; listener_capture = listener_follow = 0;
    memset(&listener_change,0,sizeof listener_change);
	memset(&scan_info,0,sizeof scan_info); scan_info.version = 2; scan_info.count = scan_count;
	memset(scan_history,0,sizeof scan_history);
	/* Collect base-channel measurements. Group policy and the original initial
	 * scan's RX hold/selection message are not connected to acquisition yet. */
	scan_scores = (struct ar_scan_scores){.band_mask=4,.initial_band=2,.selected=255};
	for (unsigned i = 0; i < scan_count; i++) {
		scan_rows[i] = le32(scan_channels + 4*i) | 0x11000000;
		(void)ar_rf_pll_word(scan_rows[i] & 0xffffff,&scan_plls[i]);
	}
	scan_apply_work_list();
	memset(&receiver, 0, sizeof receiver);
	memset(&tx_payload_info, 0, sizeof tx_payload_info);
	ar_dev_stream_init(&dev_stream);
	memset(&phy, 0, sizeof phy);
	memset(&phy_trace, 0, sizeof phy_trace); phy_poll_tick = 0;
	memset(&sync_info, 0, sizeof sync_info);
	memset(phy_control.rx,0,sizeof phy_control.rx);
	memset(phy_control.tx,0,sizeof phy_control.tx);
	phy_control.tx[0] = ar_phy_control_checksum(phy_control.tx); phy_control.ticks = 0;
	memset(&traffic_info, 0, sizeof traffic_info); traffic_info.version = 1;
	memset(&tx_mcs_info, 0, sizeof tx_mcs_info); tx_mcs_info.current = 1;
	memset(&tx_interleaver,0,sizeof tx_interleaver);
	memset(&tx_history_info,0,sizeof tx_history_info);
	/* A recovery starts a new producer session. Do not carry commands,
	 * quality samples or counter deltas across the reset. Engines are held
	 * here, so release any abandoned bandwidth-transition holds now. */
	if (bw_settle_step) {
		field(0x9e41801, 0); field(0xcfc1701, 0); field(0xbf80701, 0);
	}
	memset(&dev_cmd, 0, sizeof dev_cmd);
	quality_head = quality_count = chain_snr_fresh = 0;
	phy_last_invalid = phy_last_mac_invalid = 0;
	/* Operations 41/42 below program the configured 5 MHz receive profile. */
	bw_code = 2; bw_state = 0; bw_request_pending = 0; bw_settle_step = 0; bw_at_sweep = 0;
	apply_ceiling();   /* a new link starts at 5 MHz: the host's ceiling alone */
	bw_due_code = 0; bw_mcs_settling = 0;
	bw_target = bw_automatic ? 2 : bw_manual; ar_width_reset(&bw_width);
	bw_info.requests = 0;   /* per link: each new link widens again from 5 MHz */
	memset(&mcs_info, 0, sizeof mcs_info); mcs_info.first_observed_mcs = 25;
	if (operation < 67) { memset(&dev_mcs, 0, sizeof dev_mcs); mcs_sync_mmio = 0; }
	memset(phy_header_valid, 0, sizeof phy_header_valid); memset(phy_eop_seen, 0, sizeof phy_eop_seen);
	dev_active = 0;
	/* Stock's preparation cache belongs to the calibrated radio, not a link
	 * session. Mode-0 engine resets preserve it. Only normal host contexts
	 * may reuse this single supported band; diagnostics can replace fields
	 * independently and must not share their cached state with normal use. */
	if (operation != 76 || receive_operation != 76) { command_state.cached_bank = 3; command_state.board_band = -1; }
	memset(&channel_monitor,0,sizeof channel_monitor);
	monitor_last_ms = 0; monitor_interval_ms = 0; recovery_reset_pending = 0;
	receiver.before_monitor_header = read_header_status();
	read_control_header(receiver.before_monitor_control);
	if (operation >= 39) {
		receiver.channel_count = length / 4;
		for (unsigned i = 0; i < receiver.channel_count; i++) {
			receiver.channels[i].frequency_khz = le32(channels + i * 4);
		}
		if (ap_role && ap_channel_count) {
			receiver.channel_count = ap_channel_count;
			for (unsigned i = 0; i < ap_channel_count; i++) { receiver.channels[i].frequency_khz = le32(ap_channels + i * 4); }
			receiver.channel_index = ap_channel_start;
		}
	}
	receiver.frequency_khz = frequency;
	power_select_band(frequency);
	receive_head = receive_pending = 0;
	receive_operation = operation;
	normal_search = deferred == 3; search_candidate_seen = 0;
	receive_milliseconds = milliseconds; host_prepared = 0;
	if (deferred) { field(0x8000201, 1); field(0x8000001, 1); }
	/* Fixed-channel receive diagnostic, not a complete link/session scheduler. */
	serial_word(0x905, pll); /* DEV broadcast RX user 8. */
	if (operation) { serial_word(0x900, pll); }
	serial_word(0x90a, pll); /* Search RX user 11. */
	/* The dedicated calibration PLL bank remains at its calibration word in
	 * the working stock receiver; normal commands select per-user shadows. */
	serial_word(0x4d0, UINT64_C(0x0113000000));
	configure_receive_context(pll);
	tx_power = prepared_power;
	if (power) {
		/* Same ordered user0 writes as the plan; retain calibration and cached
		 * level for subsequent tuning. Other users remain uninitialized. */
		int result = ar_power_control_apply(&tx_power,0,dev_power_index,command_write,NULL);
		if (result) { return failure(result); }
	} else if (operation >= 58) {
		/* 0x24528a..0x2452de, user0 with both per-path controls enabled.
		 * The plan supplies effective calibration fields after any factory
		 * offset/clock scaling. No proprietary gain table is embedded and
		 * these fields do not constitute dynamic power control. */
		field(0x9a40806, tx_gain[0]); field(0x9a40008, tx_gain[1]);
		field(0xda40006, tx_gain[2]); field(0xda01808, tx_gain[3]);
	}
	/* Post-calibration PHY initialization is distinct from reset defaults.
	 * Ascent VRX uses the measured high-rate clock flag and DEV role, mode 0. */
	field(0x8001301, 1); field(0xa600401, 0);
	field(0x8000701, 0); field(0x8001002, 0); field(0x80c0002, 0);
	field(0xb201e01, 1);
	/* Per-user receive buffer controls are initialized after PSRAM in stock.
	 * Clear only the ten documented bit-5 fields, preserving adjacent users. */
	for (unsigned user = 0; user < 10; user++) {
		field(((user >> 2) + 0x2fc) * 0x40000 | ((user & 3) * 0x800 + 0x500) | 1, 0);
	}
	/* Closed-loop power setup follows user profiles in the stock initializer. */
	field(0xa281901, 1); field(0xa580006, 0); field(0xa580806, 0);
	/* Corroborated post-user-configuration helper for this profile. Reset
	 * defaults leave these controls masked; the working stock readback is 0. */
	update(BB + 0xbac, 0xffffc0c0, 0);
	update(BB + 0xbd4, 0x0003ff00, 0);
	field(0xa400020, 0x0d0d0d0d); field(0xa480020, 0x80808080); field(0xa200301, 1);
	field(0x9f81601, 1); field(0x9f81808, 0x3c);
	field(0x9e41903, 4); field(0x9e41601, 1); field(0x9e41201, 1);
	if (operation) { field(0x9e41d03, 4); field(0x9e41701, 1); field(0x9e41301, 1); }
	field(0xa900010, frequency / 2000);
	field(0xb400817, 0); field(0xb440013, 0x36000);
	field(0xba01010, 0x4000); field(0xba40007, 0x1d); field(0xba41010, 0); field(0xba80004, 9);
	field(0xcec0817, 0x654000); field(0xcf00014, 0x28800); field(0xcf00002, 2);
	field(0x8081801, 0); field(0x8081901, 0); field(0xcd00001, 1);
	/* PHY correlation/scrambler identity: the stock initializer programs it
	 * for all RX users from the XOR of the four-byte local baseband MAC. */
	for (unsigned user = 0; user < 10; user++) { field((user + 0x1b4) * 0x80000 | 8, identity_hash); }
	field(0xdf00008, identity_hash);
	for (unsigned user = 0; user < 10; user++) {
		field((((user + 2) & ~3u) + 0xb7c) * 0x10000 | ((user + 2) & 3) << 11 | 8,
			user == 0 ? identity_hash : 0);
	}
	field(0xb601008, identity_hash); field(0xb601803, 1); field(0xb601b02, 1);
	if (operation >= 44) {
		/* Before the first TX preparation, retain initializer defaults. Once
		 * loaded, preserve compensation through normal restart/recovery, as
		 * 0x223128's cached-bank check and mode-0 reset helpers do. Clearing it
		 * here could expose a pre-encode before the next prepare notification.
		 * Other calibration banks and retained TX-buffer ownership remain unported. */
		if (command_state.cached_bank == 3) {
			field(0x4800101, 0);
			field(0x4a00010, 0); field(0x4a41010, 0);
			field(0x48c0008, read32(BB + 0x48c) & 0x33);
		}
	} else {
		/* Historical globally-applied compensation diagnostic. */
		field(0x4800101, 1);
		field(0x4a00008, calibration[0]); field(0x4a00808, calibration[1]);
		field(0x4a41008, calibration[2]); field(0x4a41808, calibration[3]);
		field(0x48c0008, calibration[4] | (read32(BB + 0x48c) & 0x33));
	}
	if (operation >= 45) {
		/* DEV initializer 0x22373a -> 0x2236e0 writes 1 to B4C[7:3].
		 * Trial-67 prestart readback corroborates this role-dependent field;
		 * its later per-user updates and software state remain unported. */
		field(0xb4c0305, 1);
	}
	progress(0xa001, read32(BB + 0xfa8), 0, 0);
	/* Expanded prepare/RX/wait sequence and user metadata, from the command contract. */
	static const uint8_t operations[] = {0x11, 0x22, 0x13, 0x23, 0x24, 0x15, 0x58,
		0x11, 0x11, 0x11, 0x11, 0x13, 0x13, 0x11, 0x13, 0x13, 0x11, 0x13};
	uint8_t command = operation >= 18 ? 0x13 : operations[operation];
	uint8_t prefix = (command & 0x30) == 0x10 ? 0x3a : 9;
	write32(BB + 0x810, (uint32_t)0x5c << 24 | (uint32_t)0x3a << 16 | (uint32_t)command << 8 | prefix);
	write32(BB + 0x814, 0x80);
	write32(BB + 0x870, operation == 6 ? 0x000feedd : 0x000fee88);
	for (unsigned offset = 0x8a0; offset < 0x900; offset += 4) { write32(BB + offset, 0); }
	field(0x9701010, 0xffff); /* Bounded monitor handles completion; no short RX cutoff. */
	if (operation) { field(0x9700010, 0xffff); }
	field(0x8081d01, 1); field(0x8081e01, 0);
	if (operation == 7) {
		/* Observed stock DEV receive/search loop: six command bytes, separate
		 * per-step user nibbles, and a search pre-command power context. */
		write32(BB + 0x810, 0x183a113a); write32(BB + 0x814, 0x00802309);
		write32(BB + 0x870, 0x0f00dd88); write32(BB + 0x8a4, 0x0000e000);
		field(0x9800010, 0xffff);
	}
	if (operation >= 8) {
		unsigned user = operation == 9 ? 13 : operation >= 13 ? 8 : 0;
		write32(BB + 0x870, 0x000fee00 | user << 4 | user);
		if (!user) { field(0xa100010, 0xffff); }
		else if (user == 13) { field(0x9800010, 0xffff); }
	}
	if (operation == 15 || operation == 16) {
		/* One logical receive command expands to prepare/body, followed by
		 * the end flag. No additional wait command or invalid-user context. */
		write32(BB + 0x810, 0x0080003a | (uint32_t)command << 8);
		write32(BB + 0x814, 0);
		write32(BB + 0x870, operation == 16 ? 0x00000f88 : 0x00000f00);
		if (operation == 15) { field(0xa100010, 0xffff); }
	}
	if (operation == 17) {
		write32(BB + 0x810, 0x133a113a); write32(BB + 0x814, 0x80);
		write32(BB + 0x870, 0x000f0088);
		field(0xa100010, 0xffff);
	}
	if (operation >= 18 && operation <= 24) {
		static const uint8_t raw_users[] = {0, 8, 9, 10, 11, 12, 13};
		unsigned user = raw_users[operation - 18];
		write32(BB + 0x810, 0x0080133a); write32(BB + 0x814, 0);
		write32(BB + 0x870, 0xf00 | user << 4 | user);
		field(0xa100010, 0x0da3); field(0x9701010, 0x208a);
		field(0x99c1010, 0x0b32); field(0x9800010, 0x009c); field(0x9801010, 0x041d);
		field(0x8080e01, 0);
	}
	if (operation >= 25) {
		unsigned user = operation == 25 ? 0 : operation == 26 ? 8 : 13;
		unsigned body = operation == 27 ? 0x58 : 0x11;
		write32(BB + 0x810, 0x0080003a | body << 8); write32(BB + 0x814, 0);
		write32(BB + 0x870, 0xf00 | user << 4 | user);
		field(0x9f80003, 0); field(0x9f80301, 0); field(0x9f80602, 0);
		field(0x9800010, 0x009c); field(0x9801010, 0x041d);
		field(0xa100010, 0x0da3); field(0x9701010, 0x208a);
	}
	if (operation >= 29 && operation <= 34) {
		/* Continuous slot-0 reception with the measured broadcast modulation
		 * profile. SDK bandwidth codes are 0=1.25,1=2.5,2=5,3=10,4=20,5=40 MHz. */
		unsigned bandwidth = operation - 29;
		field(0x9a00003, 5); field(0x9a00d03, 6); field(0x9a00803, bandwidth);
		field(0x9a01808, 0x20);
		field(0xb000003, bandwidth); field(0xb001203, 3); field(0xb000401, 1);
		write32(BB + 0x810, 0x0080133a); write32(BB + 0x814, 0);
		write32(BB + 0x870, 0xf00); field(0xa100010, 0x208a);
	}
	if (operation >= 35) {
		/* Corroborated DEV/mode-0 link-start prerequisites, not the already
		 * linked register snapshot: link task message 0x200 -> 0x22b98e(0),
		 * 0x22d71c(...,1), then 0x22aaea -> 0x222bde(8,1). Physical meaning
		 * beyond these field contracts remains under investigation. */
		field(0xcf41501, 0);
		field(0xde00008, 0);
		field(0xbf00002, 0); /* Mode 0 maps logical broadcast user 8 to buffer user 0. */
		write32(BB + 0x810, 0x183a113a); write32(BB + 0x814, 0x00802309);
		write32(BB + 0x870, 0x0f00dd88); write32(BB + 0x8a4, 0x0000e000);
		field(0x9701010, 0x208a); field(0x9800010, 0x009c);
	}
	if (operation >= 41) {
		/* Ascent VRX 18.21.10 external DEV br.rx.bandwidth is 5 MHz (code 2).
		 * Startup 0x2251ec -> 0x22d3a6 -> 0x22ad12 applies that configuration;
		 * the previous 20 MHz value came from an already-linked snapshot.
		 * Keep operation 39 as the control. Frame duration uses the same
		 * non-40-MHz branch in 0x22cea8. Software pre-command metadata and
		 * later negotiated bandwidth changes are not implemented here. */
		field(0x9e00803, 2);
	}
	if (operation >= 42) {
		/* External br.rx: payload on, tintlv enabled, length 12 (code 2),
		 * number 2 (code 1). 0x22d3a6 maps these to selector 4 / interleave 5.
		 * RX8 duration 0x22cea8: floor(((17/16*26)+3)*2048/22.4)+200
		 * = 3000 us; 0x22b40c rounds this to count 4688 (0x1250).
		 * Preserve operation 41's bandwidth-only comparison. */
		field(0x9e00003, 4); field(0x9e00d03, 5);
		field(0x9701010, 0x1250);
	}
	if (operation >= 43) {
		/* Stock prestart observer trial 67 corroborates RX8's default 0x3c.
		 * 0x24c93c writes the tuning-table/default value; 0x2371e4 writes
		 * 0x20 only for link state 1. Its physical interpretation remains
		 * unverified. Operation 42 retains the linked-snapshot comparison. */
		field(0x9e01808, 0x3c);
	}
	if (operation >= 46 && operation <= 52) {
		/* Compare the previously observed linked RX8 bandwidth/interleaver
		 * against configured startup, retaining startup upper byte and all
		 * operation-45 prerequisites. This does not assert the peer's current
		 * profile: a native load interrupts an already-established link. */
		field(0x9e00803, 4); field(0x9e00003, 5); field(0x9e00d03, 6);
		field(0x9701010, 0x208a);
	}
	if (operation >= 56) {
		/* Fixed DEV startup TX0 profile, corroborated by stock trial 94 and
		 * 0x24d13a/0x221610: internal MCS index 1, bandwidth code 1,
		 * interleave length code 2 -> modulation 0, code length/rate 3,
		 * rate field 1. This is not the later adaptive MCS policy.
		 * 0x22cea8 with this configured context gives
		 * floor(((17/16 * (12+2)) + 3) * 2048/22.4) + 600 = 2234 us.
		 * 0x22b40c rounds microseconds to 640 ns hardware counts.
		 * Program before deriving search dwell; preserve the other slot. */
		field(0xb000803, 0); field(0xb000c04, 3); field(0xb001002, 1);
		field(0xa000010, (2234u * 10000 / 640 + 5) / 10);
		if (operation >= 72) {
			(void)ar_tx_mcs_control(&tx_interleaver,AR_TX_MCS_INITIALIZE,0,0,
				mcs_fch_keys,information.chip_profile,1,&tx_mcs_ops,NULL);
			tx_mcs_info.current = tx_interleaver.users[0].mcs;
		}
		frame_mode = 0; frame_pending = 0; frame_want = 0;   /* a new link starts on the standard frame */
		memset(&distance, 0, sizeof distance); memset(&distance_avg, 0, sizeof distance_avg);
	}
	if (operation == 36 || operation == 37 || operation == 39 || operation >= 41) {
		/* 0x222cac(8,0) also clears the shared identity field. Trial 59
		 * cleared only DE0. Keep operation 35 available as the old control. */
		field(0xdf00008, 0);
		receiver.search_frame_us = dev_frame_microseconds();
		receiver.search_dwell_ms = receiver.search_frame_us / 1000 + 2;
		search_pll = pll;
	}
	if (operation == 47) {
		/* Working-stock readback retains the selected local identity in RX8
		 * and its shared field. Compare this with the acquisition clear;
		 * keep it explicit rather than assuming the peer's current state. */
		field(0xde00008, identity_hash); field(0xdf00008, identity_hash);
	}
	if (operation == 48) {
		/* Isolate the RX8 upper byte written by stock 0x2371e4 in link
		 * state 1. Retain operation 46's zero acquisition identity; this
		 * profile comparison does not perform or claim link acquisition. */
		field(0x9e01808, 0x20);
	}
	if (operation >= 49 && operation <= 52) {
		/* Stock 0x223128 selects A48=0 for a following transmit command
		 * with bandwidth code below 4. Both TX profiles in this diagnostic
		 * satisfy that case; working-stock readback also has zero. Retain
		 * operation 46's acquisition RX8 profile and TX hold. */
		field(0xa480020, 0);
	}
	/* Operation 53 retains operation 45's configured 5 MHz/length-12
	 * acquisition profile and pre-TX A48 value. Stock trial 94 links with
	 * those fields and takes several seconds; this longer diagnostic uses
	 * checksum-aware retries without claiming a stock MAC transition. */
	if (operation >= 37) {
		/* Mode-0 startup (0x22511c..0x22512a -> 0x21db2e) holds TX
		 * before DEV search. RX preparation and both-engine restart preserve
		 * this bit. PSRAM setup previously cleared it and the monitor never
		 * reinstated it. A future linked-state transition must release it;
		 * this bounded search diagnostic does not claim that transition. */
		field(0x8000201, 1);
	}
	if (operation >= 54) {
		/* Original mode-0 link initialization seals seven zero bytes through
		 * 0x241544 -> 0x2299d2, then writes two words through 0x24b514.
		 * The 18-byte 0x229ee6 path is gated on mode 3, not used here. */
		uint8_t sync[8] = {0}; sync[0] = ar_rf_control_checksum(sync);
		receiver.sync_control_words[0] = le32(sync);
		receiver.sync_control_words[1] = le32(sync + 4);
		write32(BB + 0xb8c, receiver.sync_control_words[0]);
		write32(BB + 0xb90, receiver.sync_control_words[1]);
		barrier();
	}
	if (operation == 38 || operation == 40) {
		/* Initial frequency-search descriptors 0x358/0x39c expand to
		 * prepare/power/prepare/wait for raw users 13/14 (0x21dfea).
		 * Stock startup at 0x225996..0x225a16 sets these wait/power fields.
		 * Observe one fixed channel; power samples are not received packets. */
		write32(BB + 0x810, 0x1c3a183a); write32(BB + 0x814, 0x80);
		write32(BB + 0x870, 0x000feedd); write32(BB + 0x8a4, 0);
		field(0x9800010, 0x009c);
		field(0x9880010, 2500); field(0x98c1c04, 0);
		/* Diagnostic survey dwell, not a claim to implement stock averaging. */
		if (operation == 40) { receiver.search_dwell_ms = 10; }
	}
	field(0x8080f01, 1); /* Unused special-user TX disabled, as in stock DEV. */
	if (ap_role) {
		ap_identity_hash = identity_hash;
		ap_configure_radio(pll, frequency);
		struct ar_ap_config link = {.ground_count = 0, .channel = dev_config.receive_channel,
			.status = 0x50, .login_value = 0x0f, .timeout_ms = 1000};
		memcpy(link.local, dev_config.local, 4);
		ar_ap_init(&ap_link, &link, (uint8_t)(read32(TIMER) | 1));
		ar_dev_stream_init(&dev_stream);
	}
	if (operation == 76) {
		int result = build_commands();
		if (result) { return failure(result); }
	}
	if (deferred) {
		host_prepared = 1; information.error = 0;
		return 0;
	}
	return start_prepared_receive();
}

static int start_prepared_receive(void)
{
	if (automatic_recovery()) {
		/* Retain the compared threshold/interval calculation; observation and
		 * periodic state now belong to the complete channel monitor. */
		struct ar_fch_monitor timing;
		if (ar_fch_monitor_init(&timing,receiver.search_frame_us,timer_ticks()/40000)) { return failure(-95); }
		memset(&channel_monitor,0,sizeof channel_monitor);
		channel_monitor.role = 1; channel_monitor.mode = dev_config.mode; channel_monitor.band = 2;
		channel_monitor.users[8].threshold = timing.threshold;
		monitor_last_ms = timing.last_ms; monitor_interval_ms = timing.interval_ms;
	}
	/* Stock interrupt-context critical sections use the high byte of CLIC's
	 * threshold register. Mask delivery while preserving pending-bit polling. */
	write32(0xe0800008, 0xff000000);
	for (unsigned index = 0; index < 20; index++) {
		uint32_t address = 0xe0801000u + irq_vector(index) * 4;
		write8(address + 2, 1); write8(address + 1, read8(address + 1) | 1);
		irq_ack(index);
	}
	information.phase = AR_RF_RECEIVING;
	progress(0xa002, read32(BB + 0xfa8), 0, 0);
	/* Match the stock receive-start reset/idle ordering, rather than merely
	 * unfreezing a controller left in the calibration operation. */
	field(0x8000101, 1); field(0x80c0201, 1);
	int reset_result = wait_bits(BB + 0xfa8, 0x100, 0x100, 2000);
	receiver.before_release_header = read_header_status();
	field(0x80c0201, 0); field(0x8000101, 0);
	progress(0xa003, (uint32_t)reset_result, read32(BB + 0xfa8), 0);
	if (reset_result || idle()) { information.phase = AR_RF_PSRAM; return failure(-110); }
	link_event(0, 3);
	if (receiver.channel_count) { receiver.channels[0].visits = 1; }
	progress(0xa004, read32(BB + 0xfa8), 0, 0);
	receive_started = timer_ticks();
	search_started = (uint32_t)receive_started;
	receive_iterations = 0;
	information.error = 0;
	if (receive_operation == 76) { lifecycle.starts++; lifecycle.last_change_ms = 0; }
	return 0;
}

void ar_rf_poll(void)
{
	if (information.phase != AR_RF_RECEIVING) { return; }
	if (recovery.pending && automatic_recovery()) { recover_host(); return; }
	adc_poll();
	if (ap_role) {
		static uint32_t previous;
		uint32_t now = elapsed_us(), gap = now - previous;
		static const uint16_t limits[5] = {50, 100, 200, 500, 1000};
		if (previous) {
			for (unsigned i = 0; i < 5; i++) { if (gap > limits[i]) { ap_info.poll_gaps[i]++; } }
#ifdef AR_BW_TRACE
			if (gap > ap_tf_gap) { ap_tf_gap = gap; }
#endif
			if (gap > ap_info.poll_gap_max) { ap_info.poll_gap_max = gap; }
			if (read8(0xe0801000u + irq_vector(19) * 4) & 1) {
				for (unsigned i = 0; i < 5; i++) { if (gap > limits[i]) { ap_info.pre_gaps[i]++; } }
				if (gap > ap_info.pre_gap_max) { ap_info.pre_gap_max = gap; }
			}
		}
		previous = now;
		ap_poll();
		ap_stage_poll();
	}
	bandwidth_poll();
	if (!ap_role) { distance_push(); }
	if (receive_operation >= 68) {
		uint32_t tick = read32(TIMER);
		if (receiver.dev_state == 2) {
			uint32_t gap = (uint32_t)(tick - phy_poll_tick) / 40;
			phy_trace.polls++;
			if (phy_poll_tick && gap > phy_trace.max_poll_gap_us) { phy_trace.max_poll_gap_us = gap; }
		}
		phy_poll_tick = tick;
	}
	if (!(receive_iterations & 1023)) { progress(0xa005, receive_iterations, (uint32_t)elapsed_ticks(), receiver.irq_total); }
		/* The AP takes IRQ 17 (its broadcast's last block out, ~0.3 ms
		 * before the sweep ends) ahead of IRQ 5 when a late pass finds both:
		 * in index order the IRQ 5 read the PHY block a tick behind - a
		 * width switch a frame after the ground's, a stamp a frame off. */
		static const uint8_t ap_order[20] = {0,1,2,3,4,17,5,6,7,8,9,10,11,12,13,14,15,16,18,19};
		for (unsigned position = 0; position < 20; position++) {
			unsigned index = ap_role ? ap_order[position] : position;
			uint32_t address = 0xe0801000u + irq_vector(index) * 4;
			if (!(read8(address) & 1)) { continue; }
			struct ar_rf_phy_event trace;
			unsigned tracing = receive_operation >= 68 && receiver.dev_state == 2 &&
				(index == 12 || index == 13 || index == 14);
			if (tracing) { phy_trace_begin(&trace, index); }
			receiver.irq_counts[index]++; receiver.irq_total++;
			if (TRACE_ON) { ap_trace_irq(index); }
			if (receive_operation >= 54) {
				receiver.irq_last_elapsed_us[index] = elapsed_us();
			}
			irq_ack(index);
			if (ap_role && ap_sweep_us) {
				uint32_t since = elapsed_us() - ap_sweep_us;
				if ((index == 2 && since > 900) ||
			    ((index == 12 || index == 15) && since > ap_rx_end_us && since < ap_frame_us - 600)) {
					ap_late_now = 1;
				}
			}
			if (index == 16) { receive_event(); }
			else if (index == 6) { distance_sample(); }
			else if (index == 13) {
#ifdef AR_BW_TRACE
				if (ap_role) { ap_tf_13++; }
#endif
				uint32_t started = elapsed_us(); receive_complete();
				uint32_t took = elapsed_us() - started;
				phy_trace.reserved[0] += took; phy_trace.reserved[1]++;
				if (took > phy_trace.reserved[2]) { phy_trace.reserved[2] = took; }
			}
			else if (index == 19) { if (ap_role) { ap_preencode(); } else { dev_preencode(); } }
			else if (index == 18 && receive_operation >= 54) {
				uint32_t context = read32(BB + 0xf68), request = read32(BB + 0xf64);
				unsigned user = (context >> 4) & 15;
				if (receiver.irq_counts[index] == 1) {
					receiver.interleaver_first_f68 = context;
					receiver.interleaver_first_f64 = request;
				}
				receiver.interleaver_last_f68 = context;
				receiver.interleaver_last_f64 = request;
				receiver.interleaver_users[user]++;
				receiver.interleaver_b4c_before = read32(BB + 0xb4c);
				if (receive_operation >= 55) {
					/* Original 0x227e78..0x227e88: executed independently of
					 * conditional clock adjustment and MCS retry bookkeeping.
					 * Preserve the four-bit raw user; this is not MAC admission. */
					field(0xb4c0305, (user << 1) | 1);
					receiver.interleaver_updates++;
				}
				receiver.interleaver_b4c_after = read32(BB + 0xb4c);
				if (receive_operation >= 72 && dev_active) {
					unsigned second_user = (read32(BB + 0xf68) >> 4) & 15;
					if (!second_user) {
						(void)ar_interleaver_tx_step(&tx_interleaver,0,&tx_step_ops,NULL);
					}
				}
				if (receive_operation >= 67 && dev_active && user != 8) {
					phy_configure();
					(void)ar_phy_control_tick(&phy_control,timer_ticks()/40000,&phy_ops,NULL);
				}
				if (ap_role && user == 8) {
#ifdef AR_BW_TRACE
					ap_tf_18u8++;
					ap_tf_req = (request >> 20 & 15) | ((read32(BB + 0xb20) & 0x10) ? (request >> 16 & 15) << 4 : 0xf0);
#endif
					static unsigned stalled;
					unsigned now_stalled = (read32(BB + 0xf70) & 0xc0) == 0xc0;
					if (now_stalled && !stalled) {
						ap_info.tx_stalls++;
						ap_trace_mark(35, request, read32(BB + 0xb20), ap_link.state);
						/* With the stall freeze armed (aptool unfreeze 2): keep 150 interrupts after it starts. */
						if (!ap_trace.frozen && ap_trace.freeze_at == UINT32_MAX - 1) { ap_trace.freeze_at = ap_trace.head + 150; }
					}
					stalled = now_stalled;
				}
				if (ap_role && user == 8 && ap_link.state == AR_AP_CONNECTED) {
					uint8_t first = (request >> 20) & 15;
					uint8_t second = (read32(BB + 0xb20) & 0x10) ? (request >> 16) & 15 : 255;
					ap_hop_feed(first, second);
					enum ar_dev_retx_action action = ar_dev_retx_feed(&ap_retx, first, second);
					if (action == AR_DEV_RETX_DISABLE || action == AR_DEV_RETX_ENABLE) {
						field(0xb201e01, action == AR_DEV_RETX_DISABLE);
						if (action == AR_DEV_RETX_DISABLE) { ap_info.retx_disables++; } else { ap_info.retx_enables++; }
					}
				}
				if (receive_operation >= 61 && dev_active && user == 0) {
					uint8_t first = (request >> 20) & 15;
					uint8_t second = (read32(BB + 0xb00) & 0x10) ? (request >> 16) & 15 : 255;
					receiver.retx_events++; receiver.retx_request_ids[first]++;
					receiver.retx_last_request = first | (uint32_t)second << 8;
					enum ar_dev_retx_action action = ar_dev_retx_feed(&dev_retx, first, second);
					if (action == AR_DEV_RETX_INVALID) { receiver.retx_errors++; }
					if (action == AR_DEV_RETX_DISABLE || action == AR_DEV_RETX_ENABLE) {
						receiver.retx_b00_before = read32(BB + 0xb00);
						/* 0x22c6ec writes the inverse enable value to B00 bit30.
						 * Preserve MCS/profile fields and ownership of all TX data. */
						field(0xb001e01, action == AR_DEV_RETX_DISABLE);
						receiver.retx_b00_after = read32(BB + 0xb00);
						if (action == AR_DEV_RETX_DISABLE) {
							uint32_t elapsed = elapsed_us();
							if (!receiver.retx_disable) { receiver.retx_first_disable_us = elapsed; }
							receiver.retx_disable++; receiver.retx_last_disable_us = elapsed;
						} else { receiver.retx_enable++; }
					}
					receiver.retx_last_id = dev_retx.last_id; receiver.retx_repeated = dev_retx.repeated;
					receiver.retx_phase = dev_retx.phase;
				}
			}
			else if (index == 11 && ap_role) {
				ap_info.tx_end++;
#ifdef AR_BW_TRACE
				ap_tf_11++;
#endif
				if (ap_bw_code >= 4) { write32(BB + 0xa48, ap_bw_code == 5 ? 0x80808080 : 0x40404040); }   /* the next broadcast block's (IRQ 7) */
			}
			else if (index == 17 && ap_role) {
				/* The AP's PHY control block, rewritten once a frame after its
				 * last block went out: written at the frame-control interrupt
				 * (18) it was torn under the next header and grounds rejected
				 * most messages, a TTL-5 identity switch among them. */
				ap_frame_17++;
				phy_control.role = 0; phy_control.mode = 0; phy_control.wake = 1;
				/* The frame as it is now (the short one, 40 MHz), not the
				 * one the search was configured with: command 7's lead on
				 * the AP time was off by up to a millisecond after a switch,
				 * and the host's AP-clock figures with it. */
				phy_control.callbacks = 0x17e; phy_control.frame_us = ap_frame_us;
				/* The frame stamp (when the channel is free): the AP's time at
				 * the IRQ 5 before this block's header. */
				phy_control.stamp_us = ar_frame_stamp_value(&fstamp); phy_control.stamp_period_ms = 100;
				/* And the resend map every 6 frames (12 in each). */
				phy_control.retry_map = ap_retry_map; phy_control.retry_frame = ap_retry_frame; phy_control.retry_every = 6;
				(void)ar_phy_control_tick(&phy_control,timer_ticks()/40000,&phy_ops,NULL);
#ifdef AR_BW_TRACE
				if (ap_trace_17) { ap_trace_item(12 + ap_trace_17, timer_ticks() - ap_trace_switch); if (++ap_trace_17 > 8) { ap_trace_17 = 0; } }
#endif
				if (ap_bw_pending && ap_bw_pending != ap_bw_code && (phy_control.tx[1] & 15) == 5 &&
				    !(phy_control.tx[1] >> 4) && !(phy_control.tx[2] & 1)) {
					/* A ground on stock timing (no stamp yet) moves a frame or
					 * two after us: the first frame on the new width carries
					 * no video either. */
					ap_bw_quiet = ap_bw_aligned(ap_bw_pending) ? 1 : 2;
#ifdef AR_BW_TRACE
					ap_trace_irq17 = timer_ticks(); ap_trace_item(12, fstamp.irq5[0] - fstamp.irq5[1]);
#endif
				}
			}
			else if (index == 11 && dev_active) {
				receiver.tx_end++;
				(void)ar_dev_tx_retire(&dev_tx, 0); receiver.tx_owned = dev_tx.owned;
			}
			else if (index == 12) {
				/* Stock EOP handler 0x2210cc obtains raw user from FBC. */
				receiver.eop_last = read32(BB + 0xfbc);
				receiver.eop_users[(receiver.eop_last >> 20) & 15]++;
				if (ap_role) { ap_up_measure(receiver.eop_last); }
				else if (receive_operation >= 64) { dev_measure_eop(receiver.eop_last); }
				if (receiver.irq_counts[12] <= 4) {
					struct ar_rf_eop_snapshot *snapshot = &receiver.eop_snapshots[receiver.irq_counts[12] - 1];
					snapshot->elapsed_us = elapsed_us();
					snapshot->status = receiver.eop_last;
					snapshot->valid_indications = receiver.fch_locked;
					uint32_t old = read32(BB + 0xbb0);
					field(0xbb00701, 0); snapshot->header = read32(BB + 0xf00);
					field(0xbb00701, (old >> 7) & 1);
					read_control_header(snapshot->control);
				}
			}
			else if (index == 15) { receiver.error_last = read32(BB + 0xf48); }
			else if (index == 5) {
				ar_frame_stamp_irq5(&fstamp, timer_ticks());   /* first: the stamps' moment */
				/* The AP's receive path can sit 0, ~677 or ~1415 us behind
				 * its slot (where RX was last released: a sync on noise late
				 * in the slot runs on past the slot end and leaves it there).
				 * A ground's uplink then looks late: its short reports still
				 * fit, longer packets run past the slot and the decoder
				 * stalls. Re-armed at the end of the sweep, just before the
				 * receive slot, it lines up again. */
				if (ap_role) {
					/* 0xfa8 bit 6 set: the receive path halted (no sync at all,
					 * even with a ground sending); a re-arm brings it back. At
					 * most every 16th sweep: it is also set while nothing came. */
					if (ap_late_now) { ap_info.late_frames++; }
					ap_late_run = ap_late_now ? ap_late_run + 1 : 0; ap_late_now = 0;
					if (ap_late_run >= AP_LATE_FRAMES) { ap_late_run = 0; ap_rx_late = 1; ap_info.rearm_late++; }
					/* ... and only when no uplink packet came since the last look. */
					if ((read32(BB + 0xfa8) & 0x40) && !(++ap_sweeps & 15)) {
						ap_info.halted_seen++;
						if (ap_info.uplink_packets == ap_halted_packets) { ap_rx_late = 1; ap_info.rearm_halted++; }
						ap_halted_packets = ap_info.uplink_packets;
					}
					ap_up_tick();
					if (ap_trigger_pending) { ap_place_trigger(); }
					if (ap_chan_at_sweep) {
						unsigned index = ap_chan_at_sweep & 0xffff;
						ap_chan_at_sweep = 0; ap_retune(index);
					}
					/* A width switch: both ends move at the IRQ 5 of the frame
					 * whose header carried TTL 1 (our command 5 is then at TTL
					 * 0, the ground has decoded it), so both run the new width
					 * from the next one (across 40 MHz, a new frame too). */
#ifdef AR_BW_TRACE
					if (ap_trace_i7_frames && !--ap_trace_i7_frames) {
						ap_trace_raw(34, (uint16_t)(ap_trace_i7_user8 | (ap_trace_i7_all > 255 ? 255 : ap_trace_i7_all) << 8));
						ap_trace_raw(39, (uint16_t)ap_trace_i7_preset);
					}
					if (ap_trace_after) {
						ap_trace_item(3 + ap_trace_after, fstamp.irq5[0] - ap_trace_switch);
						if (++ap_trace_after > 8) { ap_trace_after = 0; }
					}
#endif
					{
						unsigned connected = ap_link.state == AR_AP_CONNECTED, action = 0;
#ifdef AR_BW_TRACE
						if (ap_tf_since < 15) { ap_tf_since++; }
						uint16_t v[3] = {
							(uint16_t)((ap_frame_17 > 3 ? 3 : ap_frame_17) | (ap_tf_7u8 > 3 ? 3 : ap_tf_7u8) << 2 |
								(ap_tf_11 > 7 ? 7 : ap_tf_11) << 4 | (ap_tf_18u8 > 7 ? 7 : ap_tf_18u8) << 7 |
								(ap_tf_13 > 7 ? 7 : ap_tf_13) << 10 | (read32(BB + 0xb20) >> 30 & 1) << 13 |
								((read32(BB + 0xf70) & 0xc0) == 0xc0) << 14 | (read32(BB + 0xa48) != 0) << 15),
							(uint16_t)((ap_tf_gap / 8 > 255 ? 255 : ap_tf_gap / 8) | (ap_retx.repeated > 7 ? 7 : ap_retx.repeated) << 8 |
								(ap_retx.phase & 3) << 11 | (ap_tf_7 > 7 ? 7 : ap_tf_7) << 13),
							(uint16_t)((ap_tf_req & 0xff) | (ap_retx.last_id & 15) << 8 | ap_tf_since << 12)};
						if (connected && !ap_frame_17 && !ap_tf_after) {
							for (unsigned i = 0; i < 4; i++) {
								const uint16_t *w = ap_tf_ring[(ap_tf_head + i) & 3];
								for (unsigned j = 0; j < 3; j++) { ap_trace_raw(40 + j, w[j]); }
							}
							ap_tf_after = 7;
						}
						if (ap_tf_after) { for (unsigned j = 0; j < 3; j++) { ap_trace_raw(40 + j, v[j]); } ap_tf_after--; }
						if (ap_tf_since >= 1 && ap_tf_since <= 4) { for (unsigned j = 0; j < 3; j++) { ap_trace_raw(43 + j, v[j]); } }
						memcpy(ap_tf_ring[ap_tf_head], v, sizeof v); ap_tf_head = (ap_tf_head + 1) & 3;
						ap_tf_7 = ap_tf_7u8 = ap_tf_11 = ap_tf_18u8 = ap_tf_13 = ap_tf_gap = 0; ap_tf_req = 0xff;
#endif
						/* The resend map: this frame's blocks sent again, or 3 with
						 * a ground and no HARQ request at all (nothing confirmed). */
						{
							unsigned v = ap_frame_resent > 2 ? 2 : ap_frame_resent;
							if (connected && !ap_frame_fed) { v = 3; }
							ap_retry_map = ap_retry_map << 2 | v; ap_retry_frame++;
							ap_frame_resent = 0; ap_frame_fed = 0;
						}
						/* The stall guard (ap_frame_17 above). */
						if (connected && !ap_frame_17) {
							ap_stall_frames++;
							if (ap_stall_frames == 2 && !ap_stall_off && !ap_retx.phase && !(read32(BB + 0xb20) >> 30 & 1)) {
								field(0xb201e01, 1); ap_stall_off = 2; ap_info.retx_disables++; action = 1;
							} else if (ap_stall_frames == 4) { (void)ap_rearm_tx(); action = 2; }
						} else { ap_stall_frames = 0; }
						if (ap_stall_off && !--ap_stall_off && connected) { field(0xb201e01, 0); ap_info.retx_enables++; action = 3; }
						ap_frame_17 = 0;
#ifdef AR_BW_TRACE
						if (action) { ap_trace_raw(46, (uint16_t)(action | ap_tf_since << 8)); }
#else
						(void)action;
#endif
					}
					if (ap_bw_quiet) { ap_bw_quiet--; }
					if (ap_bw_pending && ap_bw_pending != ap_bw_code && !ap_bw_prequiet && (phy_control.tx[1] & 15) == 5 &&
					    (phy_control.tx[1] >> 4) == 1 && !(phy_control.tx[2] & 1)) { ap_bw_prequiet = 1; }
					if (ap_bw_pending &&
					    ap_link.state == AR_AP_CONNECTED && (phy_control.tx[1] & 15) == 5 &&
					    !(phy_control.tx[1] >> 4) && !(phy_control.tx[2] & 1)) {
						ap_bw_at_sweep = 0x80 | ap_bw_pending;
					}
					/* Each of these holds and flushes the engines. Not with a
					 * receive DMA in flight: the flush can take its completion
					 * with it, and a completion lost leaves the receive queue
					 * one slot ahead of the chip for good (the AP has no
					 * recovery path). A packet late in the frame is exactly
					 * that case: the switch waits for the next IRQ 5 - except
					 * where the ground switches at this one (across 40 MHz, or
					 * a ground with our stamps), where the packet is finished
					 * first (a frame's wait there is a different frame from the
					 * ground's). */
					if (ap_bw_at_sweep && ap_bw_aligned(ap_bw_at_sweep & 7)) {
						(void)receive_settle(RECEIVE_SETTLE_US);
					}
					if (receive_pending) { ap_info.rx_pending = receive_pending; }
					else {
						if (ap_bw_at_sweep) {
							unsigned code = ap_bw_at_sweep & 7;
							ap_bw_at_sweep = 0;
							if (code != ap_bw_code) { ap_apply_bandwidth(code); }
							else { ap_bw_prequiet = 0; ap_bw_quiet = 0; }   /* nothing to switch */
						}
						if (ap_frame_at_sweep) {
							unsigned mode = ap_frame_at_sweep & 7;
							ap_frame_at_sweep = 0; ap_apply_frame_mode(mode);
						}
						else if (ap_rx_late || ap_rearm_pending == 3) {
							ap_rx_late = 0; ap_rearm_pending = 0;
							ap_info.decoder_reset_result = (uint32_t)ap_rearm_rx(); ap_info.decoder_resets++;
						}
					}
					ap_sweep_us = elapsed_us();
				} else {
#ifdef AR_BW_TOGGLE_MS
					/* Bench: the IRQ 5s since the last header that carried
					 * the AP's command 5, up to the switch (bits 7:5). */
					if (bw_heard && bw_sweeps_left < 7) { bw_sweeps_left++; }
					{
						uint32_t now_us = elapsed_us();
						if (bw_heard && now_us - bw_irq5_us > bw_irq5_gap) { bw_irq5_gap = now_us - bw_irq5_us; }
						bw_irq5_us = now_us;
						if (bw_trace_i5) {
							uint32_t at = (now_us - bw_switch_irq5_us) / 8;
							const uint8_t mark[] = {0xb9, (uint8_t)(3 - bw_trace_i5), (uint8_t)at, (uint8_t)(at >> 8)};
							publish_event(2, mark, sizeof mark); bw_trace_i5--;
						}
					}
#endif
					if (bw_due_code && fstamp.irq5[0] >= bw_due_ticks) {
						if ((bw_due_code & 7) != bw_code) { bw_at_sweep = bw_due_code; }
						bw_due_code = 0;
					}
					if (bw_at_sweep) {
						unsigned code = bw_at_sweep & 7;
						unsigned aligned = (bw_at_sweep & 0x40) || code == 5 || bw_code == 5;
						/* At the AP's IRQ 5 the receive is finished first, as on the AP. */
						if (aligned) {
#ifdef AR_BW_TOGGLE_MS
							uint32_t taken = receiver.irq_counts[13] + receiver.irq_counts[16];
#endif
							(void)receive_settle(RECEIVE_SETTLE_US);
#ifdef AR_BW_TOGGLE_MS
							if (receiver.irq_counts[13] + receiver.irq_counts[16] != taken) { bw_path |= 4; }
#endif
						}
						/* The same guard as the recovery reset's: not over a DMA. */
						if (receive_pending) {
							recovery.reset_deferred++;
#ifdef AR_BW_TOGGLE_MS
							bw_path |= 16;
#endif
						} else {
							bw_at_sweep = 0;
							if (code != bw_code) { apply_bandwidth(code, aligned); }
						}
					}
				}
				/* Stock IRQ5 (0x21f804) byte-swaps E70 before PHY message
				 * 0x100. Preserve the raw numeric contract; units unknown. */
				uint32_t power = __builtin_bswap32(read32(BB + 0xe70));
				receiver.power_last = power;
				if (!receiver.power_samples || power < receiver.power_min) { receiver.power_min = power; }
				if (!receiver.power_samples || power > receiver.power_max) { receiver.power_max = power; }
				receiver.power_samples++;
				unsigned sample_channel = receiver.channel_index;
				if (receive_operation == 76) {
					sample_channel = receiver.channel_count;
					if (scan_info.phase == 2 && scan_info.channel < scan_info.count) {
						scan_info.samples++; scan_info.channel_samples[scan_info.channel]++;
						int score_action = ar_scan_score_feed(&scan_scores,scan_history,scan_rows,NULL,
							scan_info.count,scan_info.channel,power);
						if (score_action < 0) { scan_info.errors++; }
						else {
							scan_info.score_ring = scan_scores.ring; scan_info.score_ready = scan_scores.ready;
							scan_info.score_rounds += !!(score_action & AR_SCAN_SCORE_ROUND);
							scan_info.channel_scores[scan_info.channel] = scan_history[scan_info.channel].score;
						}
						for (unsigned i = 0; i < receiver.channel_count; i++) {
							if (receiver.channels[i].frequency_khz == scan_info.frequency_khz) { sample_channel = i; break; }
						}
					} else { scan_info.dropped++; }
					scan_info.phase = 0;
				}
				if (sample_channel < receiver.channel_count) {
					struct ar_rf_channel_info *channel = &receiver.channels[sample_channel];
					channel->power_last = power;
					if (!channel->power_samples || power < channel->power_min) { channel->power_min = power; }
					if (!channel->power_samples || power > channel->power_max) { channel->power_max = power; }
					channel->power_samples++;
				}
			}
			else if (index == 2) {
				/* Stock 0x228066 selects behavior from E6C's raw user.
				 * This is sampled context: polling can lag the IRQ edge. */
				uint32_t command = read32(BB + 0xe6c);
				if (receiver.irq_counts[2] == 1) { receiver.source2_command_first = command; }
				receiver.source2_command_last = command;
				receiver.source2_users[(command >> 16) & 15]++;
				receiver.source2_modes[(command >> 20) & 3]++;
				/* Stock 0x229a56: on the (E6C >> 16) & 0xd == 0xc event it
				 * selects bank 7 of C00 with bit 0 clear and reads the four
				 * per-chain SNR halfwords at EF0/EF4 (the "snr_chan" values
				 * that feed its MCS policy through message 0x101). */
				if (((command >> 16) & 0xd) == 0xc) {
					uint32_t old = read32(BB + 0xc00);
					field(0xc000701, 1); field(0xc000001, 0);
					receiver.snr_words[0] = read32(BB + 0xef0);
					receiver.snr_words[1] = read32(BB + 0xef4);
					field(0xc000701, (old >> 7) & 1); field(0xc000001, old & 1);
					chain_snr_fresh = 1;
				}
			}
			else if (index == 14) {
				if (frame_pending) { unsigned m = frame_pending & 7; frame_pending = 0; apply_frame_mode(m); }
				if (information.phase != AR_RF_RECEIVING) { return; }
				uint32_t old = read32(BB + 0xbb0);
				field(0xbb00701, 0); receiver.fch_last = read32(BB + 0xf00);
				field(0xbb00701, (old >> 7) & 1);
				if (TRACE_ON && !ap_trace.frozen && ap_trace.head) {
					ap_trace.entries[(ap_trace.head - 1) % AR_RF_AP_TRACE_ENTRIES][5] = receiver.fch_last;
				}
				/* A stalled AP decoder reads every header invalid (c0000000)
				 * while the ground keeps sending: re-arm RX at the next sweep. */
				if (ap_role) {
					if (receiver.fch_last & 0x80) { ap_fch_bad = 0; }
					else if (++ap_fch_bad >= 8) { ap_fch_bad = 0; ap_rx_late = 1; ap_info.rearm_fch++; }
				}
				unsigned user = (receiver.fch_last >> 9) & 15;
				if (automatic_recovery() && (user == 8 || user == 9)) {
					(void)ar_channel_monitor_observe(&channel_monitor,8,!!(receiver.fch_last & 0x80),
						NULL,0,&monitor_ops,NULL);
				}
				if (bw_code == 5) {
					if (receiver.fch_last & 0x80) { bw_info.wide_ok++; bw_info.wide_last_ok = receiver.fch_last; }
					else { bw_info.wide_bad++; bw_info.wide_last_bad = receiver.fch_last; }
				}
				if (receive_operation >= 64 && (user == 8 || user == 9)) {
					/* 0x227b96 -> 0x21acb2 aliases both raw users to logical8. */
					phy_header_valid[8] = !!(receiver.fch_last & 0x80); phy_eop_seen[8] = 0;
					phy.last_fch = receiver.fch_last;
				}
#ifdef AR_BW_TOGGLE_MS
				if (bw_heard && !bw_settle_step) {
					if (user != 8 && user != 9) { bw_pre_other++; }
					else if (receiver.fch_last & 0x80) { bw_pre_valid++; } else { bw_pre_invalid++; }
				}
				if (bw_trace_left && (user == 8 || user == 9)) {
					uint32_t at = (elapsed_us() - bw_switch_irq5_us) / 8;
					const uint8_t mark[] = {0xb7, (uint8_t)((6 - bw_trace_left) | (receiver.fch_last & 0x80 ? 16 : 0)),
						(uint8_t)at, (uint8_t)(at >> 8)};
					publish_event(2, mark, sizeof mark); bw_trace_left--;
				}
#endif
				/* A width switch: the new width's valid headers bring the
				 * receiver back. Not one polled with the switch's IRQ 5 (the
				 * old frame's, late): the new frame's comes 1-3 ms after. */
				if (bw_settle_step && (user == 8 || user == 9) && (receiver.fch_last & 0x80) &&
				    elapsed_us() - bw_settle_us > 200) {
					bandwidth_settle(1);
				}
				receiver.header_users[user]++;
				if (receiver.fch_last & 0x80) {
					if (!receiver.fch_locked) {
						receiver.first_valid_header = receiver.fch_last;
						receiver.first_valid_command = read32(BB + 0xe6c);
						receiver.first_valid_elapsed_us = elapsed_us();
						/* Stock control-header reader 0x22d30a uses this bank.
						 * Retain raw words for independent content/checksum validation. */
						read_control_header(receiver.first_valid_control);
						/* Compare bank selection and a delayed read before assuming
						 * zero control content is a decoder result. No acceptance
						 * decision is made from these diagnostic samples. */
						receiver.first_selector_old = read32(BB + 0xbb0);
						field(0xbb00701, 0);
						read_control_words(receiver.first_control_bank0);
						field(0xbb00701, 1);
						receiver.first_selector_set = read32(BB + 0xbb0);
						receiver.first_bank_delay_status = (uint32_t)delay_us(1);
						receiver.first_selector_delayed = read32(BB + 0xbb0);
						read_control_words(receiver.first_control_delayed);
						field(0xbb00701, (old >> 7) & 1);
					}
					if (receive_operation >= 51) {
						receiver.last_control_header = receiver.fch_last;
						read_control_header(receiver.last_control_words);
						/* 0x22d30a extracts seven bytes starting at raw bit 28.
						 * All-zero bytes fail 0x2299d2 (computed 0x1e).
						 * This rejects only that measured pattern; nonzero
						 * content is not thereby accepted as a MAC header. */
						if (!(receiver.last_control_words[0] >> 28) &&
						    !receiver.last_control_words[1] &&
						    !(receiver.last_control_words[2] & 0xfffff)) {
							receiver.empty_control_headers++;
						}
						if (receive_operation >= 52) {
							uint8_t control[7];
							for (unsigned i = 0; i < 7; i++) {
								unsigned bit = 28 + i * 8, word = bit / 32, shift = bit % 32;
								uint32_t value = receiver.last_control_words[word] >> shift;
								if (shift > 24) { value |= receiver.last_control_words[word + 1] << (32 - shift); }
								control[i] = (uint8_t)value;
							}
							receiver.last_control_check = ar_rf_control_checksum(control);
							if (receiver.last_control_check == control[0]) {
								receiver.matching_control_headers++;
								if (user == 8 || user == 9) { capture_phy(control, receiver.fch_last); }
								if (normal_search && !search_candidate_seen && (user == 8 || user == 9)) {
									search_candidate_seen = 1; search_candidate_started = read32(TIMER);
								}
								if (listener_follow && user == 8) { listener_control(control); }
                                else if (receive_operation >= 69 && (user == 8 || user == 9)) { sync_receive(control); }
								if (ap_role && user == 0) { ap_phy_received(control); }
							}
							else {
								receiver.bad_control_headers++;
								if ((user == 8 || user == 9) && !(receiver.bad_control_headers & 15)) {
									uint8_t bad[12]; memcpy(bad, control, 7); bad[7] = receiver.last_control_check;
									memcpy(bad + 8, &receiver.last_control_words[0], 4);
									capture_add(5, bad, 12, receiver.last_control_words[1]);
								}
							}
						}
					}
					receiver.fch_locked++; receiver.header_valid_users[user]++;
					/* Stock 0x228172's key 1 maps to internal MCS index 1.
					 * Diagnostic 50 forces only 0x222c0a's two MMIO effects
					 * once for that observed user/MCS. Stock's pending-MCS
					 * software conditions and host notification are not ported.
					 * No packet or MAC-acceptance count is synthesized. */
					uint32_t mode = (receiver.fch_last >> 4) & 3;
					uint32_t key = ((receiver.fch_last >> 13) & 7) << 16 |
						((receiver.fch_last & 15) % 3) << 8 |
						(mode == 3 ? UINT32_C(1) << 24 : mode);
					if (receive_operation >= 67 && (user == 8 || user == 9)) {
						unsigned mcs = 0;
						while (mcs < 25 && mcs_fch_keys[mcs] != key) { mcs++; }
						mcs_info.last_header = receiver.fch_last;
						if (mcs < 25) {
							mcs_info.observed_counts[mcs]++;
							if (mcs_info.first_observed_mcs == 25) { mcs_info.first_observed_mcs = mcs; }
							dev_mcs_apply(ar_dev_mcs_observe(dev_mcs_context(), mcs));
						} else { mcs_info.unknown_headers++; }
					}
					else if (ap_role && user == 0) {
						unsigned mcs = 0;
						while (mcs < 25 && mcs_fch_keys[mcs] != key) { mcs++; }
						if (mcs < 25) { ap_up_observe(mcs); }
					}
					/* Operation65 tests the same effects after native login,
					 * at the stock-linked user8/key1 observation. It does not
					 * implement the pending-MCS policy that selects this call. */
					unsigned linked_buffer_probe = receive_operation == 65 && dev_active &&
						receiver.dev_state == 2 && receiver.assigned_slot == 0;
					if ((receive_operation == 50 || linked_buffer_probe) && user == 8 && key == 1 &&
					    !receiver.mcs_buffer_transitions) {
						receiver.mcs_buffer_header = receiver.fch_last;
						receiver.mcs_buffer_bf0_before = read32(BB + 0xbf0);
						receiver.mcs_buffer_d08_before = read32(BB + 0xd08);
						/* DEV mode 0 maps logical user 8 to buffer user 0. */
						field(0xbf00002, 1);
						field(0xd081001, 0);
						receiver.mcs_buffer_bf0_after = read32(BB + 0xbf0);
						receiver.mcs_buffer_d08_after = read32(BB + 0xd08);
						receiver.mcs_buffer_transitions = 1;
					}
				}
			}
			else if (index == 7) {
				receiver.command_last = read32(BB + 0xe6c);
				/* 0xa48 lasts one TX entry (it reads 0 after): stock writes it at
				 * every IRQ 7, for the next entry's bandwidth. At 40 MHz the
				 * broadcast (user 8) stalls after its first frame without it; at
				 * 20 a switch from 5 MHz lost the link every time (bench,
				 * 2026-10-19). The AP's only TX entries are the broadcast's, so it
				 * is written at every IRQ 7 (and TX end, IRQ 11): written only at
				 * the IRQ 7 that read user 8, one handled late or merged left a
				 * broadcast without it - the ground heard the first frame after a
				 * switch to 40 MHz and nothing after, the link lost (bench,
				 * 2026-10-26, 1 switch up in ~80, both frame modes). */
#ifdef AR_BW_TRACE
				if (ap_role && ap_bw_code >= 4 && ((receiver.command_last >> 16) & 15) == 8 && ap_trace_i7_frames) {
					ap_trace_i7_user8++;
					if (read32(BB + 0xa48)) { ap_trace_i7_preset += (receiver.command_last >> 20) & 15 ? 0x100 : 1; }   /* still set: low byte e6c 0x0008, high 0x0028 */
				}
#endif
				if (ap_role && ap_bw_code >= 4) {
					write32(BB + 0xa48, ap_bw_code == 5 ? 0x80808080 : 0x40404040);
				}
#ifdef AR_BW_TRACE
				if (ap_role) { ap_tf_7++; if (((receiver.command_last >> 16) & 15) == 8) { ap_tf_7u8++; } }
				if (ap_role && ap_trace_i7_frames) {
					ap_trace_i7_all++;
					if (ap_trace_i7_seen < 4) { ap_trace_raw(35 + ap_trace_i7_seen, (uint16_t)((receiver.command_last >> 16) & 0xffff)); ap_trace_i7_seen++; }
				}
#endif
				receiver.command_modes[(receiver.command_last >> 20) & 3]++;
				if (receive_operation == 76 && commands_ready) {
					command_prepare(receiver.command_last);
				} else if (((receiver.command_last >> 20) & 3) == 0) {
					/* Historical diagnostic modes keep their bounded setup. */
					static const uint16_t offsets[] = {9, 0x5b, 0x83, 0x85, 0x58, 0x5e};
					static const uint8_t values[] = {0x0e, 3, 8, 0x11, 0, 0};
					for (unsigned byte = 0; byte < 6; byte++) {
						unsigned offset = offsets[byte], shift = (offset & 3) * 8;
						update(BB + (offset & ~3u), 255u << shift, (uint32_t)values[byte] << shift);
					}
					/* Compensation is loaded for transmit-direction commands,
					 * not globally while setting up receive-only operation. */
					if (receive_operation == 1 || receive_operation == 3 || receive_operation == 4 ||
					    (dev_active && command_state.cached_bank == 3 && ((receiver.command_last >> 16) & 15) == 0)) {
						if (dev_active) {
							field(0x4800101, 1);
							field(0x4a01008, 0xff); field(0x4a80008, 0xff);
						}
						field(0x4a00008, calibration[0]); field(0x4a00808, calibration[1]);
						field(0x4a41008, calibration[2]); field(0x4a41808, calibration[3]);
						field(0x48c0008, calibration[4] | (read32(BB + 0x48c) & 0x33));
						if (dev_active) { command_state.cached_bank = 2; }
					}
				}
			}
			if (tracing) { phy_trace_finish(&trace, index); }
		}
	receive_iterations++;
	uint32_t now = read32(TIMER);
	uint64_t elapsed = elapsed_ticks();
	unsigned expired = receive_milliseconds && (elapsed >= (uint64_t)receive_milliseconds * 40000 || receive_iterations >= 1000000);
	if (receive_operation >= 69 && !expired) { sync_poll((uint32_t)(elapsed / 40000)); }
    if (listener_follow && !expired) { listener_poll(); }
	/* An AP never searches: the retune below holds and releases both engines
	 * and clears 0xbf0's buffer map, which leaves the AP's receive path at a
	 * random phase and its broadcast HARQ unusable. */
	if ((receive_operation == 36 || receive_operation == 37 || receive_operation >= 39) && !expired && !ap_role) {
		/* Bounded DEV message 0x205, with a one-entry or supplied channel
		 * list; stock alternates floor(frame_us/1000)+2 and
		 * 2*(floor(frame_us/1000)+1) at list wrap (0x22f2e2..0x22f300).
		 * Header/DMA evidence suspends retries so this diagnostic cannot reset
		 * a candidate packet. Operation 51 excludes only known-invalid empty
		 * control headers; operations 52/53 exclude checksum failures. Neither
		 * checksum equality nor this latch implements stock linked state. */
		if (normal_search) {
			/* A valid header may precede its payload/DMA interrupt. Give the
			 * first candidate on each visit two frames to reach admission.
			 * Repeated unrelated headers cannot extend this native grace window.
			 * DMA ownership and an admitted transition always prevent retuning. */
			unsigned grace = search_candidate_seen &&
				(uint32_t)(now - search_candidate_started) < 2 * receiver.search_frame_us * 40;
			receiver.search_suspended = dev_active || receiver.acquisition_ready || receive_pending || grace;
		} else {
			unsigned candidate_headers = receiver.fch_locked;
			if (receive_operation == 51) { candidate_headers -= receiver.empty_control_headers; }
			if (receive_operation >= 52) { candidate_headers -= receiver.bad_control_headers; }
			if (candidate_headers || receiver.receive_events || receive_pending) {
				receiver.search_suspended = 1;
			}
		}
		if (!receiver.search_suspended &&
		    (uint32_t)(now - search_started) >= receiver.search_dwell_ms * 40000) {
			unsigned next = 0, wrap = 1;
			if (receiver.channel_count) {
				next = receiver.channel_index + 1;
				wrap = next == receiver.channel_count;
				if (wrap) { next = 0; }
				search_pll = channel_plls[next];
			}
			if (wrap && receive_operation != 40) {
				uint32_t short_dwell = receiver.search_frame_us / 1000 + 2;
				receiver.search_dwell_ms = receiver.search_dwell_ms == short_dwell ?
					2 * (receiver.search_frame_us / 1000 + 1) : short_dwell;
			}
			int result;
			if (receiver.channel_count) {
				/* Keep both engines reset during retuning and pending power
				 * acknowledgment. Do not retune if the hold times out. This
				 * diagnostic uses a stronger boundary than stock's live tune. */
				result = hold_engines();
				if (!result) {
					irq_ack(5); irq_ack(7);
					if (normal_search) { tune_context_plls(search_pll, receiver.channels[next].frequency_khz); }
					else { serial_word(receive_operation == 40 ? 0x90a : 0x905, search_pll); }
					if (receive_operation != 40) {
						field(0xa900010, receiver.channels[next].frequency_khz / 2000);
					}
					field(0xbf00002, 0);
					release_engines();
				}
			} else {
				serial_word(0x905, search_pll);
				field(0xbf00002, 0);
				result = idle();
			}
			if (result) {
				receiver.search_error = (uint32_t)result;
				failure(result);
				expired = 1;
			} else {
				receiver.search_restarts++;
				if (receiver.channel_count) {
					receiver.channel_index = next;
					if (receive_operation >= 54) { dev_config.receive_channel = next; dev_transmit_channel = next; }
					receiver.channel_wraps += wrap;
					receiver.frequency_khz = receiver.channels[next].frequency_khz;
					if (commands_ready) {
						const unsigned users[] = {8,0};
						for (unsigned i = 0; i < 2; i++) {
							unsigned direction = i == 0;
							(void)ar_command_update(&command_program,1,users[i],direction,3,receiver.frequency_khz,0);
							/* Normal search validates every channel as band 2. */
							(void)ar_command_update(&command_program,1,users[i],direction,1,2,0);
						}
					}
					receiver.channels[next].visits++;
				}
				search_candidate_seen = 0;
				memset(phy_header_valid, 0, sizeof phy_header_valid);
				memset(phy_eop_seen, 0, sizeof phy_eop_seen);
			}
			/* No catch-up reset bursts after a delayed cooperative poll. */
			search_started = read32(TIMER);
		}
	}
	if (expired) { stop_receive(); }
	else if (automatic_recovery()) {
		monitor_poll(timer_ticks()/40000);
		if (recovery.pending) { recover_host(); return; }
		if (recovery_reset_pending) {
			if (receive_pending) { recovery.reset_deferred++; }
			else {
				field(0x8000101, 1); field(0x80c0201, 1);
				int result = wait_bits(BB + 0xfa8, 0x100, 0x100, 2000);
				field(0x80c0201, 0);
				if (result) {
					recovery.failures++; recovery.last_error = (uint32_t)result;
					stop_receive(); (void)failure(result); return;
				}
				field(0x8000101, 0); recovery.rx_resets++; recovery_reset_pending = 0;
			}
		}
	}
}

static void stop_receive(void)
{
	if (receive_operation >= 54) { field(0x8000201, 1); dev_active = 0; }
	field(0x8000001, 1);
	link_event(0, receiver.dev_state);
	receiver.monitor_elapsed_us = elapsed_us();
	information.phase = AR_RF_PSRAM;
	progress(0xa006, receiver.irq_total, receiver.packets, receiver.bytes);
	static const uint16_t offsets[16] = {0x800,0x808,0x810,0x814,0x870,0x8a4,
		0x9e0,0x9e4,0xa60,0xb20,0xbb0,0xc00,0xcd0,0xde0,0xde4,0xfa8};
	for (unsigned i = 0; i < 16; i++) { receiver.context_words[i] = read32(BB + offsets[i]); }
	receiver.decoder_controls[0] = read32(BB + 0xbac);
	receiver.decoder_controls[1] = read32(BB + 0xbd4);
	static const uint16_t clock_offsets[] = {0,4,8,12,16,20,0x80,0x200};
	for (unsigned i = 0; i < 8; i++) { receiver.clock_controls[i] = read32(CLOCK + clock_offsets[i]); }
	static const uint16_t gpio_offsets[] = {0,4,8,12,16,0x2c,0x30,0x44};
	for (unsigned i = 0; i < 8; i++) { receiver.gpio_controls[i] = read32(0x0a10a000 + gpio_offsets[i]); }
	sync_info.pending = 0; host_prepared = 0;
}

int ar_rf_stop_host(void)
{
	if ((receive_operation != 76 && !listener_capture) ||
	    (information.phase != AR_RF_RECEIVING && information.phase != AR_RF_PSRAM)) { return -11; }
	if (information.phase == AR_RF_RECEIVING) {
		/* Stop submissions, then drain an outstanding DMA before resetting
		 * its producer engine. Resetting RX first can remove the completion
		 * we need to establish ownership. No new DMA is submitted while this
		 * cooperative call runs. On timeout retain storage and refuse reuse. */
		field(0x8000201, 1);
		recovery.stop_stage = 1; recovery.stop_pending_rx = receive_pending;
		int result = 0;
		if (receive_pending) {
			result = wait_bits(0xe0801000u + irq_vector(13) * 4, 1, 1, 2000);
			if (!result) { irq_ack(13); receive_pending = 0; lifecycle.cancelled_rx++; }
		}
		if (!result) { recovery.stop_stage = 2; result = hold_engines(); }
		recovery.stop_fa8 = read32(BB + 0xfa8);
		recovery.stop_irq13 = read8(0xe0801000u + irq_vector(13) * 4);
		if (!result) { recovery.stop_stage = 3; }
		lifecycle.last_change_ms = elapsed_ms();
		stop_receive();
		if (result) { lifecycle.last_error = (uint32_t)result; return failure(result); }
		lifecycle.cancelled_tx += dev_tx.owned;
		ar_dev_tx_init(&dev_tx); receiver.tx_owned = 0;
		if (receive_operation >= 67) { dev_mcs_apply(ar_dev_mcs_connect(dev_mcs_context(), 0)); }
		receiver.dev_state = 0; receiver.assigned_slot = 8; receiver.acquisition_ready = 0;
		field(0xcf41501, 0); receiver.sync_ack_pending = 0;
		lifecycle.stops++;
	}
	host_prepared = 0; host_start_required = 1;
	channel_monitor.users[8].flags &= (uint8_t)~1; recovery.pending = 0; recovery_reset_pending = 0;
	return 0;
}

static int prepare_saved_host(void)
{
	/* configure_monitor resets native protocol/PHY state, but the original
	 * host's current candidate list belongs to configuration, not that state. */
	uint8_t candidates[AR_DEV_MAX_CANDIDATES][4];
	size_t count = dev_config.candidate_count;
	struct ar_ap_config saved_ap = ap_link.config;
	memcpy(candidates, dev_candidates, count * 4);
	int result = configure_monitor(host_config.frequency, host_config.milliseconds,
		host_config.identity, 76, host_config.channels, host_config.length, host_config.deferred,
		host_config.has_power ? &host_config.power : NULL);
	if (!result) {
		memcpy(dev_candidates, candidates, count * 4); dev_config.candidate_count = count;
		dev_config.pairing = binding.enabled;
		memcpy(dev_config.pairing_skip, binding.skip, sizeof binding.skip);
		if (ap_role) {
			/* AP preparation creates an empty admission list. Preserve the
			 * host's grounds (including a just-bound peer) across restart,
			 * but advertise the channel the rebuilt radio actually uses. */
			saved_ap.channel = ap_link.config.channel;
			saved_ap.pairing = (uint8_t)binding.enabled;
			ar_ap_configure(&ap_link, &saved_ap);
		} else if (binding.enabled) { binding_identity(); }
	}
	return result;
}
static void recover_host(void)
{
	unsigned reason = recovery.pending;
	recovery.attempts++; recovery.last_reason = reason; recovery.last_ms = elapsed_ms();
	int result = ar_rf_stop_host();
	if (!result) { result = prepare_saved_host(); }
	if (!result) {
		host_prepared = 0; host_start_required = 0;
		result = start_prepared_receive();
	}
	recovery.pending = 0;
	if (result) { recovery.failures++; recovery.last_error = (uint32_t)result; }
	else { recovery.completed++; }
}
const struct ar_rf_recovery_info *ar_rf_recovery_information(void)
{
	recovery.monitor_state = channel_monitor.users[8].flags & 7;
	recovery.monitor_missed = channel_monitor.users[8].missed;
	recovery.monitor_threshold = channel_monitor.users[8].threshold;
	recovery.monitor_interval_ms = monitor_interval_ms;
	return &recovery;
}
int ar_rf_host_control(unsigned operation, size_t payload_length)
{
	if (payload_length) { return -22; }
	if (operation > 3) { return -95; }
	/* A live listener can be stopped by its host owner, but never started
	 * through the normal association path. */
	if (listener_capture && receive_operation == 53 && operation == 1) { return ar_rf_stop_host(); }
	/* An explicit INIT after a stopped listener restores the saved pilot
	 * configuration. START alone must never turn a listener into a transmitter. */
	if (listener_capture && receive_operation == 53 && operation == 2 &&
	    information.phase != AR_RF_RECEIVING && host_config.valid) {
		int result = prepare_saved_host();
		if (result) { return result; }
		host_initialized = 1; host_start_required = 1; lifecycle.initializations++;
		return 0;
	}
	if (!host_config.valid || receive_operation != 76) { return -7; }
	if (information.error) { return (int32_t)information.error; }
	if (operation == 0) {
		if (!host_initialized || information.phase == AR_RF_RECEIVING) { return -7; }
		int result = host_prepared ? 0 : prepare_saved_host();
		if (result) { return result; }
		host_prepared = 0; host_start_required = 0;
		return start_prepared_receive();
	}
	if (operation == 1) {
		if (!host_initialized || information.phase != AR_RF_RECEIVING) { return -7; }
		return ar_rf_stop_host();
	}
	if (operation == 2) {
		if (host_initialized) { return -7; }
		int result = prepare_saved_host(); if (result) { return result; }
		/* Original sync-object creation in 0x22de32 clears time validity.
		 * Stop/start and link loss retain the offset; deinit/init recreates it.
		 * Do not bind AP time to receive_started, which changes on restart. */
		phy_control.time_value = 0; phy_control.time_valid = 0;
		host_initialized = 1; host_start_required = 1; lifecycle.initializations++;
		return 0;
	}
	if (!host_initialized || information.phase == AR_RF_RECEIVING) { return -7; }
	host_initialized = host_prepared = 0; host_start_required = 1; lifecycle.deinitializations++;
	memset(&binding, 0, sizeof binding);
	return 0;
}
/* SET 22 BB_SET_BANDWIDTH {slot, direction, code}: direction 1 (receive)
 * asks the AP for that broadcast width through internal command 9, the way
 * stock's 0x24ed72 does; direction 0 would change this DEV's own uplink
 * width, which is not supported. */
/* The policy's SNRs from the MCS rows: narrow under what the row above the
 * narrow MCS needs to stay (its snr_down), widen while the widen MCS row
 * could be entered (its snr_up; the first row at or above it, else the top). */
static void bw_policy_rows(void)
{
	const struct ar_dev_mcs_config *mcs = &dev_mcs.config;
	if (!mcs->count) { return; }
	unsigned row = 0;
	while (row + 1 < mcs->count && mcs->rows[row].mcs <= bw_policy.narrow_mcs) { row++; }
	bw_policy.narrow_snr = mcs->rows[row].snr_down;
	row = 0;
	while (row + 1 < mcs->count && mcs->rows[row].mcs < bw_widen_mcs) { row++; }
	bw_policy.widen_snr = mcs->rows[row].snr_up;
}
/* The widest the automatic width and a host request may go: 40 MHz (code 5).
 * Every switch, 40 MHz included, waits for the frame both ends move on
 * (bw_at_sweep): across 40 MHz the frame itself changes with the width. */
#define BW_WIDEST 5u
void ar_rf_set_bandwidth_policy(unsigned automatic, unsigned min, unsigned max, unsigned narrow_mcs, unsigned widen_mcs)
{
	if (max < 2 || max > 5 || min < 2 || min > max) { return; }
	if (max > BW_WIDEST) { max = BW_WIDEST; }
	if (min > max) { min = max; }
	bw_automatic = !!automatic; bw_manual = bw_target = 2;
	bw_policy.min = (uint8_t)min; bw_policy.max = (uint8_t)max;
	bw_policy.narrow_mcs = (uint8_t)narrow_mcs; bw_widen_mcs = (uint8_t)widen_mcs;
	bw_policy_rows();
}
int ar_rf_host_bandwidth(const uint8_t *payload, size_t length)
{
	if (!payload || length < 3) { return -22; }
	if (payload[1] != 1 || payload[2] > 5) { return -95; }
	if (!dev_active || receiver.dev_state != 2) { return -7; }
	/* The widest the receiver may run; the policy decides below it. */
	if (payload[2] < 2 || (bw_automatic && payload[2] < bw_policy.min)) { return -22; }
	bw_policy.max = payload[2] > BW_WIDEST ? BW_WIDEST : payload[2]; ar_width_reset(&bw_width);
	apply_ceiling();   /* MCS 10's rows follow the cap */
	if (!bw_automatic) { bw_manual = bw_target = bw_policy.max; }
	if (bw_state != 1) { bw_info.requests = 0; bw_state = 0; }
	return 0;
}
/* SET 38 (0x24f0xx bb_link_retx_ctrl_cfg): the retransmission-request control
 * parameters; byte 0 is the window the watchdog counts repeats over. */
static uint8_t retx_config[8];
/* Host TX power (stock SET 8 -> 0x24fbf2, SET 9, GET 7/8/9). The level is a
 * dBm index into the board calibration (9..39); the planner turns it into
 * the per-user analog gain / digital amplitude fields of both RF paths, the
 * same fields stock writes. SET 9 (automatic power between min and max) is
 * recorded and answered; the closed loop itself, which follows the peer's
 * reports, is not implemented: enabling it applies the maximum. */
static struct ar_power_config power_cfg;
static uint8_t power_ready, power_level[AR_POWER_USERS], power_auto[3];
/* Boards with two calibration groups (one tested air unit: 5850 MHz): stock
 * picks the group from the channel each time it sets a power (0x244412),
 * with per-band path offsets (SET 0x6e {band, offset, path}, 0x267ab5).
 * power_cfg holds the group of the channel in use; a retune reselects. */
static uint8_t power_rows[2][AR_POWER_LEVELS][4], power_groups;
static uint32_t power_band_edge_khz;
static int8_t power_offsets[2][2];
static unsigned power_band;
void ar_rf_set_power_config(const struct ar_power_config *config)
{
	if (!config) { return; }
	power_cfg = *config; power_ready = 1;
	memset(power_level, 0xff, sizeof power_level);
	memcpy(power_rows[0], config->calibration, sizeof power_rows[0]);
	memcpy(power_rows[1], config->calibration, sizeof power_rows[1]);
	for (unsigned band = 0; band < 2; band++) {
		power_offsets[band][0] = config->path_offset[0]; power_offsets[band][1] = config->path_offset[1];
	}
	power_groups = 1; power_band = 0;
}
void ar_rf_set_power_bands(const uint8_t (*high)[4], uint32_t edge_khz)
{
	if (!high || !edge_khz) { return; }
	memcpy(power_rows[1], high, sizeof power_rows[1]);
	power_band_edge_khz = edge_khz; power_groups = 2;
}
static void reapply_power(unsigned user);
/* The group of `frequency`; on a change, its rows and offsets, and the
 * transmitting user's power again. */
static void power_select_band(uint32_t frequency_khz)
{
	unsigned band = power_groups == 2 && frequency_khz >= power_band_edge_khz;
	if (!power_ready || band == power_band) { return; }
	memcpy(power_cfg.calibration, power_rows[band], sizeof power_cfg.calibration);
	power_cfg.path_offset[0] = power_offsets[band][0]; power_cfg.path_offset[1] = power_offsets[band][1];
	power_band = band;
	reapply_power(ap_role ? 8 : 0);
}
static int apply_power(unsigned user, unsigned level)
{
	struct ar_power_plan plan;
	if (!power_ready) { return -11; }
	int result = ar_power_plan(&power_cfg, user, level, &plan);
	if (result) { return result; }
	for (unsigned i = 0; i < plan.count; i++) { field(plan.writes[i].field, plan.writes[i].value); }
	uint8_t before = power_level[user];
	power_level[user] = (uint8_t)level;
	/* Event 14 for the user we transmit on; an AP also announces it. */
	if (user == (ap_role ? 8u : 0u) && before != level) {
		const uint8_t data[] = {0, 1, (uint8_t)level, before != 0xff ? before : 0};
		publish_event(14, data, sizeof data);
		if (ap_role) { ap_pwr_announce = 1; }
	}
	return 0;
}
static void reapply_power(unsigned user)
{
	if (power_ready && user < AR_POWER_USERS && power_level[user] != 0xff) { (void)apply_power(user, power_level[user]); }
}
/* RF path B (PRJ_CMD_RF_PATH_CTRL path 1). The direction is a mask:
 * bit 0 selects TX, bit 1 selects RX. In particular, the ground host's
 * {1,1,0} disables only TX B; RX B and its combining weights must survive.
 * Stock disabling RX also removes B from combining; re-enabling the chain
 * alone does not restore that receive-mode configuration. */
static void rf_path_b(unsigned directions, unsigned enable)
{
	if (directions & 1) { field(0x0000301, enable); }
	if (directions & 2) {
		field(0x0000501, enable);
		if (!enable) {
			field(0xc000101, 0); field(0xc001701, 0);
			field(0xa200701, 1);
			field(0xa2c000c, 0xfff); field(0xa2c100c, 0);
		}
	}
}
/* PRJ_CMD_SET_GPIO (stock prj_rpc_set_gpio), one word: [2:0] bank (A-D),
 * [6:3] pin, [8:7] 0 input / 1 output / 2 function only, [16:9] the pin's
 * function, [17] the output level. GPIO block 0x0a10a000: mux nibbles as
 * ar_rf_setup_pins, per bank a level register (0x40/0x4c/0x58/0x64) and an
 * output-disable one (0x44/0x50/0x5c/0x68). Only the GPIO functions are
 * known here (stock's table: mux 0, function 0x18 + the pin's index in
 * A0..D12). An air host switches its front end with {0x83, 0x50, 0/2}:
 * D0 as GPIO (0x28), out, low or high power. */
static int set_gpio(uint32_t word)
{
	static const uint8_t first[4] = {0, 5, 10, 16}, count[4] = {5, 5, 6, 13};
	static const uint8_t level[4] = {0x40, 0x4c, 0x58, 0x64};
	unsigned bank = word & 7, pin = (word >> 3) & 15, mode = (word >> 7) & 3;
	unsigned function = (word >> 9) & 0xff, value = (word >> 17) & 1;
	if (bank > 3 || pin >= count[bank] || mode > 2) { return -22; }
	if (function != 0x18u + first[bank] + pin) { return -95; }
	uint32_t mux = bank < 3 ? 0x0a10a02c + bank * 4 : pin < 8 ? 0x0a10a038 : 0x0a10a03c;
	update(mux, 15u << ((bank == 3 && pin >= 8 ? pin - 8 : pin) * 4), 0);
	if (mode == 2) { return 0; }
	uint32_t base = 0x0a10a000 + level[bank];
	update(base + 4, mode == 1 ? 1u << pin : 0, mode == 1 ? 0 : 1u << pin);
	if (mode == 1) { update(base, 1u << pin, value << pin); }
	return 0;
}
/* The radio chip's ADC (stock PRJ 0x8a start / 0x8b stop / GET 0x89): BB
 * 0x090 bit 16 powers it down, 0x08c [31:24] is channel + 1, 0x0a108180 its
 * control (byte 0 start, byte 4 done) and +0x10 the raw 10-bit sample, read
 * each period and kept per channel in mV (x1800/1023, stock's calibration
 * defaults). An air host reads the air unit's battery through it: channel 3
 * (a reference near 900 mV) ten times, then channel 0 every 300 ms (0 V on
 * the bench unit: channels 1-4 read 1585/1770/872/235 mV). Stock's settling
 * waits run as poll-loop steps instead (a wait would hold the loop). */
static void adc_start(unsigned channel, uint32_t period_ms)
{
	/* Powered up now, the channel set and the conversion started 2 ms on
	 * (from a powered-down ADC a start at once was lost: channel 0 read 0). */
	uint32_t now = elapsed_ms();
	if (!adc.on) { field(0x0901001, 0); write8(ADC_BASE + 3, 0); field(0x08c1808, 0); adc.on = 1; }
	adc.channel = (uint8_t)channel; adc.period_ms = period_ms ? period_ms : 1;
	adc.phase = 1; adc.next_ms = now + 2;
}
static void adc_poll(void)
{
	if (!adc.on || (int32_t)(elapsed_ms() - adc.next_ms) < 0) { return; }
	if (adc.phase == 1) {
		field(0x08c1808, 0); field(0x08c1808, adc.channel + 1u); write8(ADC_BASE, 0);
		adc.phase = 2; adc.next_ms = elapsed_ms() + 5;
		return;
	}
	adc.next_ms += adc.period_ms;
	int32_t raw = (int32_t)read32(ADC_BASE + 0x10);
	if (raw < 0) { return; }
	int32_t mv = raw * 1800 / 1023;
	if (mv > 0) { adc.mv[adc.channel] = mv; adc.samples++; }
}
/* SET 0xc8, the project dispatch: [0] sub-command, arguments from [4].
 * 0xcb path control {path, direction mask (TX=1, RX=2), enable}, 0x0e GPIO, 0x8a/0x8b ADC. */
int ar_rf_host_project(const uint8_t *in, size_t length)
{
	if (!in || length < 8) { return -22; }
	switch (in[0]) {
	case 0xcb:
		if (in[4] != 1) { return -95; }       /* path A stays on */
		if (in[5] & ~3u) { return -22; }
		rf_path_b(in[5], !!in[6]);
		return 0;
	case 0x0e:
		return set_gpio(in[4] | (uint32_t)in[5] << 8 | (uint32_t)in[6] << 16 | (uint32_t)in[7] << 24);
	case 0x8a:
		if (length < 12 || in[4] > 5) { return -22; }
		adc_start(in[4], in[8] | (uint32_t)in[9] << 8 | (uint32_t)in[10] << 16 | (uint32_t)in[11] << 24);
		return 0;
	case 0x8b:
		if (adc.on) { field(0x0901001, 1); adc.on = 0; adc.phase = 0; }
		return 0;
	}
	return -95;
}
/* GET 0xc8: 0x89 {[4] channel} -> [4..7] its last value (mV). */
int ar_rf_host_project_get(const uint8_t *in, size_t length, uint8_t out[256])
{
	if (!in || !out || length < 8) { return -22; }
	memset(out, 0, 256);
	if (in[0] != 0x89) { return -95; }
	if (in[4] >= 8) { return -22; }
	uint32_t v = (uint32_t)adc.mv[in[4]];
	for (unsigned b = 0; b < 4; b++) { out[4 + b] = (uint8_t)(v >> (8 * b)); }
	return 0;
}
int ar_rf_host_power(uint32_t reqid, const uint8_t *in, size_t length, uint8_t *out, size_t *out_length)
{
	*out_length = 0;
	switch (reqid) {
	case 0x02000008:
		if (length < 2 || in[0] >= AR_POWER_USERS) { return -22; }
		return apply_power(in[0], in[1]);
	case 0x02000009:
		/* Modes: 0 manual, 1 auto, 2 a range (an air host: {2, dBm, dBm}
		 * before its SET 8), 3 its standby; an AP transmits on user 8. */
		if (length < 3 || in[0] > 3) { return -22; }
		if ((in[0] == 1 || in[0] == 2) && (in[2] < 9 || in[1] > 39 || in[2] > in[1])) { return -22; }
		memcpy(power_auto, in, 3);
		/* Automatic power: a DEV asks for the AP's power in mode 1 (off in
		 * 0); an AP takes requests inside a mode-2 range. */
		if (!ap_role && in[0] <= 1) { (void)ar_pwr_auto_set_range(pwr_auto_context(), in[0], in[1], in[2]); }
		if (ap_role && in[0] == 2) { ap_pwr_max = in[1]; ap_pwr_min = in[2]; }
		/* A DEV's mode 1 is the window it asks the AP's power in (the AP's
		 * own levels), not its own power: a host sets that with SET 8. */
		if (!ap_role && in[0] == 1) { return 0; }
		return in[0] == 1 || in[0] == 2 ? apply_power(ap_role ? 8 : 0, in[1]) : 0;
	case 0x01000007:
		out[0] = power_cfg.closed[0] | power_cfg.closed[1] << 1; *out_length = 1; return 0;
	case 0x01000008:
		if (length < 1) { return -22; }
		out[0] = in[0]; out[1] = in[0] < AR_POWER_USERS ? power_level[in[0]] : 0xff; *out_length = 2; return 0;
	case 0x01000009:
		memcpy(out, power_auto, 3); *out_length = 3; return 0;
	case 0x0200006e: {
		/* {band, offset (dB, signed), path} (stock 0x24bxxx: path 0-1, a
		 * configured band, +-3 dB; the second path +-6). */
		if (length < 3 || in[2] > 1 || in[0] >= power_groups) { return -22; }
		int offset = (int8_t)in[1], limit = in[2] ? 6 : 3;
		if (offset < -limit || offset > limit) { return -22; }
		power_offsets[in[0]][in[2]] = (int8_t)offset;
		if (in[0] == power_band) { power_cfg.path_offset[in[2]] = (int8_t)offset; reapply_power(ap_role ? 8 : 0); }
		return 0;
	}
	case 0x01000071:
		/* {band} -> the two paths' offsets (an air host's telemetry). */
		if (length < 1 || in[0] > 2) { return -22; }
		out[0] = (uint8_t)power_offsets[in[0] < power_groups ? in[0] : power_groups - 1][0];
		out[1] = (uint8_t)power_offsets[in[0] < power_groups ? in[0] : power_groups - 1][1];
		out[2] = out[3] = 0; *out_length = 4; return 0;
	}
	return -95;
}
/* Stock's link-rate figure (GET 6), kbps: 8 blocks a frame; a block is 288
 * symbols x bits of the modulation, doubled per width step from 5 MHz (40
 * MHz codes as 20), halved per repeat at the two lowest indexes; 7 bytes
 * go per group of LDPC codewords. Matches stock's reports (MCS 10 at 20 MHz
 * on the 5633.92 us frame: 25 934 kbps; stock said 25 933). The index is
 * the radio's internal one (MCS + 2). */
static uint32_t link_rate_kbps(unsigned index, unsigned width, uint32_t frame_us)
{
	static const uint8_t rep[16] = {2, 1}, cr[16] = {0,0,0,1,2,0,1,2,0,1,2,0,1,2,0,1};
	static const uint8_t mod[16] = {0,0,0,0,0,1,1,1,2,2,2,3,3,3,4,4}, bits[5] = {1,2,4,6,8};
	static const uint8_t num[3] = {1,2,3}, den[3] = {2,3,4}, grp[3] = {4,4,8};
	if (index > 15 || width < 2 || width > 5 || !frame_us) { return 0; }
	int shift = (width == 5 ? 4 : (int)width) - rep[index];
	uint32_t coded = (bits[mod[index]] * 288u) << shift;
	uint32_t groups = (coded / 2304u + grp[cr[index]] - 1) / grp[cr[index]];
	int32_t bytes = (int32_t)(coded * num[cr[index]] / den[cr[index]] / 8) - 7 * (int32_t)groups;
	return bytes > 0 ? (uint32_t)((uint64_t)bytes * 64000u / frame_us) : 0;
}
#ifdef AR_RF_SIMULATED
uint32_t ar_rf_test_link_rate(unsigned index, unsigned width, uint32_t frame_us) { return link_rate_kbps(index, width, frame_us); }
#endif
/* GET 6 {dir, slot} -> {mcs, pad[3], u32 kbps}. A DEV has slot 0 only. RX:
 * the MCS the AP sends with, TX: ours; 25 = none, as stock. */
int ar_rf_host_mcs(const uint8_t *in, size_t length, uint8_t out[8])
{
	if (!in || length < 2 || in[0] > 1) { return -22; }
	if (in[1] != 0) { return -6; }
	memset(out, 0, 8);
	unsigned linked = dev_active && receiver.dev_state == 2;
	unsigned index = !linked ? 25 : in[0] ? dev_mcs.observed : tx_mcs_info.current;
	unsigned width = in[0] ? bw_code : 2;
	if (ap_role) {
		/* An AP's TX is its broadcast (user 8). */
		index = ap_link.state == AR_AP_CONNECTED && !in[0] ? tx_interleaver.users[8].mcs : 25;
		width = ap_bw_code;
	}
	out[0] = (uint8_t)index;
	uint32_t kbps = index < 16 ? link_rate_kbps(index, width, dev_frame_microseconds()) : 0;
	for (unsigned b = 0; b < 4; b++) { out[4 + b] = (uint8_t)(kbps >> (8 * b)); }
	return 0;
}
/* GET 107 {frames} -> 216 bytes: self quality (bb_info_t: snr, LDPC error
 * ratios x10000, gains, our TX MCS/channel/power, TX frequency), peer zero.
 * SNR and gains are averaged over the last 8 frames like stock; the LDPC
 * ratios here are the share of those frames with a decode error (stock
 * counts codewords, which this receiver does not expose yet). */
int ar_rf_host_link_info(const uint8_t *in, size_t length, uint8_t out[216])
{
	if (!in || length < 1) { return -22; }
	memset(out, 0, 216);
	if (!dev_active || !quality_count) { return -1; }
	uint32_t newest = quality_ring[(quality_head + 7) & 7].ms, now = (uint32_t)(elapsed_ticks() / 40000);
	if (now - newest > 200) { return -1; }
	uint32_t snr = 0, ga = 0, gb = 0, bad = 0;
	for (unsigned i = 0; i < quality_count; i++) {
		snr += quality_ring[i].snr; ga += quality_ring[i].gain_a; gb += quality_ring[i].gain_b; bad += quality_ring[i].errored;
	}
	uint16_t avg = (uint16_t)(snr / quality_count), ratio = (uint16_t)(bad * 10000 / quality_count);
	out[0] = (uint8_t)avg; out[1] = avg >> 8;
	out[2] = (uint8_t)ratio; out[3] = ratio >> 8; out[4] = (uint8_t)ratio; out[5] = ratio >> 8;
	out[6] = (uint8_t)(ga / quality_count); out[7] = (uint8_t)(gb / quality_count);
	out[8] = (uint8_t)(receiver.dev_state == 2 ? tx_mcs_info.current : 25);
	out[9] = (uint8_t)dev_transmit_channel;
	out[10] = power_level[0] != 0xff ? power_level[0] : 0;
	for (unsigned b = 0; b < 4; b++) { out[12 + b] = (uint8_t)(tuned_frequency[0][0] >> (8 * b)); }
	out[16] = (uint8_t)bw_code;
	return 0;
}
int ar_rf_host_retx_config(const uint8_t *payload, size_t length)
{
	/* Stock 0x2000026 (bb_link_retx_ctrl_cfg): mode 0 only, the first 8
	 * bytes replace a 64-bit retransmission control mask in the link state,
	 * answered 0. Its consumer is the per-user retx feed, which this DEV
	 * does not run, so the mask is only kept. */
	if (!payload || length < 8) { return -22; }
	memcpy(retx_config, payload, 8);
	return 0;
}
/* SET 36 (0x24ea0c): the host replaces one row of the DEV MCS policy, found
 * by its internal index; a row the configuration does not have answers 6.
 * Wire: [4] index, [5] consecutive-down, [6..7] SNR up, [8..9] SNR down,
 * [10] errors up, [11] errors down, [12..13] up keep ms, [14..15] down keep.
 * Both the saved plan (what a rebuild reloads) and the live policy change. */
int ar_rf_host_mcs_row(const uint8_t row[16])
{
	if (!row) { return -22; }
	if (!host_config.valid || host_config.length < 32 + AR_DEV_MCS_CONFIG_BYTES) { return -7; }
	uint8_t *policy = host_config.channels + 32;
	unsigned count = policy[2], index = row[4];
	for (unsigned i = 0; i < count && i < AR_DEV_MCS_ROWS; i++) {
		uint8_t *r = policy + 16 + i * 12;
		if (r[0] != index) { continue; }
		/* SNR thresholds and keep times only. The error counts and the
		 * consecutive-error rule stay the configuration's: stock's rows (2 bad
		 * samples in 8, 2 in a row, for MCS 10) are tuned to its LDPC failure
		 * count, while this policy samples the F08 per-packet error word,
		 * non-zero in 1-2% of packets on a clean bench link - with stock's
		 * numbers the link flapped 10<->8 every few seconds. */
		memcpy(r + 4, row + 6, 2); memcpy(r + 6, row + 8, 2);
		memcpy(r + 8, row + 12, 2); memcpy(r + 10, row + 14, 2);
		if (i < dev_mcs.config.count && dev_mcs.config.rows[i].mcs == index) {
			struct ar_dev_mcs_row *live = &dev_mcs.config.rows[i];
			live->snr_up = row[6] | (uint16_t)row[7] << 8; live->snr_down = row[8] | (uint16_t)row[9] << 8;
			live->up_ms = row[12] | (uint16_t)row[13] << 8; live->down_ms = row[14] | (uint16_t)row[15] << 8;
		}
		return 0;
	}
	return 6;
}
const struct ar_rf_lifecycle_info *ar_rf_lifecycle_information(void)
{
	lifecycle.state = information.error ? 3 : !host_initialized ? 0 : information.phase == AR_RF_RECEIVING ? 2 : 1;
	return &lifecycle;
}

const struct ar_rf_rx_info *ar_rf_receive_information(void) { return &receiver; }

int ar_rf_host_registers(unsigned offset, unsigned length, uint8_t output[256])
{
	if (!output) { return -22; }
	/* Original 0x24612c accepts unaligned and empty windows in a 4 KiB
	 * page. Check bounds before MMIO, including offset 4096 with length 0. */
	if (length > 256 || offset > 4096 || length > 4096 - offset) { return -6; }
	if (information.phase == AR_RF_OFF) { return -11; }
	/* Stock's core handler leaves the unused reply tail untouched. Do not
	 * expose previous buffer contents through the fixed-size host reply. */
	memset(output, 0, 256);
	for (unsigned i = 0; i < length; i++) {
		unsigned byte = offset + i;
		/* Preserve one aligned volatile word read per byte, as in stock.
		 * Coalescing reads would change behavior for changing registers. */
		output[i] = read32(BB + (byte & ~3u)) >> ((byte & 3) * 8);
	}
	return 0;
}

int ar_rf_read_registers(unsigned offset, unsigned length, uint8_t *output)
{
	/* Diagnostic readback of the baseband register bank (configuration page
	 * and the status words above 0xe00), in any phase after PSRAM: reads
	 * are harmless and a linked-state dump is what a comparison with the
	 * stock firmware needs. Never exposes arbitrary MCU memory. */
	if (information.phase < AR_RF_PSRAM || !output || !length || length > 256 ||
	    (offset & 3) || (length & 3) || offset > 0x1000 - length) { return -22; }
	for (unsigned i = 0; i < length; i += 4) {
		uint32_t word = read32(BB + offset + i);
		for (unsigned byte = 0; byte < 4; byte++) { output[i + byte] = word >> (byte * 8); }
	}
	return 0;
}
