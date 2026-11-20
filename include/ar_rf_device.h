/* SPDX-License-Identifier: MIT */
#ifndef AR_RF_DEVICE_H
#define AR_RF_DEVICE_H
#ifdef AR_RF_SIMULATED
#include "ar_command.h"
#include "ar_power_control.h"
const struct ar_power_control *ar_rf_power_information(void);
const struct ar_command_program *ar_rf_command_information(void);
#endif
#include <stddef.h>
#include <stdint.h>

enum ar_rf_phase { AR_RF_OFF, AR_RF_PROBED, AR_RF_DEFAULTS, AR_RF_CALIBRATED, AR_RF_PSRAM, AR_RF_RECEIVING };
#define AR_RF_MAX_CHANNELS 42u
struct ar_rf_channel_info {
	uint32_t frequency_khz, visits, power_samples, power_last, power_min, power_max;
};
struct ar_rf_info {
	uint32_t phase, chip_profile, default_writes, calibration_rounds;
	uint32_t otp_status, calibration_status, psram_status, error;
};
struct ar_rf_eop_snapshot {
	uint32_t elapsed_us, status, header, valid_indications;
	uint32_t control[7];
};
/* Operation 64: local EOP measurements for subsequent PHY policy.
 * Register fields retain their observed representation; units are unknown. */
/* DEV4 = DEV3 prefix32 + policy136 + original-payload FCH keys25*4. */
#define AR_RF_DEV4_PREFIX 268
struct ar_rf_mcs_info {
	uint32_t samples, requests, acknowledgements, unsolicited, timeouts;
	uint32_t advertisements, cancellations, state, current, previous, observed;
	uint32_t average_snr, bad_samples, recent_bad, sample_ms, request_ms, direction_ms;
	uint32_t fch_writes, buffer_applies, rx_resets, reset_errors, fch_words[2];
	uint32_t first_request_ms, first_ack_ms, first_observed_mcs, last_header;
	uint32_t sync_mmio, reserved[4], observed_counts[25], unknown_headers, padding[6];
	/* Appended to preserve the existing 256-byte diagnostic prefix. */
	uint32_t reset_deferred, reset_pending;
};
const struct ar_rf_mcs_info *ar_rf_mcs_information(void);
/* Host requests for the DEV link: SET 36 MCS row, SET 22 bandwidth,
 * SET 16 frame mode, GET 6 MCS/rate, GET 107 link quality, power (GET 7/8/9,
 * SET 8/9), SET 0x26 retransmission mask, GET 10 channels, SET 5/6/35. */
int ar_rf_host_mcs_row(const uint8_t row[16]);
int ar_rf_host_bandwidth(const uint8_t *payload, size_t length);
int ar_rf_host_frame_mode(const uint8_t *payload, size_t length);
int ar_rf_host_mcs(const uint8_t *in, size_t length, uint8_t out[8]);
int ar_rf_host_link_info(const uint8_t *in, size_t length, uint8_t out[216]);
int ar_rf_host_power(uint32_t reqid, const uint8_t *in, size_t length, uint8_t *out, size_t *out_length);
int ar_rf_host_retx_config(const uint8_t *payload, size_t length);
int ar_rf_host_channel_info(uint8_t output[1028]);
int ar_rf_host_channel_set(unsigned order, const uint8_t *payload, size_t length);
/* GET 5: {u8 slot bitmap} -> int32 metres[8], -1 without a recent fix. */
int ar_rf_host_distance(const uint8_t *payload, size_t length, uint8_t output[32]);
/* Operation68 observational trace. First32 and a rolling last32 records;
 * register reads are sequential, not an atomic hardware snapshot. */
struct ar_rf_phy_event {
	uint32_t begin_us, end_us, flags, fbc, fch_before, fch_after;
	uint32_t errors, snr, command, pending, samples, mac_valid;
};
struct ar_rf_phy_bucket {
	uint32_t count, snr_min, snr_max, snr_sum, zero_errors, error_sum, first_us, last_us;
};
struct ar_rf_phy_trace {
	uint32_t events, first_count, last_count, last_head, polls, max_poll_gap_us;
	uint32_t simultaneous_pending, header_changed, max_handler_us, reserved[7];
	/* Accepted EOP, duplicate EOP, EOP with stored valid FCH, valid FCH,
	 * invalid FCH, DMA completion (whether or not MAC validation passes). */
	struct ar_rf_phy_bucket buckets[6];
	struct ar_rf_phy_event first[32], last[32];
};
const struct ar_rf_phy_trace *ar_rf_phy_trace_information(void);
/* Operation69: checksum/length-gated incoming FCH identity synchronization. */
struct ar_rf_sync_info {
	uint32_t accepted, rejected, unsupported, identity_commands, armed, immediate;
	uint32_t timer_fired, rx_writes, tx_writes, state_skips, invalid_users;
	uint32_t pending, deadline_ms, first_apply_us, last_apply_us, user, identity, transmit;
	uint32_t field_before, field_after, shared_before, shared_after;
	uint8_t first_identity[8], last_control[8];
	uint32_t reserved[6];
};
const struct ar_rf_sync_info *ar_rf_sync_information(void);
struct ar_rf_command_observation {
	uint32_t accepted, dispatched, first_us, last_us;
	uint8_t first[8], last[8];
};
struct ar_rf_port_observation {
	uint32_t packets, bytes, first_us, last_us, first_sequence, last_sequence;
};
/* Operation71: observations after MAC or FCH integrity checks. Port counts
 * precede peer/session gating; they do not imply endpoint delivery. */
struct ar_rf_traffic_info {
	uint32_t version, reserved;
	struct ar_rf_command_observation commands[10];
	struct ar_rf_port_observation ports[5];
	uint32_t port_bytes_by_second[16][5];
	uint32_t socket_offered, socket_dropped; /* ar_socket_rx_totals, since boot */
	/* Main loop: iterations, socket frames pushed, pushes refused by the bus,
	 * longest iteration (us), bus transfers started, bus queue high water. */
	uint32_t loop_iterations, loop_pushes, loop_refused, loop_max_us, bus_transfers, bus_high_water;
};
const struct ar_rf_traffic_info *ar_rf_traffic_information(void);
struct ar_rf_tx_mcs_info {
	uint32_t requests, applied, unchanged, rejected, downgrades;
	uint32_t first_apply_us, last_apply_us, current, requested, before, after, key;
	uint32_t mode_changed, pending, drained, busy_requests;
};
const struct ar_rf_tx_mcs_info *ar_rf_tx_mcs_information(void);
struct ar_rf_tx_history_info {
	uint32_t samples, last_user, other_users, eligible;
	uint32_t history, bookkeeping, mcs, previous_mcs;
};
const struct ar_rf_tx_history_info *ar_rf_tx_history_information(void);
struct ar_rf_phy_info {
	uint32_t samples, duplicates, other_users, forced_errors, sample_elapsed_us;
	uint32_t last_fbc, last_fch, raw_errors, raw_snr, raw_gain[2], raw_offset[2], raw_aux;
	uint8_t sample[16], first_sample[16];
};
const struct ar_rf_phy_info *ar_rf_phy_information(void);
/* Project diagnostic, version2. Counts remain observations. channel_scores use
 * the original base-channel averaging/conversion on the confirmed subset of
 * polling events; physical attribution and IRQ cadence remain unverified. */
struct ar_rf_scan_info {
	uint32_t version, count, channel, frequency_khz, tunes, armed, samples, dropped, stale, errors, phase;
	uint32_t channel_samples[AR_RF_MAX_CHANNELS];
	uint32_t score_ring, score_ready, score_rounds;
	int32_t channel_scores[AR_RF_MAX_CHANNELS];
};
const struct ar_rf_scan_info *ar_rf_scan_information(void);
/* What the peer sends, recorded on change: MAC packets (header, descriptors and
 * the start of the outband) whose outband or descriptor layout differs from the
 * last one recorded of its kind, and valid PHY control messages whose bytes
 * differ (command 7, the AP clock, every 64th). For learning the AP side. */
#define AR_RF_CAPTURE_ENTRIES 48
struct ar_rf_capture_entry { uint32_t seq, elapsed_us, extra; uint16_t kind, length; uint8_t data[52]; };
struct ar_rf_capture_info {
	uint32_t version, head, packets, full, compact, other, phy, recorded;
	uint32_t phy_commands[16], outband_lengths[8];
	/* The DEV's own side: state, slot, what it sent up. */
	uint32_t dev_state, assigned_slot, tx_preencode, tx_packets, tx_errors, tx_budget, tx_user;
	uint32_t lock_result, login_packets, login_matching, sync_unsupported, dev_active, admission_result, lock_messages, sync_acks;
	uint32_t rx_max_length, rx_over_4k;     /* received packets: the largest, and those over 4096 bytes */
	struct ar_rf_capture_entry entries[AR_RF_CAPTURE_ENTRIES];
};
const struct ar_rf_capture_info *ar_rf_capture_information(void);
/* The AP role (air unit, AP04 plan): link state, what it sent, what came up. */
struct ar_rf_ap_info {
	uint32_t version, state, token, channel;
	uint32_t preencodes, other_users, packets, bytes, payload_bytes, errors;
	uint32_t budget_last, budget_min, budget_max, last_user;
	uint32_t uplink_packets, uplink_invalid, reports, bad_reports, foreign_reports;
	uint32_t logins, connects, drops, acks, bad_acks, commands, bandwidth_requests;
	uint32_t phy_sent, phy_expired, identity, tx_end;
	uint8_t ground[4], last_report[16], last_packet[64];
	uint32_t phase, error, irq_total, irq_counts[20], reg800, reg80c, regfa8, rege6c, regf70;
	uint32_t error_last, fch_last, fch_valid, power_last, regf00, regf04, regf08, regf0c;
	uint32_t receive_events, invalid_packets, dma_completions, rx_pending, rx_metadata, regf88, regf90, regbe8;
	uint32_t decoder_resets, decoder_reset_result;
	uint32_t calibration_rounds, calibration_status, configured_us;
	uint32_t broadcast_mcs, bandwidth, mcs_requests, bandwidth_switches, uplink_phy;
	uint32_t payload_packets, empty_packets;
	uint32_t preencode_requests[16];         /* pre-encodes by 0xf58[7:4] */
	uint32_t preencode_late;                 /* handled with the encoder already started (IRQ 4 pending) */
	/* Main-loop gaps between RF polls, and those just before a pre-encode:
	 * counts over 50, 100, 200, 500, 1000 us, and the longest (us). */
	uint32_t poll_gaps[5], pre_gaps[5], poll_gap_max, pre_gap_max;
	/* Main-loop stages (0 bus receive, 1 frame parse + dispatch, 2 socket
	 * services, 3 reply push): longest (us) and how often over 200 us. */
	uint32_t host_max[4], host_slow[4];
	uint32_t frame_mode, frame_requests, frame_switches, frame_us;   /* TDD frame mode (0 standard, 2 short) */
	uint32_t retx_threshold, retx_disables, retx_enables;            /* broadcast HARQ recovery */
	uint32_t tx_stalls;                                              /* frame control with 0xf70 [7:6] set, first of a run */
	uint32_t uplink_mcs, mcs_keys[4];                                /* the MCS asked of the ground (command 3), keys 0-3 */
	uint32_t channel_requests, channel_moves, channel_errors;         /* command 4 heard, hops done, bad index */
	uint32_t gpio_d[3], adc_samples, adc_raw; int32_t adc_mv[8];      /* bank D mux 0x38, level 0x64, output-off 0x68; ADC */
	/* The ground's uplink MCS policy: requests (command 3), changes seen,
	 * timeouts, samples (end of packet), last SNR, packets with LDPC errors,
	 * the MCS its FCH shows. */
	uint32_t uplink_requests, uplink_changes, uplink_timeouts, uplink_samples, uplink_snr, uplink_errors, uplink_observed;
	/* Own channel choice: frames with a block sent again, the last measure,
	 * hops by level / blind, rollbacks, hops kept 2 s, measured starts,
	 * last hop (from | to << 8 | measure << 16). */
	uint32_t hop_retx_frames, hop_measure, hop_by_power, hop_blind, hop_rollbacks, hop_kept, measured_starts, hop_last;
	/* Receive re-arms asked for, by cause: uplink interrupts late (frames in
	 * a row), the halted bit with no uplink packet, invalid headers; and the
	 * late frames and halted readings that did not lead to one. */
	uint32_t rearm_late, rearm_halted, rearm_fch, late_frames, halted_seen;
	/* Stream bytes copied ahead of their pre-encode, and of those sent. */
	uint32_t staged_bytes, staged_used;
};
const struct ar_rf_ap_info *ar_rf_ap_information(void);
/* The highest MCS the receive policy may ask for (request 0x0c5a0036, one
 * byte: the stock MCS number, -2..13, or 0xff for none). Kept until changed. */
int ar_rf_host_mcs_ceiling(const uint8_t *payload, size_t length);
/* The frame and the AP's clock on it (both roles; ar_frame_stamp.h): what the
 * hosts' apps read their radio clock from. flags: 1 AP, 2 the AP's clock
 * known (an AP always; a ground once it took a stamp on this link), 4 the AP
 * is Open8030 (ground: stamps on this link). The AP's clock at our latest
 * IRQ 5 (us, modulo 2^32: ap_us_hi 0), and how long ago that IRQ 5 was on our timer. */
struct ar_rf_frame_info {
	uint32_t version, flags, frames, frame_ticks, since_us;
	uint32_t ap_us_lo, ap_us_hi;
	uint32_t taken, rejected;
	int32_t residual_us;
	uint32_t width, frame_mode, sent;
	/* Blocks sent again (AP: frames that sent one, its channel choice's
	 * count; ground: blocks, from the AP's resend maps, since start), then
	 * two fields kept 0 (steady mode, its steps down: kestrel-gnd reads this
	 * layout). */
	uint32_t retries, steady, retry_steps;
	/* Ground: the AP's resend maps per frame of ours. retry_frame: our frame
	 * (frames' count: the one that ended at that IRQ 5) of the newest known;
	 * retry_map: 64 frames back from it, two bits each (frame retry_frame - k
	 * in word k / 16, bits 2 (k % 16) + 1..0: 0..2 blocks sent again, 3 none
	 * confirmed); retry_known: one bit each, set where the map knows it;
	 * frames known and stalled ones since start. */
	uint32_t retry_frame, retry_known_frames, retry_stalls;
	uint32_t retry_map[4], retry_known[2];
	/* Ground: each of our last 64 frames as its header was received -
	 * q_word[k] is frame q_frame - k: [7:0] SNR in quarter dB, [15:8] and
	 * [23:16] the two paths' AGC gain, [27:24] the MCS (internal), [29:28] the
	 * width code - 2, [30] set when known, [31] the decoder's error flag. */
	uint32_t q_frame, q_word[64];
};
const struct ar_rf_frame_info *ar_rf_frame_information(void);
void ar_rf_note_host_stage(unsigned stage, uint32_t us);
/* Stock's receive-decoder reset (its "DEC HALT patch"): baseband hold, both
 * engine flush requests until acknowledged, release. 0, or -110 when a flush
 * was not acknowledged. */
int ar_rf_ap_decoder_reset(void);
/* SET 0xc8 project dispatch (path B control; GPIO and ADC acknowledged). */
int ar_rf_host_project(const uint8_t *in, size_t length);
int ar_rf_host_project_get(const uint8_t *in, size_t length, uint8_t out[256]);
/* SET 0x1b {slot, min MCS, max MCS} (internal indexes). */
int ar_rf_host_uplink_mcs_range(const uint8_t *in, size_t length);
/* Test: 1 re-arms RX now (0x800 bit 1 + 0x80c bit 2), 2 TX now (bit 2 +
 * bit 3), 3 RX at the next IRQ 5 (end of the sweep before the receive slot). */
int ar_rf_ap_rearm(unsigned variant);
/* Prepared with an AP04 plan: the radio starts by itself (the air unit's host
 * never sends init/start). */
int ar_rf_ap_role(void);
/* The AP's last interrupts, frozen when its pre-encodes stop (a stalled
 * frame engine, frozen 1) or 120 interrupts after its first login (2): time,
 * source | 0xfa8 << 8, 0xf70, 0xf88, 0xf90, and on 14 the header read. */
#ifdef AR_ROLE_AP
#define AR_RF_AP_TRACE_ENTRIES 400
#else
#define AR_RF_AP_TRACE_ENTRIES 160   /* a DEV's replies hold 4 KiB */
#endif
struct ar_rf_ap_trace { uint32_t head, frozen, frozen_us, last_pre_us, freeze_at; uint32_t entries[AR_RF_AP_TRACE_ENTRIES][6]; };
const struct ar_rf_ap_trace *ar_rf_ap_trace_information(void);
void ar_rf_ap_trace_unfreeze(unsigned mode);
void ar_rf_ap_trigger(unsigned position, unsigned byte);
struct ar_rf_rx_info {
	uint32_t frequency_khz, irq_total, irq_counts[20];
	uint32_t fch_last, fch_locked, receive_events, dma_completions;
	uint32_t packets, invalid_packets, bytes, last_length, last_user, last_mcs;
	uint8_t first_bytes[64];
	uint32_t command_last, command_modes[4];
	uint32_t context_words[16];
	uint32_t error_last, snr_words[2];
	uint32_t decoder_controls[2];
	uint32_t clock_controls[8];
	uint32_t gpio_controls[8];
	uint32_t search_restarts, search_frame_us, search_dwell_ms;
	uint32_t search_suspended, search_error, monitor_elapsed_us;
	uint32_t power_samples, power_last, power_min, power_max;
	uint32_t channel_count, channel_index, channel_wraps;
	struct ar_rf_channel_info channels[AR_RF_MAX_CHANNELS];
	/* Source-2 command context observed while polling, not decoded packets.
	 * Append fields to preserve the existing diagnostic wire offsets. */
	uint32_t source2_users[16], source2_modes[4];
	uint32_t source2_command_first, source2_command_last;
	uint32_t header_users[16], header_valid_users[16];
	uint32_t first_valid_header, first_valid_command, first_valid_elapsed_us;
	uint32_t eop_users[16], eop_last;
	/* First valid indication: F04/F08/F0C/F10/F34/F38/F3C with BB0[7]=1.
	 * Raw control-header evidence; not an accepted MAC packet. */
	uint32_t first_valid_control[7];
	/* First four EOPs expose timing relative to the header indication. */
	struct ar_rf_eop_snapshot eop_snapshots[4];
	uint32_t before_monitor_header, before_monitor_control[7];
	uint32_t before_release_header;
	/* First-valid bank audit: selector readbacks and delayed bank-1 data. */
	uint32_t first_selector_old, first_selector_set, first_selector_delayed;
	uint32_t first_control_bank0[7], first_control_delayed[7];
	uint32_t first_bank_delay_status;
	/* Operation 50: bounded MCS-buffer hypothesis, not MAC acceptance. */
	uint32_t mcs_buffer_transitions, mcs_buffer_header;
	uint32_t mcs_buffer_bf0_before, mcs_buffer_bf0_after;
	uint32_t mcs_buffer_d08_before, mcs_buffer_d08_after;
	/* Operation 51: count the known-invalid all-zero control pattern. */
	uint32_t empty_control_headers, last_control_header, last_control_words[7];
	uint32_t bad_control_headers, matching_control_headers, last_control_check;
	/* Packet integrity is separate from DMA completion and peer acceptance.
	 * DEV details retain the last successfully decoded full control message. */
	uint32_t mac_valid_packets, mac_invalid_packets;
	uint32_t dev_control_valid, dev_control_invalid, dev_control_unsupported;
	uint32_t last_mac_result, last_dev_result, last_mac_sequence;
	uint32_t last_mac_descriptors, last_outband_length;
	uint32_t last_dev_length, last_dev_sequence, last_dev_channels, last_dev_extensions;
	uint8_t last_dev_peer[4], last_dev_control[200];
	uint32_t first_packet_elapsed_us, last_packet_elapsed_us, last_rx_metadata;
	/* Operation 54: configured mode-0 admission and bounded control TX.
	 * Acquisition readiness/engine activation do not imply state-2 login. */
	uint32_t admission_attempts, admission_rejects, last_admission_result;
	uint32_t acquisition_ready, transition_status, transition_elapsed_us;
	uint32_t tx_preencode, tx_packets, tx_end, tx_errors;
	uint32_t last_tx_user, last_tx_budget, last_tx_request, tx_owned;
	uint32_t first_tx_elapsed_us, last_tx_elapsed_us, sync_control_words[2];
	uint8_t last_tx_packet[24], accepted_peer[4];
	uint32_t assigned_slot;
	/* Sampled IRQ timing and source-18 request context, not over-air ACKs. */
	uint32_t irq_last_elapsed_us[20], interleaver_users[16];
	uint32_t interleaver_first_f68, interleaver_last_f68;
	uint32_t interleaver_first_f64, interleaver_last_f64;
	uint32_t interleaver_b4c_before, interleaver_b4c_after, interleaver_updates;
	/* Operation 57 observes post-admission lock decisions without changing
	 * the transmitted state/slot or asserting completed remote sync. */
	uint32_t lock_messages, login_extension_packets, login_matching_packets;
	uint32_t first_login_elapsed_us, last_login_elapsed_us;
	uint32_t first_matching_elapsed_us, last_matching_elapsed_us, last_lock_result;
	uint32_t lock_results[8], lock_slot_states[4], lock_observed_extensions;
	uint32_t first_login_length, first_matching_length;
	uint8_t first_login_control[200], first_matching_control[200];
	uint32_t last_login_slot, last_login_token, last_login_value, last_login_ap_state;
	uint32_t last_login_sync;
	/* Operation 59: mode-0 assignment and zero-flags remote-sync ACK.
	 * These are local transition/submission observations, not video parity. */
	uint32_t dev_state, dev_token, connected_elapsed_us, sync_unsupported;
	uint32_t sync_ack_pending, sync_ack_submitted, link_invalidations;
	uint32_t last_tx_length;
	uint8_t last_tx_extended[40];
	uint32_t payload_packets, port0_packets, port0_bytes, first_port0_length, last_port0_length;
	uint8_t first_port0[132], last_port0[132];
	uint8_t first_sync_ack_packet[36]; /* 33 wire bytes plus zero padding. */
	/* Operation 60: compact integrity is separate from context acceptance. */
	uint32_t compact_valid, compact_invalid;
	uint8_t last_compact[8];
	uint32_t port_packets[5], port_bytes[5], other_port_packets, other_port_bytes;
	uint32_t first_data_port, first_data_length;
	uint8_t first_data[128];
	uint32_t port0_commands[8], port0_unframed, first_nonstats_length;
	uint8_t first_nonstats[132];
	/* Operation 61: hardware request-ID watchdog, not over-air delivery. */
	uint32_t retx_events, retx_disable, retx_enable, retx_first_disable_us, retx_last_disable_us;
	uint32_t retx_last_request, retx_last_id, retx_repeated, retx_phase;
	uint32_t retx_b00_before, retx_b00_after, retx_errors, retx_request_ids[16];
	/* Operation 62: complete internal stream envelopes, including splits. */
	uint32_t stream_commands[8], stream_checksum_errors, stream_length_errors;
	uint32_t stream_discarded, stream_buffered;
	/* Receive events dropped: flagged (metadata bit 29), all DMAs busy, other. */
	uint32_t drop_flagged, drop_busy, drop_other;
};
int ar_rf_probe(void);
/* Payload view is borrowed only for this synchronous polling callback. */
typedef void (*ar_rf_payload_callback)(unsigned slot, unsigned port,
	const uint8_t *data, size_t length, void *context);
void ar_rf_set_payload_callback(ar_rf_payload_callback callback, void *context);
typedef size_t (*ar_rf_tx_peek_callback)(uint8_t *scratch, size_t capacity,
	unsigned *mac_port, void *context);
typedef void (*ar_rf_tx_commit_callback)(unsigned mac_port, size_t length, void *context);
/* Both callbacks are required. Polling only: peek and commit cannot race with
 * host close/reset. Commit follows copying into the RF-owned packet ring. */
void ar_rf_set_tx_callbacks(ar_rf_tx_peek_callback peek, ar_rf_tx_commit_callback commit, void *context);
/* Optional (an AP): peek from offset bytes in on a chosen port, and the bytes
 * a port has to go - the next block is staged from the main loop. */
typedef size_t (*ar_rf_tx_peek_at_callback)(unsigned *mac_port, size_t offset, uint8_t *scratch,
	size_t capacity, void *context);
typedef size_t (*ar_rf_tx_available_callback)(unsigned mac_port, void *context);
void ar_rf_set_tx_staging(ar_rf_tx_peek_at_callback peek_at, ar_rf_tx_available_callback available);
struct ar_rf_tx_payload_info {
	uint32_t packets, bytes, first_length, last_length, last_port;
	uint8_t first_packet[160], last_packet[160];
};
const struct ar_rf_tx_payload_info *ar_rf_tx_payload_information(void);
int ar_rf_setup_pins(const uint8_t *records, size_t length);
/* The I/O domain bits ar_rf_setup_pins selects (bits 1-2 of GPIO+0); 0 derives
 * them from the banks used. Call before ar_rf_setup_pins. */
void ar_rf_set_io_domains(unsigned mask);
int ar_rf_load_defaults(const uint8_t *records, size_t length);
int ar_rf_calibrate(void);
int ar_rf_setup_psram(void);
const struct ar_rf_info *ar_rf_information(void);
int ar_rf_pll_word(uint32_t frequency, uint64_t *word);
/* Ascent VRX 18.21.10 control check; input includes the stored check in byte 0.
 * A matching check alone does not validate peer identity or MAC semantics. */
uint8_t ar_rf_control_checksum(const uint8_t control[7]);
int ar_rf_monitor(uint32_t frequency, uint32_t milliseconds, uint8_t identity_hash, unsigned operation);
/* LE32 channel list for DEV search (39), power survey (40), configured RX8
 * bandwidth (41), configured RX8 bandwidth/interleaver/duration (42), or
 * that profile with the observed prestart upper RX8 byte (43), or
 * the same profile with startup compensation fields restored (44), or
 * that profile with the DEV B4C startup field (45), or the observed linked
 * RX8 bandwidth/interleaver/duration with operation-45 prerequisites (46),
 * or that profile with the observed linked receive identity (47), or operation
 * 46 with the linked RX8 upper byte and zero acquisition identity (48), or
 * operation 46 with A48 matched to the configured TX bandwidths (49), or
 * that baseline with a one-shot observed-MCS buffer transition (50), or
 * baseline 49 with search retained after all-zero control headers (51), or
 * after control checksum failures (52), or operation 45's configured 5 MHz
 * startup profile with checksum-aware retries and up to 10 seconds (53).
 * Operation 54 accepts a DEV0 prefix followed by the complete frequency table:
 * magic[4], local[4], configured_peer[4], candidates[2][4], candidate_count,
 * receive_channel_index, raw_status_a, raw_status_b. Mode 0, bitmap 1, no relay
 * or pairing, and same-channel uplink only. The frequency argument must equal
 * the indexed table entry; search stays on that one frequency. Up to 10 seconds.
 * Operation 55 adds the unconditional source-18 B4C user-field update; its
 * conditional clock adjustment and later MCS retry policy remain unported.
 * Operation 56 uses the stock acquisition TX0 MCS index 1 and its configured
 * 2234-us duration, including the resulting search-frame/dwell calculation.
 * Operation 57 adds lock/login observations and retains first login evidence;
 * it does not apply an assignment or execute remote synchronization.
 * Operation 58 requires DEV1: the same first 24 prefix bytes plus four
 * effective TX0 gain bytes (A analog6/digital8, B analog6/digital8), then the
 * frequency table. Caller resolves calibration offsets/clock scaling; no
 * proprietary table or automatic power policy is provided by this API.
 * Operation 59 requires DEV2: DEV1 fields plus configured power index (0..63)
 * and three reserved zero bytes, then the frequency table. Applies slot-0
 * mode-0 login when remote-sync flags are zero, queues its port-0 ACK and
 * reports state 2. Other remote-sync operations and general link maintenance
 * remain unsupported. Records bounded port-0 payload evidence.
 * Operation 60 adds compact control decoding bound to the established
 * mode-0 slot-0 context and per-port payload diagnostics. Payload delivery,
 * stream reassembly, PHY feedback and dynamic scheduling remain unported.
 * Operation 61 requires DEV3: DEV2 with byte29 = effective signed-positive
 * request-repeat threshold (0..127, zero disables); bytes30/31 remain zero.
 * Adds the DEV request-ID watchdog and its two-event TX control recovery.
 * Operation 62 uses DEV3 and reassembles the internal port-0 byte stream.
 * Complete command observations are distinct from frame-local diagnostics.
 * Earlier operations retain their 1.5-second maximum.
 * The list is copied and fully validated before any peripheral write. */
int ar_rf_monitor_plan(uint32_t frequency, uint32_t milliseconds, uint8_t identity_hash,
	unsigned operation, const uint8_t *channels, size_t length);
/* Host-session preparation configures the tested DEV4 profile with RX/TX
 * held. The first valid candidate-list command starts its bounded lifetime. */
int ar_rf_prepare_host(uint32_t frequency, uint32_t milliseconds, uint8_t identity_hash,
	const uint8_t *channels, size_t length);
/* Firmware-owned session without a diagnostic deadline. Zero duration remains
 * invalid in ar_rf_prepare_host and all wire diagnostic monitor operations. */
int ar_rf_prepare_host_continuous(uint32_t frequency, uint8_t identity_hash,
	const uint8_t *channels, size_t length);
/* Loader preparation retains resolved calibration for power/tuning callbacks. */
struct ar_power_config;
int ar_rf_prepare_config(uint32_t frequency, uint8_t identity_hash,
	const uint8_t *channels, size_t length, unsigned search, const struct ar_power_config *power);
/* Read-only OTP identity; -2 means blank identity, permitting the JSON fallback. */
int ar_rf_board_identity(uint8_t mac[4]);
/* Hold RX/TX and cancel pending sync work; a new prepare is required to restart. */
int ar_rf_stop_host(void);
/* Original zero-payload lifecycle group0b: start, stop, init, deinit.
 * Configuration survives stop/deinit; active DMA must quiesce before reuse. */
int ar_rf_host_control(unsigned operation, size_t payload_length);
struct ar_rf_lifecycle_info {
	uint32_t version, state, starts, stops, initializations, deinitializations;
	uint32_t cancelled_rx, cancelled_tx, last_error, last_change_ms;
};
const struct ar_rf_lifecycle_info *ar_rf_lifecycle_information(void);
/* Cumulative automatic recovery diagnostics; not reset by session rebuild. */
struct ar_rf_recovery_info {
	uint32_t version, pending, attempts, completed, failures, last_reason;
	uint32_t fch_losses, invalidations, candidate_removals, rx_resets, reset_deferred;
	uint32_t last_error, last_ms, monitor_state, monitor_missed, monitor_threshold, monitor_interval_ms;
	uint32_t stop_stage, stop_pending_rx, stop_fa8, stop_irq13;
};
const struct ar_rf_recovery_info *ar_rf_recovery_information(void);
int ar_rf_host_candidates(const uint8_t *data, size_t length);
int ar_rf_host_pairing(const uint8_t *data, size_t length);
int ar_rf_host_pairing_result(uint8_t output[164]);
int ar_rf_host_peer(const uint8_t *data, size_t length);
const struct ar_rf_rx_info *ar_rf_receive_information(void);
/* Matched GET_STATUS wire layout. Available after a DEV4 fixed-channel plan;
 * unrequested/reserved bytes are zero. Stopped monitors never report linked. */
int ar_rf_host_status(uint16_t users, uint8_t output[300]);
uint32_t ar_rf_host_ap_time(void);
/* Original GET_REG byte window; successful replies always contain 256 bytes.
 * Bytes beyond the requested window are zero; live reads are not atomic. */
int ar_rf_host_registers(unsigned offset, unsigned length, uint8_t output[256]);
typedef void (*ar_rf_event_callback)(unsigned id, const uint8_t *data, size_t length, void *context);
void ar_rf_set_event_callback(ar_rf_event_callback callback, void *context);
/* Event 14 with the TX power as it is now: our own, and on a DEV the AP's
 * as it last announced it (each only if known). */
void ar_rf_power_events(void);
void ar_rf_poll(void);
int ar_rf_read_registers(unsigned offset, unsigned length, uint8_t *output);
struct ar_power_config;
void ar_rf_set_power_config(const struct ar_power_config *config);
/* Boards with a second calibration group: its rows, used from `edge_khz` up
 * (call after ar_rf_set_power_config). */
void ar_rf_set_power_bands(const uint8_t (*high)[4], uint32_t edge_khz);   /* AR_POWER_LEVELS rows */
struct ar_hop_config;
/* An AP's channel policy (see ar_config.h), taken at boot. */
void ar_rf_set_hop_config(const struct ar_hop_config *hop);
/* A DEV's broadcast width policy from its configuration (ar_dev_width.h):
 * narrowest and widest code (2..5), the internal MCS it narrows at or below
 * and widens above. A zero max keeps 5 to 20 MHz. */
void ar_rf_set_bandwidth_policy(unsigned automatic, unsigned min, unsigned max, unsigned narrow_mcs, unsigned widen_mcs);
#endif

int ar_rf_listener_state(uint32_t output[8]);
