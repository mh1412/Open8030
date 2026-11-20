/* SPDX-License-Identifier: MIT */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include "ar_frame.h"
#include "ar_factory.h"
#ifdef CONFIG_SOC_AR8030_BRINGUP
#include "ar_transport.h"
#include "ar_usb_device.h"
#include "ar_sdio_device.h"
static const struct ar_transport *bus;
#include "ar_rf_device.h"
#include "ar_boot.h"
#include "ar_socket_rx.h"
#include "ar_event.h"
#include "ar_telemetry.h"
static struct ar_socket_rx sockets;
static struct ar_events events;
static struct ar_telemetry telemetry;
static struct {
	uint32_t high_water, largest_feed, drop_calls, last_drop_bytes;
	uint32_t tx_room, active_ticks, last_ticks, max_ticks, tx_packets, rx_packets;
} socket_flow[AR_SOCKET_RX_PORTS];
/* Bounded diagnostic record of real host commands during stack integration. */
static struct {
	uint32_t count, overflow, event_drops, reserved;
	struct { uint32_t reqid, count, length; uint8_t last_prefix[16]; } entries[64];
	uint32_t unsupported[64];   /* per entry: answered "unsupported" by the generic dispatch */
} host_audit;
static void rf_event(unsigned id, const uint8_t *data, size_t length, void *context)
{
	(void)context;
	if (id == 0 && length == 3 && data[0] == 0) {
		ar_telemetry_link(&telemetry, data[1] == 2);
	}
	(void)ar_event_publish(&events, id, data, length);
}
static size_t transmit_peek(uint8_t *scratch, size_t capacity, unsigned *port, void *context)
{
	(void)context;
	return ar_socket_tx_peek(&sockets, scratch, capacity, port);
}
static size_t transmit_peek_at(unsigned *port, size_t offset, uint8_t *scratch, size_t capacity, void *context)
{
	(void)context;
	return ar_socket_tx_peek_at(&sockets, port, offset, scratch, capacity);
}
static size_t transmit_available(unsigned port, void *context)
{
	(void)context;
	return ar_socket_tx_available(&sockets, port);
}
static void transmit_commit(unsigned port, size_t length, void *context)
{
	(void)context;
	ar_socket_tx_commit(&sockets, port, length);
}
static void received_payload(unsigned slot, unsigned port, const uint8_t *data, size_t length, void *context)
{
	(void)context;
	ar_telemetry_receive(&telemetry, slot, port, data, length);
	size_t accepted = ar_socket_rx_feed(&sockets, slot, port, data, length);
	if (slot || !port || port > AR_SOCKET_RX_PORTS) { return; }
	unsigned i = port - 1;
	if (!sockets.ports[i].capacity || sockets.ports[i].closing) { return; }
	if (sockets.ports[i].used > socket_flow[i].high_water) { socket_flow[i].high_water = sockets.ports[i].used; }
	if (length > socket_flow[i].largest_feed) { socket_flow[i].largest_feed = length; }
	if (accepted != length) {
		const struct ar_sdio_stats *stats = ar_sdio_statistics();
		socket_flow[i].drop_calls++; socket_flow[i].last_drop_bytes = length - accepted;
		socket_flow[i].tx_room = ar_sdio_tx_room();
		socket_flow[i].active_ticks = ar_sdio_active_ticks();
		socket_flow[i].last_ticks = stats->tx_last_ticks;
		socket_flow[i].max_ticks = stats->tx_max_ticks;
		socket_flow[i].tx_packets = stats->tx_packets;
		socket_flow[i].rx_packets = stats->rx_packets;
	}
}
_Static_assert(sizeof(struct ar_rf_rx_info) <= AR_MAX_PAYLOAD, "RF diagnostics must fit one response");
#include <zephyr/fatal.h>
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	(void)esf;
	uint32_t cause, pc, value;
	__asm__ volatile("csrr %0, mcause" : "=r"(cause));
	__asm__ volatile("csrr %0, mepc" : "=r"(pc));
	__asm__ volatile("csrr %0, mtval" : "=r"(value));
	if (bus) { bus->record_fault(reason, cause, pc, value); }
	for (;;) { arch_nop(); }
}
static uint8_t transformed[AR_MAX_PAYLOAD];
static uint32_t observed_unary(uint32_t input)
{
	/* Observed register-only instruction in socket key construction. Its
	 * arithmetic is challenged on hardware before accepting an ISA model. */
	register uint32_t value __asm__("a5") = input;
	__asm__ volatile(".4byte 0xad8787f7" : "+r"(value));
	return value;
}
static uint32_t observed_unary_s8(uint32_t input)
{
	/* Exact word at stock socket dispatch 0x2407fc. Test this register form
	 * independently before extending the partial analysis model. */
	register uint32_t value __asm__("s8") = input;
	__asm__ volatile(".4byte 0xad8c0c77" : "+r"(value));
	return value;
}
static uint32_t observed_reverse_s0(uint32_t input)
{
	/* Exact stock measurement-helper word at 0x22d28a. */
	register uint32_t value __asm__("s0") = input;
	__asm__ volatile(".4byte 0x8204140b" : "+r"(value));
	return value;
}
#else

/* UART is the emulator/diagnostic transport, not the AR8030 SDIO peripheral. */
static const struct device *const uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#endif
static struct ar_parser parser;
static uint8_t reply[AR_MAX_FRAME];
#ifdef CONFIG_SOC_AR8030_BRINGUP
uint32_t ar_rf_host_ap_time(void);
int ar_rf_host_mcs_row(const uint8_t row[16]);
int ar_rf_host_bandwidth(const uint8_t *payload, size_t length);
int ar_rf_host_frame_mode(const uint8_t *payload, size_t length);
int ar_rf_host_mcs(const uint8_t *in, size_t length, uint8_t out[8]);
int ar_rf_host_link_info(const uint8_t *in, size_t length, uint8_t out[216]);
int ar_rf_host_power(uint32_t reqid, const uint8_t *in, size_t length, uint8_t *out, size_t *out_length);
int ar_rf_host_retx_config(const uint8_t *payload, size_t length);
int ar_rf_host_channel_info(uint8_t output[1028]);
int ar_rf_host_channel_set(unsigned order, const uint8_t *payload, size_t length);
/* Host bytes parsed between two RF polls (see the main loop). */
#define AR_FEED_PIECE 1024u
static uint32_t loop_stats[4], loop_previous;
void ar_rf_note_push_time(uint32_t us);
void ar_main_loop_statistics(uint32_t out[6])
{
	const struct ar_usb_stats *usb = ar_usb_statistics();
	out[0] = loop_stats[0]; out[1] = loop_stats[1]; out[2] = loop_stats[2]; out[3] = loop_stats[3];
	out[4] = usb->tx_packets; out[5] = usb->tx_queue_high_water;
}

#endif

static void receive(const struct ar_frame *frame, void *context)
{
	(void)context;
#ifdef CONFIG_SOC_AR8030_BRINGUP
	struct ar_frame socket_response;
	uint32_t diagnostic_reqid = ar_diagnostic_request_id(frame->reqid);
	if (diagnostic_reqid == AR_REQ_TELEMETRY_INFO) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid};
		if (frame->status || frame->length) { response.status = AR_STATUS_INVALID; }
		else {
			response.payload = (const uint8_t *)&telemetry.info;
			response.length = sizeof telemetry.info;
		}
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_SOCKET_FLOW_INFO && !frame->status && !frame->length) {
		/* Version, live SDIO state, cumulative transport stats, then four
		 * port records. Drop snapshots are lifetime diagnostics, not events. */
		uint32_t header[] = {1, ar_sdio_tx_room(), ar_sdio_active_ticks()};
		size_t n = sizeof header;
		memcpy(transformed, header, n);
		memcpy(transformed + n, ar_sdio_statistics(), sizeof(struct ar_sdio_stats));
		n += sizeof(struct ar_sdio_stats);
		memcpy(transformed + n, socket_flow, sizeof socket_flow); n += sizeof socket_flow;
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = transformed, .length = n};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
    if (frame->reqid == AR_REQ_SPECTATOR_STATE) {
        uint32_t state[8]; ar_rf_listener_state(state);
        struct ar_frame response={.reqid=frame->reqid,.msgid=frame->msgid,
            .status=(frame->status || frame->length)?AR_STATUS_INVALID:0,
            .payload=(const uint8_t *)state,.length=sizeof state};
        size_t size=ar_encode(reply,sizeof reply,&response);
        (void)bus->send(reply,size); return;
    }
	if (frame->reqid == AR_REQ_SPECTATOR_CAPS) {
		/* LST1 envelope, API revision 2 (LST2 following); initial-width bitmap (40 MHz).
		 * Read-only even while a normal pilot link is running. */
		uint8_t caps[28] = {'L','S','T','1',2,0,0,0,0x20,0,0,0};
		const struct ar_rf_rx_info *rx = ar_rf_receive_information();
		const uint32_t values[] = {rx->dev_state,rx->tx_packets,rx->tx_preencode,rx->sync_ack_submitted};
		for (unsigned i=0;i<4;i++) for (unsigned j=0;j<4;j++) caps[12+i*4+j]=values[i]>>(j*8);
		struct ar_frame response = {.reqid=frame->reqid, .msgid=frame->msgid,
			.status=(frame->status || frame->length) ? AR_STATUS_INVALID : 0,
			.payload=caps, .length=sizeof caps};
		size_t size = ar_encode(reply,sizeof reply,&response);
		(void)bus->send(reply,size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_AP_TRACE && !frame->status && frame->length == 1) {
		/* One byte: let the trace run on, never freezing again (1), or
		 * freezing only when the AP stops transmitting (2). */
		ar_rf_ap_trace_unfreeze(frame->payload[0]);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_AP_TRACE && !frame->status && frame->length == 3 && frame->payload[0] == 0x50) {
		/* Bench: {0x50, entry, byte} moves the broadcast's pre-encode. */
		ar_rf_ap_trigger(frame->payload[1], frame->payload[2]);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_AP_TRACE && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_ap_trace_information(), .length = sizeof(struct ar_rf_ap_trace)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_AP_RESET && !frame->status && frame->length == 1) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.status = ar_rf_ap_rearm(frame->payload[0])};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_AP_RESET && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.status = ar_rf_ap_decoder_reset()};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_MCS_CEILING && !frame->status && frame->length == 1) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.status = ar_rf_host_mcs_ceiling(frame->payload, frame->length)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_FRAME_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_frame_information(), .length = sizeof(struct ar_rf_frame_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_AP_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_ap_information(), .length = sizeof(struct ar_rf_ap_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_CAPTURE_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_capture_information(), .length = sizeof(struct ar_rf_capture_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_SCAN_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_scan_information(), .length = sizeof(struct ar_rf_scan_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_RECOVERY_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_recovery_information(), .length = sizeof(struct ar_rf_recovery_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_LIFECYCLE_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_lifecycle_information(), .length = sizeof(struct ar_rf_lifecycle_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_FLASH_READ && !frame->status && frame->length == 6) {
		const uint8_t *p = frame->payload;
		uint32_t offset = p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
		size_t length = p[4] | (size_t)p[5] << 8;
		int status = length > 1024 ? AR_STATUS_INVALID :
			ar_factory_device_flash_read(ar_boot_information()->flash_capacity, offset, transformed, length);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.status = status, .payload = transformed, .length = status ? 0 : length};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_BOOT_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_boot_information(), .length = sizeof(struct ar_boot_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_HOST_AUDIT && !frame->status && !frame->length) {
		host_audit.event_drops = events.dropped;
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)&host_audit, .length = sizeof host_audit};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid >> 24 != 0x0c) {
		unsigned i;
		for (i = 0; i < host_audit.count; i++) { if (host_audit.entries[i].reqid == frame->reqid) { break; } }
		if (i == 64) { host_audit.overflow++; }
		else {
			if (i == host_audit.count) { host_audit.count++; host_audit.entries[i].reqid = frame->reqid; }
			host_audit.entries[i].count++; host_audit.entries[i].length = frame->length;
			memset(host_audit.entries[i].last_prefix, 0, 16);
			if (frame->length) { memcpy(host_audit.entries[i].last_prefix, frame->payload, frame->length < 16 ? frame->length : 16); }
		}
	}
	if ((frame->reqid >> 24) == 0x0b) {
		int status = frame->status ? AR_STATUS_INVALID : ar_rf_host_control(frame->reqid & 0xffffff, frame->length);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x01000005)) {
		/* GET 5: distance per slot (stock 0x221866). */
		int status = frame->status ? AR_STATUS_INVALID : ar_rf_host_distance(frame->payload, frame->length, transformed);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.status = status, .payload = transformed, .length = status ? 0 : 32};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x0100000a)) {
		int status = frame->status || frame->length ? AR_STATUS_INVALID : ar_rf_host_channel_info(transformed);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.status = status, .payload = transformed, .length = status ? 0 : 1028};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x02000005) || frame->reqid == UINT32_C(0x02000006) || frame->reqid == UINT32_C(0x02000023)) {
		int status = frame->status ? AR_STATUS_INVALID :
			ar_rf_host_channel_set(frame->reqid & 0xff, frame->payload, frame->length);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x01000006) || frame->reqid == UINT32_C(0x0100006b)) {
		unsigned mcs = frame->reqid == UINT32_C(0x01000006);
		int status = frame->status ? AR_STATUS_INVALID : mcs ?
			ar_rf_host_mcs(frame->payload, frame->length, transformed) :
			ar_rf_host_link_info(frame->payload, frame->length, transformed);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.status = status, .payload = transformed, .length = status ? 0 : mcs ? 8 : 216};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x0200001b)) {
		int status = frame->status ? AR_STATUS_INVALID : ar_rf_host_uplink_mcs_range(frame->payload, frame->length);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x01000011)) {
		static uint8_t out[AR_SOCKET_RING_INFO_BYTES];
		int status = frame->status ? AR_STATUS_INVALID : ar_socket_rx_ring_info(&sockets, frame->payload, frame->length, out);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status,
			.payload = out, .length = status ? 0 : sizeof out};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x010000c8)) {
		uint8_t out[256];
		int status = frame->status ? AR_STATUS_INVALID : ar_rf_host_project_get(frame->payload, frame->length, out);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status,
			.payload = out, .length = status ? 0 : sizeof out};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x020000c8)) {
		int status = frame->status ? AR_STATUS_INVALID : ar_rf_host_project(frame->payload, frame->length);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x02000008) || frame->reqid == UINT32_C(0x02000009) ||
	    frame->reqid == UINT32_C(0x0200006e) || frame->reqid == UINT32_C(0x01000071) ||
	    frame->reqid == UINT32_C(0x01000007) || frame->reqid == UINT32_C(0x01000008) || frame->reqid == UINT32_C(0x01000009)) {
		size_t out_length = 0;
		int status = frame->status ? AR_STATUS_INVALID :
			ar_rf_host_power(frame->reqid, frame->payload, frame->length, transformed, &out_length);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.status = status, .payload = transformed, .length = status ? 0 : out_length};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x02000010)) {
		/* SET 16: TDD frame mode (0 standard, 2 short). */
		int status = frame->status ? AR_STATUS_INVALID : ar_rf_host_frame_mode(frame->payload, frame->length);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x02000016) || frame->reqid == UINT32_C(0x02000026)) {
		int status = frame->status ? AR_STATUS_INVALID : frame->reqid == UINT32_C(0x02000016) ?
			ar_rf_host_bandwidth(frame->payload, frame->length) : ar_rf_host_retx_config(frame->payload, frame->length);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x02000024)) {
		/* SET 36: one MCS policy row from the host (0x24ea0c). */
		int status = frame->status || frame->length != 16 ? AR_STATUS_INVALID :
			ar_rf_host_mcs_row(frame->payload);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x02000002) || frame->reqid == UINT32_C(0x02000003) ||
	    frame->reqid == UINT32_C(0x01000001)) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid};
		if (frame->status) { response.status = AR_STATUS_INVALID; }
		else if (frame->reqid == UINT32_C(0x02000002)) {
			response.status = ar_rf_host_pairing(frame->payload, frame->length);
		} else if (frame->reqid == UINT32_C(0x02000003)) {
			response.status = ar_rf_host_peer(frame->payload, frame->length);
		} else {
			response.status = frame->length ? AR_STATUS_INVALID : ar_rf_host_pairing_result(transformed);
			if (!response.status) { response.payload = transformed; response.length = 164; }
		}
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x02000004)) {
		int status = frame->status ? AR_STATUS_INVALID : ar_rf_host_candidates(frame->payload, frame->length);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (ar_event_request(&events, frame, &socket_response)) {
		size_t size = ar_encode(reply, sizeof reply, &socket_response);
		(void)bus->send(reply, size);
		/* A subscription to event 14 (TX power) gets the state as it is:
		 * a host that came up after the last change learns it at once. */
		if (frame->reqid == UINT32_C(0x0301000e) && !socket_response.status) { ar_rf_power_events(); }
		return;
	}
	if (frame->reqid == UINT32_C(0x02000064)) {
		/* SET 100: baseband register write, the mirror of GET 100: u16
		 * offset from 0x0a110000, u16 length (1..256), then the bytes. Each
		 * touched word is read, the given bytes replaced, written once. */
		int status = AR_STATUS_INVALID;
		if (!frame->status && frame->length >= 4) {
			uint32_t offset = frame->payload[0] | (uint32_t)frame->payload[1] << 8;
			uint32_t count = frame->payload[2] | (uint32_t)frame->payload[3] << 8;
			if (count && count <= 256 && offset + count <= 0x1000 && frame->length >= 4 + count) {
				uint32_t first = offset & ~3u, end = (offset + count + 3) & ~3u;
				for (uint32_t a = first; a < end; a += 4) {
					volatile uint32_t *r = (volatile uint32_t *)(uintptr_t)(UINT32_C(0x0a110000) + a);
					uint32_t word = *r;
					for (unsigned b = 0; b < 4; b++) {
						uint32_t at = a + b;
						if (at >= offset && at < offset + count) {
							word = (word & ~(UINT32_C(255) << (8 * b))) | (uint32_t)frame->payload[4 + at - offset] << (8 * b);
						}
					}
					*r = word;
				}
				status = 0;
			}
		}
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x01000064)) {
		unsigned malformed = frame->status || frame->length != 4;
		/* Stock allocates the declared 256-byte reply even on a range error.
		 * Initialize it so errors cannot return a previous response's bytes. */
		memset(transformed, 0, 256);
		int status = malformed ? AR_STATUS_INVALID :
			ar_rf_host_registers(frame->payload[0] | (unsigned)frame->payload[1] << 8,
				frame->payload[2] | (unsigned)frame->payload[3] << 8, transformed);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.status = status, .payload = transformed, .length = malformed ? 0 : 256};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x0100000c)) {
		uint8_t output[4];
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid};
		if (frame->status || frame->length) { response.status = AR_STATUS_INVALID; }
		else {
			uint32_t value = ar_rf_host_ap_time();
			for (unsigned i = 0; i < 4; i++) { output[i] = value >> (8 * i); }
			response.payload = output; response.length = sizeof output;
		}
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == UINT32_C(0x01000000)) {
		int status = frame->status || frame->length != 2 ? AR_STATUS_INVALID :
			ar_rf_host_status(frame->payload[0] | (uint16_t)frame->payload[1] << 8, transformed);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.status = status, .payload = transformed, .length = status ? 0 : 300};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_TX_PAYLOAD_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_tx_payload_information(), .length = sizeof(struct ar_rf_tx_payload_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_SOCKET_TX_INFO && !frame->status && !frame->length) {
		uint32_t words[AR_SOCKET_RX_PORTS * 12 + 6];
		for (unsigned port = 0; port < AR_SOCKET_RX_PORTS; port++) {
			const struct ar_socket_rx_port *s = &sockets.ports[port];
			uint32_t values[] = {s->flags, s->tx_capacity, s->tx_used, s->pending_count,
				(uint32_t)s->position, (uint32_t)(s->position >> 32), s->enqueued, s->completed,
				s->submitted, s->rejected, s->tx_head, s->msgid};
			for (unsigned i = 0; i < 12; i++) { words[port * 12 + i] = values[i]; }
		}
		const uint32_t totals[6] = {sockets.tx_writes, sockets.tx_write_bytes, sockets.tx_full,
			sockets.tx_out_of_order, sockets.tx_completed, sockets.tx_peak};
		memcpy(words + AR_SOCKET_RX_PORTS * 12, totals, sizeof totals);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)words, .length = sizeof words};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_MCS_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_mcs_information(), .length = sizeof(struct ar_rf_mcs_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_SYNC_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_sync_information(), .length = sizeof(struct ar_rf_sync_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_TRAFFIC_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_traffic_information(), .length = sizeof(struct ar_rf_traffic_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_TX_MCS_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_tx_mcs_information(), .length = sizeof(struct ar_rf_tx_mcs_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_TX_HISTORY_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_tx_history_information(), .length = sizeof(struct ar_rf_tx_history_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_PHY_TRACE && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_phy_trace_information(), .length = sizeof(struct ar_rf_phy_trace)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_PHY_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)ar_rf_phy_information(), .length = sizeof(struct ar_rf_phy_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	int socket_handled = ar_socket_rx_request(&sockets, frame, &socket_response);
	if (socket_handled) {
		if (socket_handled == 1) {
			size_t size = ar_encode(reply, sizeof reply, &socket_response);
			(void)bus->send(reply, size);
		}
		return;
	}
	if (diagnostic_reqid == AR_REQ_SOCKET_CLOSE_INFO && !frame->status && !frame->length) {
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)&sockets.close_info, .length = sizeof sockets.close_info};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_SOCKET_RX_INFO && !frame->status && !frame->length) {
		uint32_t words[AR_SOCKET_RX_PORTS * 8];
		for (unsigned port = 0; port < AR_SOCKET_RX_PORTS; port++) {
			const struct ar_socket_rx_port *s = &sockets.ports[port];
			uint32_t values[] = {s->capacity, s->used, s->msgid, s->offered,
				s->accepted, s->delivered, s->dropped, s->head};
			for (unsigned i = 0; i < 8; i++) { words[port * 8 + i] = values[i]; }
		}
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid,
			.payload = (const uint8_t *)words, .length = sizeof words};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == AR_REQ_RF_REGISTERS && frame->status == 0 && frame->length == 4) {
		unsigned offset = frame->payload[0] | (unsigned)frame->payload[1] << 8;
		unsigned length = frame->payload[2] | (unsigned)frame->payload[3] << 8;
		int status = ar_rf_read_registers(offset, length, transformed);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status,
			.payload = transformed, .length = status ? 0 : length};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == AR_REQ_RF_PLL && frame->status == 0 && frame->length == 4) {
		uint32_t frequency = 0;
		for (unsigned byte = 0; byte < 4; byte++) frequency |= (uint32_t)frame->payload[byte] << (8 * byte);
		uint64_t pll = 0;
		int status = ar_rf_pll_word(frequency, &pll);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status,
			.payload = (const uint8_t *)&pll, .length = sizeof pll};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (diagnostic_reqid == AR_REQ_RF_RX_INFO) {
		struct ar_frame response;
		ar_rx_diagnostic_read(frame, (const uint8_t *)ar_rf_receive_information(),
				      sizeof(struct ar_rf_rx_info), &response);
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid >= AR_REQ_RF_PROBE && frame->reqid <= AR_REQ_RF_MONITOR) {
		bus->record_progress(0xb000 | (frame->reqid & 255), frame->length, frame->msgid, 0);
	}
	if (frame->reqid == AR_REQ_RF_MONITOR && frame->status == 0 && frame->length >= 16) {
		uint32_t frequency = 0, milliseconds = 0;
		for (unsigned byte = 0; byte < 4; byte++) {
			frequency |= (uint32_t)frame->payload[byte] << (8 * byte);
			milliseconds |= (uint32_t)frame->payload[byte + 4] << (8 * byte);
		}
		int status = frame->payload[12] == 76 ?
			ar_rf_prepare_host(frequency, milliseconds, frame->payload[8], frame->payload + 16, frame->length - 16) :
			ar_rf_monitor_plan(frequency, milliseconds, frame->payload[8], frame->payload[12],
				frame->payload + 16, frame->length - 16);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status,
			.payload = (const uint8_t *)ar_rf_receive_information(), .length = sizeof(struct ar_rf_rx_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid == AR_REQ_RF_PINS && frame->status == 0) {
		int status = ar_rf_setup_pins(frame->payload, frame->length);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status,
			.payload = (const uint8_t *)ar_rf_information(), .length = sizeof(struct ar_rf_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size); return;
	}
	if (frame->reqid >= AR_REQ_RF_PROBE && frame->reqid <= AR_REQ_RF_PSRAM && frame->status == 0) {
		int status;
		if (frame->reqid == AR_REQ_RF_PROBE && frame->length == 0) { status = ar_rf_probe(); }
		else if (frame->reqid == AR_REQ_RF_DEFAULTS) { status = ar_rf_load_defaults(frame->payload, frame->length); }
		else if (frame->reqid == AR_REQ_RF_CALIBRATE && frame->length == 0) { status = ar_rf_calibrate(); }
		else if (frame->reqid == AR_REQ_RF_PSRAM && frame->length == 0) { status = ar_rf_setup_psram(); }
		else { status = AR_STATUS_INVALID; }
		bus->record_progress(0xb100 | (frame->reqid & 255), status, ar_rf_information()->phase, 0);
		struct ar_frame response = {.reqid = frame->reqid, .msgid = frame->msgid, .status = status,
			.payload = (const uint8_t *)ar_rf_information(), .length = sizeof(struct ar_rf_info)};
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size);
		return;
	}
	if ((frame->reqid == AR_REQ_OP_AD8 || frame->reqid == AR_REQ_OP_AD8_S8 || frame->reqid == AR_REQ_OP_REV_S0) &&
	    frame->status == 0 && frame->length % 4 == 0) {
		for (uint32_t i = 0; i < frame->length; i += 4) {
			uint32_t value = frame->payload[i] | (uint32_t)frame->payload[i + 1] << 8 |
				(uint32_t)frame->payload[i + 2] << 16 | (uint32_t)frame->payload[i + 3] << 24;
			value = frame->reqid == AR_REQ_OP_AD8 ? observed_unary(value) :
				frame->reqid == AR_REQ_OP_AD8_S8 ? observed_unary_s8(value) : observed_reverse_s0(value);
			for (unsigned byte = 0; byte < 4; byte++) { transformed[i + byte] = value >> (8 * byte); }
		}
		struct ar_frame response = *frame;
		response.payload = transformed;
		size_t size = ar_encode(reply, sizeof reply, &response);
		(void)bus->send(reply, size);
		return;
	}
	if (frame->reqid == AR_REQ_TX_BURST && frame->status == 0) {
		/* Project-only transport test: four independently queued nonce echoes.
		 * It injects no RF/video data and does not implement a vendor socket. */
		struct ar_frame response = *frame;
		for (unsigned i = 0; i < bus->tx_depth; i++) {
			response.msgid = frame->msgid + i;
			size_t size = ar_encode(reply, sizeof reply, &response);
			if (bus->send(reply, size)) { break; }
		}
		return;
	}
#endif
	size_t length;
	ar_dispatch(frame, reply, &length);
#ifdef CONFIG_SOC_AR8030_BRINGUP
	if (length >= AR_HEADER_SIZE) {
		int32_t status = (int32_t)((uint32_t)reply[13] << 24 | (uint32_t)reply[14] << 16 | (uint32_t)reply[15] << 8 | reply[16]);
		for (unsigned i = 0; status == AR_STATUS_UNSUPPORTED && i < host_audit.count; i++) {
			if (host_audit.entries[i].reqid == frame->reqid) { host_audit.unsupported[i]++; break; }
		}
	}
	(void)bus->send(reply, length);
#else
	for (size_t i = 0; i < length; i++) { uart_poll_out(uart, reply[i]); }
#endif
}

int main(void)
{
#ifdef CONFIG_SOC_AR8030_BRINGUP
	/* Stock 0x2174b0 runs with the caches on: dcache.iall, mhcr |= 0x103e,
	 * icache.iall, mhcr |= 1 (T-Head: IE bit 0, DE 1, WA 2, WB 3, RS 4,
	 * BPE 5, bit 12). Data cache maintenance around DMA
	 * is handled by ar_cache.h at every engine hand-off (RF receive buffers,
	 * DEV transmit packets, USB and SDIO buffers), so both caches, the
	 * return stack and branch prediction go on. With everything off, copying
	 * a 4 KiB frame took 1.4 ms and receiving one 1.1 ms (2026-10-15). */
	__asm__ volatile("fence\n\tfence\n\t.word 0x0020000b\n\tcsrrs zero, 0x7c1, %0\n\t"
		"fence\n\tfence\n\tfence\n\tfence\n\t.word 0x0100000b\n\tcsrrs zero, 0x7c1, %1\n\tfence\n\tfence.i"
		:: "r"(0x103eu), "r"(1u) : "memory");
	/* Diagnostic polling must not enter an unported CLIC ISR when host-side
	 * function enable or DMA completion asserts the controller's CPU IRQ. */
	(void)arch_irq_lock();
	bus = ar_transport_detect();
#ifdef AR_USB_PULSE_DIAG
	/* Read-out diagnostic: no loop, the core state is clocked out over the
	 * connect/disconnect line (see ar_usb_pulse_diag). */
	extern void ar_usb_pulse_diag(void);
	bus->init();
	ar_usb_pulse_diag();
#endif
#ifdef AR_USB_PROBE_ONLY
	/* Enumeration-only diagnostic: bring up the transport and do nothing but
	 * service it, so a USB enumeration failure is isolated from the radio
	 * bring-up and the RF poll loop. No radio, no sockets. */
	bus->init();
	for (;;) { const uint8_t *d; (void)bus->receive(&d); }
#endif
	/* USB needs EP0 answered within 5 s of connecting, so connect after the
	 * radio bring-up below; SDIO tolerated the delay. */
	if (!bus->init_after_boot) { bus->init(); }
	ar_socket_rx_init(&sockets);
	sockets.yield = ar_rf_poll;
	ar_events_init(&events);
	ar_telemetry_init(&telemetry, &events);
	ar_rf_set_event_callback(rf_event, NULL);
	ar_rf_set_payload_callback(received_payload, NULL);
	ar_rf_set_tx_callbacks(transmit_peek, transmit_commit, NULL);
	ar_rf_set_tx_staging(transmit_peek_at, transmit_available);
#ifdef AR_BOOT_EMBEDDED
	int boot_status = ar_boot_embedded_profile.plan ? ar_boot_prepare(&ar_boot_embedded_profile) :
		ar_boot_prepare_loader(&ar_boot_embedded_profile, (const char *)(uintptr_t)0x200000, 16384);
	bus->record_progress(0xc000, ar_boot_information()->stage, boot_status,
		ar_boot_information()->default_bytes);
	/* An air unit's radio (AP04 plan) starts by itself: its host never sends
	 * init/start, the stock AP is on the air as soon as it is loaded. */
	if (!boot_status && ar_rf_ap_role()) {
		/* The loader path is already initialized (init answers -7, as it
		 * does to a ground's backend); start is what matters. */
		int init = ar_rf_host_control(2, 0), start = ar_rf_host_control(0, 0);
		bus->record_progress(0xc001, (uint32_t)init, (uint32_t)start, 0);
	}
#endif
	if (bus->init_after_boot) { bus->init(); }
	unsigned completion_first = 1;
	for (;;) {
		uint32_t loop_started = *(volatile uint32_t *)0xe000bff8u;
		if (loop_stats[0]) { uint32_t us = (loop_started - loop_previous) / 40; if (us > loop_stats[3]) { loop_stats[3] = us; } }
		loop_previous = loop_started;
		loop_stats[0]++;
		const uint8_t *data;
		size_t length = bus->receive(&data);
		uint32_t stage = *(volatile uint32_t *)0xe000bff8u;
		ar_rf_note_host_stage(0, (stage - loop_started) / 40);
		if (length) {
			/* In pieces, the radio served between them: a slice's write
			 * (~7 KB) took over 200 us to parse in one go, and the
			 * pre-encode waiting behind it filled its block up to ~0.5 ms
			 * late - 8.6% of them, 1.45% of frames sent again where stock
			 * sends none (bench 2026-11-02). The parser keeps its state
			 * across calls. */
			for (size_t done = 0; done < length; ) {
				size_t piece = length - done > AR_FEED_PIECE ? AR_FEED_PIECE : length - done;
				ar_feed(&parser, data + done, piece, receive, NULL);
				done += piece;
				if (done < length) { ar_rf_poll(); }
			}
			uint32_t fed = *(volatile uint32_t *)0xe000bff8u;
			ar_rf_note_host_stage(1, (fed - stage) / 40); stage = fed;
		}
		else { arch_nop(); }
		ar_socket_tx_service(&sockets);
		ar_socket_close_service(&sockets);
		ar_rf_note_host_stage(2, (*(volatile uint32_t *)0xe000bff8u - stage) / 40);
		ar_rf_poll();
		struct ar_frame frame;
		/* Leave one entry for a reply to an already granted host request. */
		if (bus->tx_room() > 1) {
			int ready;
			/* Bound copy/framing work between RF polls; the bus sets the
			 * frame size its host drains best. */
			size_t chunk = bus->chunk;
			/* Copy directly to the final frame payload. Its contents remain
			 * owned here while RF appends to the independent endpoint queue. */
			uint8_t *payload = reply + AR_HEADER_SIZE;
			unsigned event = ar_event_peek(&events, payload, AR_MAX_PAYLOAD, &frame);
			unsigned close = !event && ar_socket_close_peek(&sockets, &frame);
			if (event || close) { ready = 1; }
			else if (completion_first) {
				ready = ar_socket_tx_completion(&sockets, payload, &frame) ||
					ar_socket_rx_peek(&sockets, payload, chunk, &frame);
			} else {
				ready = ar_socket_rx_peek(&sockets, payload, chunk, &frame) ||
					ar_socket_tx_completion(&sockets, payload, &frame);
			}
			if (ready) {
				/* The peek owns an immutable scratch copy. RF may append to
				 * the endpoint while we prepare SDIO's copy, but cannot move
				 * its head; only the commit below consumes these bytes. */
				ar_rf_poll();
				uint32_t push_started = *(volatile uint32_t *)0xe000bff8u;
				size_t size = ar_encode(reply, sizeof reply, &frame);
				ar_rf_poll();
				int sent = bus->send(reply, size);
				ar_rf_note_push_time((*(volatile uint32_t *)0xe000bff8u - push_started) / 40);
				ar_rf_note_host_stage(3, (*(volatile uint32_t *)0xe000bff8u - push_started) / 40);
				if (!sent) {
					loop_stats[1]++;
					unsigned completion = (frame.reqid >> 16 & 255) == 1;
					if (event) { ar_event_commit(&events); }
					else if (close) { ar_socket_close_commit(&sockets, &frame); }
					else if (completion) { ar_socket_tx_complete(&sockets, &frame); }
					else { ar_socket_rx_commit(&sockets, &frame); }
					if (!event && !close) { completion_first = !completion; }
				} else { loop_stats[2]++; }
			}
		}
	}
#else
	if (!device_is_ready(uart)) { return -ENODEV; }
	for (;;) {
		unsigned char byte;
		if (uart_poll_in(uart, &byte) == 0) {
			ar_feed(&parser, &byte, 1, receive, NULL);
		} else {
			/* Polling bring-up also works before a hardware timer is ported. */
			arch_nop();
		}
	}
#endif
	return 0;
}
