/* SPDX-License-Identifier: MIT */
/* Synthetic peripheral behavior: real RF acceptance still requires hardware. */
#include "ar_pwr_auto.h"
#include "ar_rf_device.h"
#include "ar_mac.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
void ar_main_loop_statistics(uint32_t out[6]) { for (unsigned i = 0; i < 6; i++) { out[i] = 0; } }

static struct { uint32_t address, value; } registers[1024];
static unsigned used, writes;
static unsigned engine_resets;
static unsigned compensation_writes, board_prepare_writes;
static uint64_t ticks;
static unsigned emulate_otp;
static uint32_t otp_words[64];
static unsigned tear_timer_read;
static unsigned inject_receive, injected, inject_command, inject_header;
static unsigned check_header_bank, header_bank_reads, header_bank0_reads;
static unsigned inject_valid;
static unsigned delay_dma;
static unsigned complete_dma_reads;
static unsigned measurement_banks;
static unsigned trace_register_reads, volatile_register_reads, register_reads;
static uint32_t register_read_addresses[256];
static uint8_t valid_packet[256];
static unsigned forwarded_count, forwarded_slot, forwarded_port;
static size_t forwarded_length;
static uint8_t forwarded[256];
static unsigned event_count, event_ids[32];
static uint8_t event_data[32][4];
static unsigned last_event_id = 255, channel_events, bandwidth_events;
static uint8_t last_event[4], channel_event[4], bandwidth_event[4];
static unsigned distance_events; static uint8_t distance_event[4];
static void receive_event(unsigned id, const uint8_t *data, size_t length, void *context)
{
	/* Link (0, 3 bytes) and MCS (1); Open8030 also publishes channel (2) and
	 * bandwidth (12) changes, 4 bytes each. */
	assert(context == event_data && (id < 2 || id == 2 || id == 12 || id == 15) && length == (id ? 4u : 3u));
	if (id == 15) { distance_events++; memcpy(distance_event, data, 4); return; }   /* the distance, pushed */
	last_event_id = id; memset(last_event, 0, 4); memcpy(last_event, data, length);
	if (id == 2) { channel_events++; memcpy(channel_event, data, 4); }
	if (id == 12) { bandwidth_events++; memcpy(bandwidth_event, data, 4); }
	if (id >= 2) { return; }
	assert(event_count < 32);
	event_ids[event_count] = id;
	memcpy(event_data[event_count++], data, length);
}
static void receive_payload(unsigned slot, unsigned port, const uint8_t *data, size_t length, void *context)
{
	assert(context == forwarded && length <= sizeof forwarded);
	forwarded_count++; forwarded_slot = slot; forwarded_port = port; forwarded_length = length;
	memcpy(forwarded, data, length);
}
static unsigned transmit_calls, transmit_commits;
static size_t transmit_remaining;
static uint8_t transmit_source[140];
static size_t transmit_peek(uint8_t *scratch, size_t capacity, unsigned *port, void *context)
{
	assert(context == transmit_source); transmit_calls++;
	size_t count = transmit_remaining < capacity ? transmit_remaining : capacity;
	memcpy(scratch, transmit_source + sizeof transmit_source - transmit_remaining, count);
	*port = 3;
	return count;
}
static void transmit_commit(unsigned port, size_t length, void *context)
{
	assert(port == 3 && context == transmit_source && length <= transmit_remaining);
	transmit_commits++; transmit_remaining -= length;
}
static void put32(uint8_t *data, uint32_t value)
{
	for (unsigned i = 0; i < 4; i++) { data[i] = value >> (i * 8); }
}
static uint32_t *reg(uint32_t address)
{
	for (unsigned i = 0; i < used; i++) if (registers[i].address == address) return &registers[i].value;
	assert(used < 1024);
	registers[used].address = address;
	return &registers[used++].value;
}
uint32_t ar_rf_test_read(uint32_t address)
{
	if (trace_register_reads && address >= 0x0a110000 && address < 0x0a111000) {
		assert(register_reads < 256);
		register_read_addresses[register_reads++] = address;
		if (volatile_register_reads) { return *reg(address) ^ (register_reads * 0x01010101u); }
	}
	/* Model a DMA that finishes only while its RX producer is not reset.
	 * This distinguishes drain-before-reset from the old shutdown order. */
	if (address == 0xe0801164 && complete_dma_reads && !(*reg(0x0a11080c)&4)) {
		if (!--complete_dma_reads) { *reg(address) |= 1; }
	}
	if (measurement_banks && (address == 0x0a110e30 || address == 0x0a110e34)) {
		if (*reg(0x0a110808) & (1u << 27)) {
			return address == 0x0a110e30 ? 0x1234aaaa : 0xbbbb5678;
		}
		return address == 0x0a110e30 ? 0x07051111 : 0x22220309;
	}
	if (check_header_bank && ((address >= 0x0a110f04 && address <= 0x0a110f10) ||
	    (address >= 0x0a110f34 && address <= 0x0a110f3c))) {
		if (!(*reg(0x0a110bb0) & 0x80)) {
			header_bank0_reads++;
			return *reg(address) ^ 0xffffffff;
		}
		header_bank_reads++;
	}
	if (address == 0xe000bffc) {
		if (tear_timer_read) { ticks = (ticks | UINT32_MAX) - 20; tear_timer_read = 0; }
		return ticks >> 32;
	}
	if (address == 0xe000bff8) { ticks += 40; return (uint32_t)ticks; }
	if (address == 0xe0801170 && inject_receive && !injected && ar_rf_information()->phase == AR_RF_RECEIVING) {
		*reg(address) |= 1; injected = 1;
	}
	if (address == 0xe080114c && inject_command && ar_rf_information()->phase == AR_RF_RECEIVING) {
		*reg(address) |= 1; inject_command = 0;
	}
	if (address == 0xe0801168 && inject_header && ar_rf_information()->phase == AR_RF_RECEIVING) {
		*reg(address) |= 1; inject_header = 0;
	}
	return *reg(address);
}
void ar_rf_test_write(uint32_t address, uint32_t value)
{
	if (address == 0x0a11080c && (value & ~*reg(address) & 12)) { engine_resets++; }
	if (address == 0x0a110008 || address == 0x0a110058 || address == 0x0a11005c || address == 0x0a110080 || address == 0x0a110084) { board_prepare_writes++; }
	if (address == 0x0a110480 || address == 0x0a11048c ||
	    address == 0x0a1104a0 || address == 0x0a1104a4 || address == 0x0a1104a8) {
		compensation_writes++;
	}
	/* Reject an accidentally enlarged encoded BB offset, rather than treating
	 * unmapped MMIO as a harmless synthetic register. */
	assert((address >= 0x0a110000 && address < 0x0a111000) ||
	       (address >= 0x0a104000 && address < 0x0a104204) ||
	       address == 0x0a1060c8 ||
	       (address >= 0x0a10a02c && address < 0x0a10a034) || address == 0x0a090140 ||
	       (address >= 0x00140000 && address < 0x00140300) ||
	       address == 0xe0800008 || (address >= 0xe0801120 && address < 0xe0801180));
	*reg(address) = value; writes++;
	if (emulate_otp && address == 0x00140098) {
		assert(value < 64); *reg(0x0014009c) = otp_words[value]; *reg(0x001400a4) = 4;
	}
}
void ar_rf_test_start_dma(uint8_t *buffer, unsigned length)
{
	for (unsigned i = 0; i < length; i++) buffer[i] = 0x58 + i;
	if (inject_valid) { assert(length <= sizeof valid_packet); memcpy(buffer, valid_packet, length); }
	if (!delay_dma) { *reg(0xe0801164) |= 1; } /* RX DMA completion: source 13, CLIC vector 89. */
}
static void prepare_frame(void)
{
	const uint32_t commands[] = {(3u<<20)|(8u<<16),(3u<<20)|(13u<<16),0};
	for (unsigned i=0;i<3;i++) {
		*reg(0x0a110e6c)=commands[i]; inject_command=1; ar_rf_poll();
		assert(ar_rf_command_information()->cursor==(int)i);
	}
}
static void assert_channel_plls(uint32_t frequency)
{
	const struct ar_command_program *commands = ar_rf_command_information();
	assert(commands->count == 3 && commands->cursor == -1);
	const unsigned users[] = {8,11,0};
	for (unsigned i = 0; i < 3; i++) {
		assert(commands->entries[i].frequency == (i == 1 ? ar_rf_scan_information()->frequency_khz : frequency));
		assert((commands->entries[i].bits >> 13 & 3) == 2);
		uint32_t profile = *reg(0x0a110000 + (i == 2 ? 0xb00 : 0x9a0 + users[i] * 8));
		assert((commands->entries[i].bits >> 10 & 7) == (i == 2 ? profile & 7 : profile >> 8 & 7));
	}
	/* Stock layout: the broadcast receive bank (0x905) and the uplink bank
	 * (0x910) tune; 0x90a is the scan's; the rest park at 2100 MHz. */
	uint64_t expected, parked; assert(!ar_rf_pll_word(frequency, &expected) && !ar_rf_pll_word(2100000, &parked));
	const unsigned offsets[] = {0x900,0x905,0x90a,0x910,0x915,0x91a,0x920,0x925,
		0x92a,0x930,0x935,0x93a,0x940,0x945,0x94a,0x950,0x955,0x95a,0x960,0x965,0x96a};
	for (unsigned i = 0; i < sizeof offsets / sizeof offsets[0]; i++) {
		uint64_t actual = 0;
		for (unsigned b = 0; b < 5; b++) {
			unsigned offset = offsets[i] + b;
			actual = actual << 8 | ((*reg(0x0a110000 + (offset & ~3u)) >> (8 * (offset & 3))) & 255);
		}
		if (offsets[i] == 0x90a) {
			uint64_t scan_pll; assert(!ar_rf_pll_word(ar_rf_scan_information()->frequency_khz,&scan_pll));
			assert(actual == scan_pll);
		} else { assert(actual == (offsets[i] == 0x905 || offsets[i] == 0x910 ? expected : parked)); }
	}
}
static void receive_sync(uint8_t control[7], unsigned seal)
{
	if (seal) { control[0] = ar_rf_control_checksum(control); }
	uint32_t words[3] = {0};
	for (unsigned i = 0; i < 7; i++) {
		unsigned bit = 28 + i * 8, word = bit / 32, shift = bit % 32;
		words[word] |= (uint32_t)control[i] << shift;
		if (shift > 24) { words[word + 1] |= (uint32_t)control[i] >> (32 - shift); }
	}
	for (unsigned i = 0; i < 3; i++) { *reg(0x0a110f04 + 4 * i) = words[i]; }
	*reg(0x0a110f00) = 0x1096; *reg(0xe0801168) |= 1; ar_rf_poll();
}


/* ---- Host features on a linked DEV session ------------------------------ */
uint32_t ar_rf_test_link_rate(unsigned index, unsigned width, uint32_t frame_us);
static void irq(unsigned source)
{
	*reg(0xe0801000u + (source + (source < 4 ? 72 : 76)) * 4) |= 1; ar_rf_poll();
}
static void frame_header(void) { *reg(0x0a110f00) = 0x1096; irq(14); }
/* One uplink frame: preencode (19), then user 0's interleaver event (18),
 * which runs the PHY control slot's countdown. */
static void uplink_frame(void)
{
	irq(11);
	ticks += 6 * 40000ull; *reg(0x0a110f70) = 0; irq(19);
	*reg(0x0a110f68) = 0; irq(18);
	irq(11);
}
/* A PHY command from the AP: [checksum, ttl<<4|command, len<<1|ttl>>4, payload]. */
static void ap_command(unsigned command, unsigned ttl, const uint8_t *payload, unsigned length)
{
	uint8_t control[7] = {0, (uint8_t)((ttl & 15) << 4 | command), (uint8_t)(length << 1 | ttl >> 4)};
	memcpy(control + 3, payload, length);
	receive_sync(control, 1);
}
/* A ranging result at entry index (IRQ 6): 24 bits packed at 0xfd0/0xfdc. */
static void ranging(unsigned index, uint32_t entry)
{
	uint32_t base = index < 4 ? 0xfd0 : 0xfdc, at = base + 3 * (index & 3);
	for (unsigned b = 0; b < 3; b++) {
		uint32_t a = at + b, shift = (a & 3) * 8;
		uint32_t *w = reg(0x0a110000 + (a & ~3u));
		*w = (*w & ~(255u << shift)) | ((entry >> (8 * b)) & 255) << shift;
	}
	*reg(0x0a110fe8) = index; irq(6);
}
static int32_t distance_of(unsigned slots, unsigned slot)
{
	uint8_t in[1] = {(uint8_t)slots}, out[32];
	assert(!ar_rf_host_distance(in, 1, out));
	int32_t v = 0; memcpy(&v, out + slot * 4, 4);
	return v;
}
static void open8030_linked_tests(void)
{
	/* Host TX-only path control must not disable RX diversity. The direction
	 * byte is a mask, not a boolean; zero is a no-op. */
	const uint32_t path_addresses[] = {0x0a110000, 0x0a110c00, 0x0a110a20, 0x0a110a2c};
	uint32_t path_saved[4];
	for (unsigned i = 0; i < 4; i++) { path_saved[i] = *reg(path_addresses[i]); }
	*reg(path_addresses[0]) = 0x2201007c; *reg(path_addresses[1]) = 0x4cbeff56;
	*reg(path_addresses[2]) = 0x4003004a; *reg(path_addresses[3]) = 0;
	uint8_t path_request[8] = {0xcb,0,0,0,1,1,0,0};
	assert(!ar_rf_host_project(path_request, sizeof path_request));
	assert(*reg(path_addresses[0]) == 0x22010074);
	assert(*reg(path_addresses[1]) == 0x4cbeff56);
	assert(*reg(path_addresses[2]) == 0x4003004a && !*reg(path_addresses[3]));
	unsigned path_writes = writes;
	path_request[5] = 0;
	assert(!ar_rf_host_project(path_request, sizeof path_request) && writes == path_writes);
	path_request[5] = 4;
	assert(ar_rf_host_project(path_request, sizeof path_request) == -22 && writes == path_writes);
	path_request[5] = 2;
	assert(!ar_rf_host_project(path_request, sizeof path_request));
	assert(*reg(path_addresses[0]) == 0x22010054);
	assert(*reg(path_addresses[1]) == 0x4c3eff54);
	assert(*reg(path_addresses[2]) == 0x400300ca && *reg(path_addresses[3]) == 0xfff);
	path_request[6] = 1;
	assert(!ar_rf_host_project(path_request, sizeof path_request));
	assert(*reg(path_addresses[0]) == 0x22010074 && *reg(path_addresses[1]) == 0x4c3eff54);
	path_request[5] = 3;
	assert(!ar_rf_host_project(path_request, sizeof path_request));
	assert(*reg(path_addresses[0]) == 0x2201007c);
	for (unsigned i = 0; i < 4; i++) { *reg(path_addresses[i]) = path_saved[i]; }

	const struct ar_rf_rx_info *rx = ar_rf_receive_information();
	assert(rx->dev_state == 2 && rx->assigned_slot == 0);
	ar_rf_set_event_callback(receive_event, event_data);   /* off in this part of main() */

	/* GET 5: (raw - offset - the radios' delay) x 1.5 m, offset 0 in this
	 * plan, the link at 5 MHz here: 37 counts (185 / 5) + the chains' 37;
	 * DEV slot 0 only. */
	uint8_t in[2] = {0}, out[32];
	assert(ar_rf_host_distance(NULL, 1, out) == -22 && ar_rf_host_distance(in, 0, out) == -22);
	assert(distance_of(1, 0) == -1);                 /* no sample yet */
	ranging(0, 103); assert(distance_of(1, 0) == 43);
	/* Event 15 pushes it too, without a request: known, 43 m (the pass that
	 * takes a sample pushes before it; the next, 100 ms on, with it). */
	distance_events = 0; ticks += 1100 * 40000ull; ranging(0, 103);
	ticks += 150 * 40000ull; ar_rf_poll();
	assert(distance_events && distance_event[0] == 0 && distance_event[1] == 1 && distance_event[2] == 43 && !distance_event[3]);
	/* Stale after a second: pushed as not known. */
	ticks += 1100 * 40000ull; ar_rf_poll();
	assert(distance_event[1] == 0 && !distance_event[2]);
	ranging(0, 103); ticks += 150 * 40000ull; ar_rf_poll();
	ranging(5, 107); assert(distance_of(1, 0) == 46);  /* any index lands in slot 0; the mean of 103 and 107 */
	assert(distance_of(3, 1) == -1 && distance_of(1, 1) == 0); /* slot 1: -1 when asked, untouched if not */
	ranging(0, 117 | 1u << 20); ranging(0, 117 | 1u << 22); ranging(0, 117 | 1u << 23);
	assert(distance_of(1, 0) == 46);                 /* invalid flags dropped */
	ranging(0, 1u << 19 | 5); assert(distance_of(1, 0) == 0);   /* below zero: 0 m */
	ranging(0, 97); ranging(0, 937);                 /* a jump over 500 is an outlier */
	assert(distance_of(1, 0) == 4);                  /* mean of 103, 107, 0, 97 */
	for (unsigned i = 0; i < 8; i++) { ranging(0, 937); }
	assert(distance_of(1, 0) == 4);
	ranging(0, 937); assert(distance_of(1, 0) == 1294);   /* the tenth in a row is taken: the mean starts over */
	/* The mean is the last 400 ms: 100 ms later a sample of 941 averages in
	 * (939: 1297 m); 500 ms after that only the new ones count. */
	ticks += 100 * 40000ull; ranging(0, 941); assert(distance_of(1, 0) == 1297);
	ticks += 500 * 40000ull; ranging(0, 945); assert(distance_of(1, 0) == 1306);
	ticks += 1100 * 40000ull; assert(distance_of(1, 0) == -1); /* stale after 1 s */

	/* GET 10: the configured channels and the link's; SET 5 / SET 35. */
	uint8_t info[1028];
	assert(!ar_rf_host_channel_info(info));
	assert(info[0] == rx->channel_count && info[3] == rx->channel_index);
	for (unsigned i = 0; i < rx->channel_count; i++) {
		uint32_t f; memcpy(&f, info + 4 + 4 * i, 4); assert(f == rx->channels[i].frequency_khz);
	}
	uint8_t automatic[1] = {0};
	assert(!ar_rf_host_channel_set(5, automatic, 1));
	assert(!ar_rf_host_channel_info(info) && info[1] == 0);
	automatic[0] = 1; assert(!ar_rf_host_channel_set(5, automatic, 1));
	assert(ar_rf_host_channel_set(5, automatic, 0) == -22);
	uint8_t work[3] = {2, 0, 1};
	assert(!ar_rf_host_channel_set(35, work, 3));
	work[2] = (uint8_t)rx->channel_count; assert(ar_rf_host_channel_set(35, work, 3) == -22);
	work[0] = 129; assert(ar_rf_host_channel_set(35, work, 3) == -22);
	work[0] = 2; assert(ar_rf_host_channel_set(35, work, 2) == -22);
	assert(ar_rf_host_channel_set(99, work, 3) == -95);

	/* SET 6: a hop from the host - command 4 through the PHY control slot,
	 * TTL 16, both ends retune when it expires (event 2). */
	uint8_t hop[2] = {0, (uint8_t)rx->channel_count};
	assert(ar_rf_host_channel_set(6, hop, 2) == -6);
	hop[1] = (uint8_t)rx->channel_index; assert(!ar_rf_host_channel_set(6, hop, 2)); /* already there */
	unsigned from = rx->channel_index, to = from ? 0 : 1;
	hop[1] = (uint8_t)to;
	const uint32_t uplink_before = *reg(0x0a110910);
	assert(!ar_rf_host_channel_set(6, hop, 2));
	assert(ar_rf_host_channel_set(6, hop, 2) == -16);   /* one at a time */
	for (unsigned f = 0; f < 40 && rx->channel_index != to; f++) { uplink_frame(); }
	assert(rx->channel_index == to && channel_events && channel_event[2] == to && channel_event[3] == from);
	assert(rx->dev_state == 2);
	/* ... our uplink TX (PLL 0x910) with it: the AP listens for us on the
	 * new channel and advertises it as our return channel. */
	assert(*reg(0x0a110910) != uplink_before);

	/* An AP uplink-only move must preserve the downlink PLL and channel. */
	uint32_t downlink = *reg(0x0a110904), downlink_tail = *reg(0x0a110908);
	uint8_t move[3] = {0x50, (uint8_t)from, 0};
	ap_command(4, 1, move, 3);
	assert(rx->channel_index == to && channel_event[1] == 0);
	assert(*reg(0x0a110904) == downlink && *reg(0x0a110908) == downlink_tail);
	const struct ar_command_program *program = ar_rf_command_information();
	assert(program->entries[0].frequency == rx->channels[to].frequency_khz);
	assert(program->entries[2].frequency == rx->channels[from].frequency_khz);
	uint8_t status[300], frequency[4];
	assert(!ar_rf_host_status(0x101, status));
	put32(frequency, rx->channels[from].frequency_khz); assert(!memcmp(status + 20, frequency, 4));
	put32(frequency, rx->channels[to].frequency_khz); assert(!memcmp(status + 224, frequency, 4));
	move[1] = (uint8_t)to; ap_command(4, 1, move, 3);
	assert(program->entries[2].frequency == rx->channels[to].frequency_khz);
	uint32_t uplink = *reg(0x0a110910), uplink_tail = *reg(0x0a110914);
	move[1] = (uint8_t)from;
	/* Mode-0 ignores the mode-1/3 arm. A real RX command moves RX only. */
	move[0] = 0x80; ap_command(4, 1, move, 3);
	assert(rx->channel_index == to);
	move[0] = 0x20;
	ap_command(4, 1, move, 3);
	assert(rx->channel_index == from && channel_event[2] == from && rx->dev_state == 2);
	assert(*reg(0x0a110910) == uplink && *reg(0x0a110914) == uplink_tail);
	assert(program->entries[0].frequency == rx->channels[from].frequency_khz);
	assert(program->entries[2].frequency == rx->channels[to].frequency_khz);
	move[0] = 0x50; ap_command(4, 1, move, 3); /* Restore both for later fixtures. */

	/* AP command 5: the AP changes the broadcast width (code in bits 7:5). */
	uint8_t width[3] = {0, 4 << 5, 0};
	{
		unsigned samples = ar_rf_mcs_information()->samples;
		uint32_t fbc = *reg(0x0a110fbc), errors = *reg(0x0a110f28);
		ap_command(5, 3, width, 3);
		*reg(0x0a110fbc) = 8u << 20; *reg(0x0a110f28) = 1;
		/* The first announcement precedes application. Even these early
		 * transition errors must be excluded from the adaptation window. */
		for (unsigned i = 0; i < 3; i++) {
			ticks += 40000; frame_header(); irq(12);
		}
		assert(((*reg(0x0a1109e0) >> 8) & 7) == 2);
		assert(ar_rf_mcs_information()->samples == samples);
		*reg(0x0a110fbc) = fbc; *reg(0x0a110f28) = errors;
	}
	ap_command(5, 1, width, 3);
	/* ... applied at the frame's IRQ 5, as the AP does. */
	irq(5);
	assert(((*reg(0x0a1109e0) >> 8) & 7) == 4 && bandwidth_events && bandwidth_event[2] == 4);
	uint8_t accepted_width[3] = {0, 1, 4};
	assert(!ar_rf_host_bandwidth(accepted_width, sizeof accepted_width));
	/* Width-transition errors must not initiate a coding change while the
	 * peer is changing its TX profile. Measurements resume after settling. */
	{
		unsigned samples = ar_rf_mcs_information()->samples;
		unsigned requests = ar_rf_mcs_information()->requests;
		uint32_t fbc = *reg(0x0a110fbc), errors = *reg(0x0a110f28);
		*reg(0x0a110fbc) = 8u << 20; *reg(0x0a110f28) = 1;
		for (unsigned i = 0; i < 8; i++) {
			ticks += 40000; frame_header(); irq(12);
		}
		assert(ar_rf_mcs_information()->samples == samples);
		assert(ar_rf_mcs_information()->requests == requests);
		ticks += 501 * 40000ull; frame_header(); irq(12);
		assert(ar_rf_mcs_information()->samples == samples + 1);
		*reg(0x0a110fbc) = fbc; *reg(0x0a110f28) = errors;
	}

	/* Frame mode: SET 16 validates, a mode-2 request goes out as command 8
	 * and is applied at the frame header after it expires: the uplink slot
	 * 2234 -> 1251 us (0xa00), interleave 2 -> 1 (0xb00 [20:18]). */
	uint8_t mode[1] = {1};
	assert(ar_rf_host_frame_mode(NULL, 1) == -22 && ar_rf_host_frame_mode(mode, 0) == -22);
	assert(ar_rf_host_frame_mode(mode, 1) == -95);
	mode[0] = 0; assert(!ar_rf_host_frame_mode(mode, 1));          /* already standard */
	assert((*reg(0x0a110a00) & 0xffff) == 0x0da3);
	mode[0] = 2; assert(!ar_rf_host_frame_mode(mode, 1));
	/* The command slot busy (with this request itself here, at link-up with
	 * the width request): accepted and sent as the slot frees, not refused
	 * with -16 - the host does not ask again. */
	assert(!ar_rf_host_frame_mode(mode, 1));
	for (unsigned f = 0; f < 20 && (*reg(0x0a110a00) & 0xffff) != 0x07a3; f++) {
		uplink_frame(); frame_header();
	}
	assert((*reg(0x0a110a00) & 0xffff) == 0x07a3 && ((*reg(0x0a110b00) >> 18) & 7) == 1);
	assert(!(*reg(0x0a110800) & 4));                 /* engines re-armed, not left held */
	/* AP command 8 back to the standard frame: applied at the next header. */
	uint8_t standard[4] = {0};
	ap_command(8, 1, standard, 4); frame_header();
	assert((*reg(0x0a110a00) & 0xffff) == 0x0da3 && ((*reg(0x0a110b00) >> 18) & 7) == 2);
	/* Command 8 sub-type 6 (a power request: 18 dBm, down) is no frame
	 * switch, though its first byte's upper nibble reads as mode 2. */
	uint8_t power_request[4] = {0x26, 0x21, 0, 0};
	ap_command(8, 1, power_request, 4); frame_header(); frame_header();
	assert((*reg(0x0a110a00) & 0xffff) == 0x0da3 && ((*reg(0x0a110b00) >> 18) & 7) == 2);

	/* A frame change must not reset an RX producer whose DMA completion
	 * has not arrived. The pending change applies after ownership retires. */
	unsigned saved_inject = inject_valid, saved_receive = inject_receive;
	inject_receive = 0; inject_valid = 0; delay_dma = 1;
	*reg(0x0a110f88) = 1; *reg(0x0a110f90) = 1;
	irq(16);
	unsigned resets = engine_resets;
	uint8_t short_frame[4] = {2 << 4};
	ap_command(8, 1, short_frame, 4); frame_header();
	assert(engine_resets == resets && (*reg(0x0a110a00) & 0xffff) == 0x0da3);
	delay_dma = 0; irq(13); frame_header();
	assert(engine_resets > resets && (*reg(0x0a110a00) & 0xffff) == 0x07a3);
	ap_command(8, 1, standard, 4); frame_header();
	assert((*reg(0x0a110a00) & 0xffff) == 0x0da3);
	/* A deferred bandwidth change stays aligned to IRQ 5 after DMA retires. */
	delay_dma = 1; irq(16); resets = engine_resets;
	width[1] = 2 << 5; ap_command(5, 1, width, 3); irq(5);
	assert(engine_resets == resets && ((*reg(0x0a1109e0) >> 8) & 7) == 4);
	delay_dma = 0; irq(13);
	assert(engine_resets == resets && ((*reg(0x0a1109e0) >> 8) & 7) == 4);
	irq(5);
	assert(engine_resets > resets && ((*reg(0x0a1109e0) >> 8) & 7) == 2);
	/* Across 40 MHz the frame changes with the width, so a frame late is a
	 * different frame from the AP's. The frame's last packet, its DMA done
	 * but its IRQ 13 behind IRQ 5 in the poll pass, is finished first, not
	 * waited a frame for. */
	width[1] = 5 << 5; resets = engine_resets;
	/* The AP writes each header's TTL a frame ahead and moves at the first
	 * IRQ 5 after the TTL-2 header (its TTL-1 header goes out on the new
	 * width): TTL - 1 frames after the IRQ 5 before the header, on our
	 * timer. Not at the IRQ 5 before it; a late IRQ 5 still counts. */
	{
		uint64_t frame = (uint64_t)rx->search_frame_us * 40, phase = frame / 2, a = ticks + 2 * frame;
		ticks = a - frame; irq(5);
		ticks = a - frame + phase; ap_command(5, 3, width, 3);
		ticks = a; irq(5);                                   /* the IRQ 5 before TTL 2's header */
		ticks = a + phase; ap_command(5, 2, width, 3);
		assert(engine_resets == resets && ((*reg(0x0a1109e0) >> 8) & 7) == 2);
		ticks = a + frame + frame / 4;                       /* the next, its IRQ 5 handled late */
	}
	*reg(0x0a110f88) = 1; *reg(0x0a110f90) = 1; irq(16);
	assert(*reg(0xe0801164) & 1);                       /* IRQ 13 waits for the next pass */
	*reg(0xe0801000u + (5 + 76) * 4) |= 1; ar_rf_poll();
	assert(engine_resets > resets && ((*reg(0x0a1109e0) >> 8) & 7) == 5 && bandwidth_event[2] == 5);
	assert(*reg(0x0a110970) >> 16 == 0x09c4 && !(*reg(0xe0801164) & 1));
	/* Across 40 MHz the receiver is back from the re-arm, unmuted and
	 * released (the AP's first burst comes ~2.6 ms later, nothing of the
	 * old width after it). */
	assert(!(*reg(0x0a1109e4) >> 24 & 1) && !(*reg(0x0a110cfc) >> 23 & 1) && !(*reg(0x0a110bf8) >> 7 & 1));
	/* Back across 40 with an uplink packet whose TX end never came: it is
	 * retired, the switch is not put off. */
	*reg(0x0a110f70) = 0; *reg(0x0a110f5c) = 64 << 16; irq(19);
	assert(rx->tx_owned);
	width[1] = 4 << 5; ap_command(5, 1, width, 3); irq(5);
	assert(((*reg(0x0a1109e0) >> 8) & 7) == 4 && !rx->tx_owned && *reg(0x0a110970) >> 16 == 0x1250);
	width[1] = 2 << 5; ap_command(5, 1, width, 3); irq(5);
	assert(((*reg(0x0a1109e0) >> 8) & 7) == 2);
	/* With the AP's frame stamps (an Open8030 AP) 5/10/20 is timed too: on
	 * stock timing the ground moved a frame or two after the AP. */
	{
		irq(5); irq(5);
		uint8_t stamp[4] = {1, 2, 3, 4}; ap_command(10, 0, stamp, 4);
		uint64_t frame = (uint64_t)rx->search_frame_us * 40, phase = frame / 2, a = ticks + 2 * frame;
		width[1] = 4 << 5; resets = engine_resets;
		ticks = a; irq(5);
		ticks = a + phase; ap_command(5, 2, width, 3);
		assert(engine_resets == resets && ((*reg(0x0a1109e0) >> 8) & 7) == 2);
		ticks = a + frame; irq(5);
		assert(engine_resets > resets && ((*reg(0x0a1109e0) >> 8) & 7) == 4);
		assert(!(*reg(0x0a1109e4) >> 24 & 1) && !(*reg(0x0a110cfc) >> 23 & 1) && !(*reg(0x0a110bf8) >> 7 & 1));
		/* A TTL-3 header handled behind the next IRQ 5 (one poll pass), and
		 * the TTL-2 one lost: its frame began at the IRQ 5 before. */
		width[1] = 2 << 5; resets = engine_resets; a = ticks + 2 * frame;
		ticks = a - frame; irq(5);
		ticks = a; *reg(0xe0801000u + (5 + 76) * 4) |= 1; ap_command(5, 3, width, 3);
		assert(engine_resets == resets);
		ticks = a + frame; irq(5);
		assert(engine_resets > resets && ((*reg(0x0a1109e0) >> 8) & 7) == 2);
	}
	/* The AP's TX power announcement (command 9; command 8 sub-type 13 from
	 * an AP built in between): never a frame mode. 26 dBm is 0x1ad, whose
	 * (byte >> 4) & 7 a ground that took it as command 8's frame mode read
	 * as mode 2. */
	{
		const uint32_t mode = ar_rf_frame_information()->frame_mode;
		const uint8_t power[4] = {0xad, 0x01, 0, 0};
		ap_command(AR_PHY_PWR_ANNOUNCE, 5, power, 4); irq(5); irq(5);
		assert(ar_rf_frame_information()->frame_mode == mode);
		ap_command(8, 5, power, 4); irq(5); irq(5);
		assert(ar_rf_frame_information()->frame_mode == mode);
	}
	/* The AP's resend maps (command 11): 12 frames, two bits each, newest
	 * first, placed on our frames (the newest: the frame that ended at our
	 * IRQ 5 before the latest); a map again or an older one changes nothing,
	 * frames no map covered are not known. */
	{
		const struct ar_rf_frame_info *fi = ar_rf_frame_information();
		const uint32_t b0 = fi->retries, k0 = fi->retry_known_frames, s0 = fi->retry_stalls;
		/* newest 1 block, then 0, 2, none confirmed (3); index 0x25 */
		uint8_t r[4] = {0xe1, 0, 0, 5};
		ap_command(11, 2, r, 4); fi = ar_rf_frame_information();
		assert(fi->retry_frame == fi->frames - 1 && fi->retries == b0 + 3 && fi->retry_stalls == s0 + 1);
		assert(fi->retry_known_frames == k0 + 12 && (fi->retry_map[0] & 0xffffff) == 0xe1 && fi->retry_known[0] == 0xfff);
		ap_command(11, 2, r, 4); fi = ar_rf_frame_information();         /* the same map again */
		assert(fi->retry_known_frames == k0 + 12 && fi->retries == b0 + 3);
		/* six frames on: index 0x2b, the oldest new one 1 block; the older six overlap */
		for (unsigned i = 0; i < 6; i++) { irq(5); }
		r[0] = 0; r[1] = 0x04; r[2] = 0; r[3] = 0xb;   /* entry 5 = 1, entries 6..11 as before */
		ap_command(11, 2, r, 4); fi = ar_rf_frame_information();
		assert(fi->retry_frame == fi->frames - 1 && fi->retry_known_frames == k0 + 18 && fi->retries == b0 + 4);
		assert(((fi->retry_map[0] >> 10) & 3) == 1 && ((fi->retry_map[0] >> 12) & 3) == 1 && ((fi->retry_map[0] >> 18) & 3) == 3);
		assert(fi->retry_known[0] == 0x3ffff);
		/* thirty frames on (the channel busy): 12 known, the 18 before them not */
		for (unsigned i = 0; i < 30; i++) { irq(5); }
		r[0] = r[1] = r[2] = 0; r[3] = 0x9;               /* index 0x49 */
		ap_command(11, 4, r, 4); fi = ar_rf_frame_information();
		assert(fi->retry_known_frames == k0 + 30 && fi->retries == b0 + 4);
		assert((fi->retry_known[0] & 0x3fffffff) == 0xfff && (fi->retry_known[0] >> 30) == 3 && (fi->retry_known[1] & 0xffff) == 0xffff);
		/* The per-frame quality: the newest header's frame, known. */
		assert(fi->q_frame && (fi->q_word[0] >> 30 & 1) && fi->q_frame <= fi->frames + 1);
	}
	inject_valid = saved_inject; inject_receive = saved_receive;

	/* SET 22: the receiver's bandwidth target (dir RX = 1, code <= 5). */
	uint8_t bw[3] = {0, 1, 4};
	assert(ar_rf_host_bandwidth(NULL, 3) == -22 && ar_rf_host_bandwidth(bw, 2) == -22);
	assert(!ar_rf_host_bandwidth(bw, 3));
	bw[1] = 0; assert(ar_rf_host_bandwidth(bw, 3) == -95);
	bw[1] = 1; bw[2] = 6; assert(ar_rf_host_bandwidth(bw, 3) == -95);

	/* SET 0x26: the retransmission mask, 8 bytes. */
	uint8_t retx[8] = {3};
	assert(ar_rf_host_retx_config(retx, 7) == -22 && !ar_rf_host_retx_config(retx, 8));

	/* SET 36: one MCS policy row by internal index; unknown index answers 6. */
	uint8_t row[16] = {0};
	assert(ar_rf_host_mcs_row(NULL) == -22);
	row[4] = 200; assert(ar_rf_host_mcs_row(row) == 6);
	row[4] = 1; row[6] = 0x34; row[7] = 0x12; assert(!ar_rf_host_mcs_row(row));

	/* Power: GET 7 closed-loop flags, SET/GET 8 a user's level, SET/GET 9
	 * automatic range; unknown requests -95. */
	uint8_t pin[3] = {0}, pout[8]; size_t plen = 9;
	assert(ar_rf_host_power(0x01000007, pin, 0, pout, &plen) == 0 && plen == 1);
	pin[0] = 0; assert(ar_rf_host_power(0x01000008, pin, 1, pout, &plen) == 0 && plen == 2 && pout[0] == 0);
	pin[0] = 200; assert(ar_rf_host_power(0x02000008, pin, 2, pout, &plen) == -22);
	assert(ar_rf_host_power(0x02000008, pin, 1, pout, &plen) == -22);
	uint8_t range[3] = {1, 20, 30};                     /* max below min */
	assert(ar_rf_host_power(0x02000009, range, 3, pout, &plen) == -22);
	range[0] = 2; assert(ar_rf_host_power(0x02000009, range, 3, pout, &plen) == -22);
	range[0] = 0; range[1] = 30; range[2] = 10;
	assert(!ar_rf_host_power(0x02000009, range, 3, pout, &plen));
	assert(!ar_rf_host_power(0x01000009, pin, 0, pout, &plen) && plen == 3 && !memcmp(pout, range, 3));
	assert(ar_rf_host_power(0x01000099, pin, 0, pout, &plen) == -95 && plen == 0);

	/* GET 6: MCS and stock's link-rate model. */
	uint8_t mcs_in[2] = {2, 0}, mcs_out[8];
	assert(ar_rf_host_mcs(NULL, 2, mcs_out) == -22 && ar_rf_host_mcs(mcs_in, 2, mcs_out) == -22);
	mcs_in[0] = 1; mcs_in[1] = 1; assert(ar_rf_host_mcs(mcs_in, 2, mcs_out) == -6);
	mcs_in[1] = 0; assert(!ar_rf_host_mcs(mcs_in, 2, mcs_out));
	assert(ar_rf_test_link_rate(12, 4, 5634) == 25934 || ar_rf_test_link_rate(12, 4, 5634) == 25933);
	uint32_t mcs11 = ar_rf_test_link_rate(13, 4, 4651);
	assert(mcs11 > 35100 && mcs11 < 35800);              /* stock: 35.52 Mbps */
	uint32_t at5 = ar_rf_test_link_rate(12, 2, 5634);   /* 5 MHz: stock about 6483 */
	assert(at5 >= 6400 && at5 <= 6500);
	assert(!ar_rf_test_link_rate(16, 4, 5634) && !ar_rf_test_link_rate(12, 1, 5634) && !ar_rf_test_link_rate(12, 4, 0));
	assert(ar_rf_test_link_rate(12, 5, 5634) == ar_rf_test_link_rate(12, 4, 5634));   /* 40 codes as 20, as stock */

	/* GET 107: link quality over the last frames (or -1 with none fresh). */
	uint8_t link_out[216];
	assert(ar_rf_host_link_info(NULL, 1, link_out) == -22);
	int link = ar_rf_host_link_info(mcs_in, 1, link_out);
	assert(link == 0 || link == -1);
	/* The AP applied 5 MHz above; SET 22 only requested 20 MHz afterward. */
	if (!link) { assert(link_out[9] == rx->channel_index && link_out[16] == 2); }
	ar_rf_set_event_callback(NULL, NULL);
}

int main(void)
{
	assert(ar_rf_host_ap_time() == UINT32_MAX);
	uint8_t register_output[256];
	assert(ar_rf_host_registers(0, 1, register_output) == -11);
	uint8_t host_status[300];
	assert(ar_rf_host_status(0x3ff, NULL) == -22);
	assert(ar_rf_host_status(0x3ff, host_status) == -11);
	*reg(0x0a104080) = 0x80000000;
	*reg(0x001400a4) = 4;
	*reg(0x0014009c) = 0x00010000;
	*reg(0x0a1060c8) = 0x80000000;
	assert(ar_rf_probe() == 0);
	assert(*reg(0x0a1060c8) == 0x80000003);
	assert(ar_rf_information()->phase == AR_RF_PROBED && ar_rf_information()->chip_profile == 1);
	for (unsigned i = 0; i < 64; i++) { *reg(0x0a110100 + i * 4) = 0x03020100 + i * 0x04040404u; }
	unsigned register_writes_before = writes;
	trace_register_reads = 1;
	assert(!ar_rf_host_registers(0x103, 7, register_output) && register_reads == 7);
	for (unsigned i = 0; i < 256; i++) { assert(register_output[i] == (i < 7 ? i + 3 : 0)); }
	for (unsigned i = 0; i < 7; i++) { assert(register_read_addresses[i] == 0x0a110000 + ((0x103 + i) & ~3u)); }
	register_reads = 0; volatile_register_reads = 1;
	assert(!ar_rf_host_registers(0x103, 7, register_output) && register_reads == 7);
	for (unsigned i = 0; i < 7; i++) { assert(register_output[i] == ((i + 3) ^ (i + 1))); }
	register_reads = 0; volatile_register_reads = 0;
	assert(!ar_rf_host_registers(0x100, 256, register_output) && register_reads == 256);
	for (unsigned i = 0; i < 256; i++) { assert(register_output[i] == i); }
	register_reads = 0; *reg(0x0a110ffc) = 0xa5000000;
	assert(!ar_rf_host_registers(4095, 1, register_output) && register_reads == 1 && register_output[0] == 0xa5);
	assert(register_read_addresses[0] == 0x0a110ffc);
	register_reads = 0;
	assert(!ar_rf_host_registers(4096, 0, register_output) && !register_reads);
	for (unsigned i = 0; i < 256; i++) { assert(!register_output[i]); }
	const unsigned bad_windows[][2] = {{4096, 1}, {4095, 2}, {0, 257}, {65535, 65535}, {UINT32_MAX, 1}, {1, UINT32_MAX}};
	memset(register_output, 0xa5, sizeof register_output);
	for (unsigned i = 0; i < sizeof bad_windows / sizeof bad_windows[0]; i++) {
		assert(ar_rf_host_registers(bad_windows[i][0], bad_windows[i][1], register_output) == -6);
		assert(!register_reads);
		for (unsigned j = 0; j < 256; j++) { assert(register_output[j] == 0xa5); }
	}
	assert(ar_rf_host_registers(0, 1, NULL) == -22 && !register_reads);
	assert(writes == register_writes_before); trace_register_reads = 0;
	assert(*reg(0x0a104080) == 0x80000002 && *reg(0x00140098) == 2);
	/* OTP identity takes byte14 and unaligned bytes26..29, not a stored MAC. */
	emulate_otp = 1; otp_words[3] = 0xa5260000; otp_words[6] = 0x1234abcd; otp_words[7] = 0xabcd5678;
	uint8_t hardware_mac[4];
	assert(ar_rf_board_identity(hardware_mac) == 0);
	uint32_t expected_mac = (UINT32_C(0x56781234) << 6) | 0x26;
	for (unsigned i=0;i<4;i++) { assert(hardware_mac[i] == (uint8_t)(expected_mac >> (8*i))); }
	memset(otp_words,0,sizeof otp_words); assert(ar_rf_board_identity(hardware_mac) == -2);
	emulate_otp = 0;

	uint8_t pins[40] = {0};
	for (unsigned i = 0; i < 10; i++) { pins[i * 4] = i / 5; pins[i * 4 + 1] = i % 5; pins[i * 4 + 2] = 3; }
	assert(ar_rf_setup_pins(pins, sizeof pins) == 0);
	uint64_t pll;
	assert(ar_rf_pll_word(5770000, &pll) == 0 && pll == UINT64_C(0x0120800000));
	assert(ar_rf_pll_word(5839000, &pll) == 0 && pll == UINT64_C(0x0123f33333));
	assert(ar_rf_pll_word(2100000, &pll) == 0 && pll == UINT64_C(0x92d2000000));
	assert(ar_rf_pll_word(0, &pll) == -22);
	uint8_t records[704 * 8] = {0};
	for (unsigned i = 0; i < 704; i++) {
		records[i * 8] = i * 4; records[i * 8 + 1] = (i * 4) >> 8;
		records[i * 8 + 4] = 0x5a;
	}
	unsigned before = writes;
	records[6] = 0xfe;
	records[8] = 1; /* Second record is unaligned: reject before writing the first. */
	assert(ar_rf_load_defaults(records, 16) == -22 && writes == before);
	records[8] = 4;
	assert(ar_rf_load_defaults(records, sizeof records) == 0);
	assert(ar_rf_information()->default_writes == 704 && *reg(0x0a110000) == 0xfe005a);
	assert(ar_rf_load_defaults(records, 8) == -22);
	*reg(0x0a1100e0) = 0xff00006f;
	*reg(0x0a110fa8) = 0x300;
	*reg(0x0a110360) = 6;
	*reg(0x0a1104f0) = 0x8483;
	*reg(0x0a1104f4) = 0x868500;
	assert(ar_rf_calibrate() == 0);
	assert(ar_rf_information()->phase == AR_RF_CALIBRATED && ar_rf_information()->calibration_rounds == 3);
	*reg(0x0a110fa0) = 0xf700;
	*reg(0x0a110fa4) = 0xaa991007;
	assert(ar_rf_setup_psram() == 0 && ar_rf_information()->phase == AR_RF_PSRAM);
	uint8_t readback[256];
	memset(readback, 0x5a, sizeof readback);
	assert(ar_rf_read_registers(0xcd0, 4, readback) == 0);
	assert(readback[0] == (*reg(0x0a110cd0) & 255) && readback[4] == 0x5a);
	assert(ar_rf_read_registers(0xcd1, 4, readback) == -22);
	assert(ar_rf_read_registers(0xffc, 8, readback) == -22);
	assert(ar_rf_read_registers(0, 260, readback) == -22);
	assert(ar_rf_read_registers(0, 0, readback) == -22);
	assert(ar_rf_read_registers(0, 4, NULL) == -22);
	assert(ar_rf_read_registers(UINT32_MAX, 4, readback) == -22);
	assert((*reg(0x0a110cd4) & 0xff0000) == 0x8b0000);
	*reg(0x0a110f88) = 1u << 8;
	*reg(0x0a110f90) = 32;
	*reg(0x0a110bb0) = 0x102080;
	*reg(0x0a110f00) = 0x1080;
	*reg(0x0a110bac) = 0xffffc0c0 | 0x1107;
	*reg(0x0a110bd4) = 0xa5000042 | 0x3ff00;
	before = writes;
	assert(ar_rf_monitor(5770000, 0, 0xcb, 0) == -22 && writes == before);
	assert(ar_rf_monitor(5770000, 2, 0xcb, 39) == -22 && writes == before);
	inject_receive = 1;
	inject_header = 1;
	assert(ar_rf_monitor(5770000, 2, 0xcb, 0) == 0);
	assert(ar_rf_receive_information()->before_monitor_header == 0x1080);
	assert(ar_rf_receive_information()->before_release_header == 0x1080);
	assert(ar_rf_receive_information()->fch_locked == 0); /* Baselines never count as events. */
	assert(ar_rf_information()->phase == AR_RF_RECEIVING);
	assert(ar_rf_read_registers(0, 4, readback) == 0); /* Readable while receiving (linked dumps). */
	assert((*reg(0x0a110800) & 0x80000) != 0 && (*reg(0x0a110a60) & 16) == 0);
	assert((*reg(0x0a1109e0) & 0xffe7) == 0xc4a5);
	/* Slot-0 RX starts at byte 0x93a; slot-1 starts at 0x940. A uniform
	 * five-byte stride or ignoring the field shift tunes the wrong words. */
	/* User 0's first bank is parked at 2100 MHz like stock (0x92 0xd2 0 0 0);
	 * the uplink bank (0x910) and the broadcast receive bank (0x905) tune. */
	assert((*reg(0x0a110938) & 0xffff0000) == 0xd2920000);
	assert((*reg(0x0a11093c) & 0x00ffffff) == 0);
	assert(*reg(0x0a110940) == 0x0000d292); /* User 1 parked at 2100 MHz. */
	assert(*reg(0x0a110bac) == 0x1107 && *reg(0x0a110bd4) == 0xa5000042);
	assert(*reg(0x0a110910) == 0x00802001);
	assert((*reg(0x0a110914) & 0xffffff00) == 0x00d29200); /* User 1 other bank parked. */
	assert((*reg(0x0a110b00) & 0xffffff) == 0x086021);
	assert((*reg(0x0a110b20) & 0x40ffffff) == 0x40496032);
	for (unsigned iteration = 0; iteration < 3000 && ar_rf_information()->phase == AR_RF_RECEIVING; iteration++) { ar_rf_poll(); }
	assert(ar_rf_receive_information()->fch_locked == 1 && ar_rf_receive_information()->fch_last == 0x1080);
	assert(*reg(0x0a110bb0) == 0x102080); /* FCH read restores the selector. */
	assert(ar_rf_receive_information()->receive_events == 1);
	assert(ar_rf_receive_information()->dma_completions == 1);
	assert(ar_rf_receive_information()->packets == 1 && ar_rf_receive_information()->bytes == 32);
	assert(ar_rf_receive_information()->first_bytes[0] == 0x58);
	assert(ar_rf_receive_information()->mac_invalid_packets == 1);
	assert(ar_rf_receive_information()->mac_valid_packets == 0);
	assert(ar_rf_receive_information()->dev_control_valid == 0);
	assert(offsetof(struct ar_rf_rx_info, mac_valid_packets) == 525 * 4);
	assert(offsetof(struct ar_rf_rx_info, first_packet_elapsed_us) == 590 * 4);
	assert(offsetof(struct ar_rf_rx_info, admission_attempts) == 593 * 4);
	assert(offsetof(struct ar_rf_rx_info, irq_last_elapsed_us) == 619 * 4);
	assert(sizeof(struct ar_rf_rx_info) == 1024 * 4);
	assert(offsetof(struct ar_rf_rx_info, lock_messages) == 662 * 4);
	assert(offsetof(struct ar_rf_rx_info, first_login_control) == 685 * 4);
	assert(offsetof(struct ar_rf_rx_info, last_login_sync) == 789 * 4);
	assert(ar_rf_information()->phase == AR_RF_PSRAM);
	assert(ar_rf_monitor(5770000, 2, 0xcb, 15) == 0);
	assert(*reg(0x0a110810) == 0x0080133a && *reg(0x0a110814) == 0);
	assert(*reg(0x0a110870) == 0xf00);
	for (unsigned iteration = 0; iteration < 3000 && ar_rf_information()->phase == AR_RF_RECEIVING; iteration++) { ar_rf_poll(); }
	*reg(0x0a110e6c) = 0; /* Prepare/transmit command loads cached compensation. */
	*reg(0x0a11048c) = 0xa5000033;
	inject_command = 1;
	assert(ar_rf_monitor(5770000, 2, 0xcb, 1) == 0);
	for (unsigned iteration = 0; iteration < 3000 && ar_rf_information()->phase == AR_RF_RECEIVING; iteration++) { ar_rf_poll(); }
	assert(ar_rf_receive_information()->command_modes[0] == 1);
	assert(*reg(0x0a11048c) == 0xa50000ff);
	assert((*reg(0x0a1104a0) & 0xffff) == 0x8483);
	assert((*reg(0x0a1104a4) >> 16) == 0x8685);
	assert(ar_rf_information()->phase == AR_RF_PSRAM);
	*reg(0x001400a4) = 0;
	*reg(0x0a110cf4) = 0x20200003;
	*reg(0x0a110bf0) = 0x61612121;
	assert(ar_rf_monitor(5770000, 2, 0xcb, 35) == 0);
	assert(*reg(0x0a110cf4) == 0x20000003);
	assert((*reg(0x0a110de0) & 255) == 0 && (*reg(0x0a110da0) & 255) == 0xcb);
	assert((*reg(0x0a110bf0) & 3) == 0 && (*reg(0x0a110bf0) & 0xfffffffc) == 0x41410100);
	assert(*reg(0x0a110810) == 0x183a113a && *reg(0x0a110870) == 0x0f00dd88);
	for (unsigned iteration = 0; iteration < 3000 && ar_rf_information()->phase == AR_RF_RECEIVING; iteration++) { ar_rf_poll(); }
	/* Known six-step frame: three 100 us prepares, 5331 us broadcast RX,
	 * 100 us special-user slot and 3696 us TX. Cross the 32-bit clock wrap. */
	*reg(0x0a110984) = 0x2210009c;
	*reg(0x0a110a00) = 0x168f;
	*reg(0x0a110df0) = 0xaabbcc55;
	ticks = UINT32_MAX - 200000;
	assert(ar_rf_monitor(5770000, 100, 0xcb, 36) == 0);
	assert((*reg(0x0a110df0) & 0x00ffffff) == 0x00bbcc00);
	assert((*reg(0x0a110de0) & 255) == 0 && (*reg(0x0a110da0) & 255) == 0xcb);
	assert(ar_rf_receive_information()->search_frame_us == 9427);
	assert(ar_rf_receive_information()->search_dwell_ms == 11);
	ticks += 10 * 40000;
	ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 0);
	ticks += 40000;
	ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 1);
	assert(ar_rf_receive_information()->search_dwell_ms == 20);
	ticks += 19 * 40000;
	ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 1);
	ticks += 40000;
	ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 2);
	assert(ar_rf_receive_information()->search_dwell_ms == 11);
	ticks += 50 * 40000; /* Late polling does one restart, not catch-up bursts. */
	ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 3);
	assert(ar_rf_receive_information()->packets == 0 && ar_rf_receive_information()->fch_locked == 0);
	ticks += 20 * 40000;
	ar_rf_poll();
	assert(ar_rf_information()->phase == AR_RF_PSRAM);
	assert(ar_rf_receive_information()->search_restarts == 3); /* Deadline wins. */
	assert(ar_rf_receive_information()->monitor_elapsed_us >= 100000);
	assert(ar_rf_monitor(5770000, 100, 0xcb, 36) == 0);
	inject_header = 1;
	ticks += 12 * 40000;
	ar_rf_poll();
	assert(ar_rf_receive_information()->search_suspended == 1);
	assert(ar_rf_receive_information()->search_restarts == 0);
	assert(ar_rf_receive_information()->packets == 0); /* Header is not a packet. */
	ticks += 100 * 40000;
	ar_rf_poll();
	/* DEV startup holds TX across initial release, search restarts and
	 * header detection. A header alone must not enable transmission. */
	*reg(0x0a110800) &= ~4u;
	assert(ar_rf_monitor(5770000, 100, 0xcb, 37) == 0);
	assert((*reg(0x0a110800) & 7) == 4);
	ticks += 12 * 40000;
	ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 1);
	assert((*reg(0x0a110800) & 7) == 4);
	inject_header = 1;
	ticks += 21 * 40000;
	ar_rf_poll();
	assert(ar_rf_receive_information()->search_suspended == 1);
	assert(ar_rf_receive_information()->search_restarts == 1);
	assert(ar_rf_receive_information()->packets == 0);
	assert((*reg(0x0a110800) & 7) == 4);
	ticks += 100 * 40000;
	ar_rf_poll();
	assert(ar_rf_information()->phase == AR_RF_PSRAM);
	assert((*reg(0x0a110800) & 7) == 5);
	assert(ar_rf_monitor(5770000, 100, 0xcb, 38) == 0);
	assert(*reg(0x0a110810) == 0x1c3a183a && *reg(0x0a110870) == 0x000feedd);
	assert((*reg(0x0a110988) & 0xffff) == 2500 && (*reg(0x0a11098c) >> 28) == 0);
	assert(*reg(0x0a1108a4) == 0 && (*reg(0x0a110800) & 7) == 4);
	static const uint32_t powers[] = {0x44332211, 0, 0xffffff7f, 0x01000000};
	for (unsigned sample = 0; sample < 4; sample++) {
		*reg(0x0a110e70) = powers[sample];
		*reg(0xe0801144) |= 1; /* IRQ5, CLIC vector 81. */
		ar_rf_poll();
	}
	assert(ar_rf_receive_information()->power_samples == 4);
	assert(ar_rf_receive_information()->power_last == 1);
	assert(ar_rf_receive_information()->power_min == 0);
	assert(ar_rf_receive_information()->power_max == 0x7fffffff);
	ticks += 100 * 40000;
	ar_rf_poll();
	assert(ar_rf_information()->phase == AR_RF_PSRAM);
	assert(ar_rf_receive_information()->search_restarts == 0);
	assert(ar_rf_receive_information()->packets == 0 && ar_rf_receive_information()->fch_locked == 0);
	uint8_t channels[12];
	/* Configured startup bandwidth must survive channel retune/restart;
	 * search, identity and TX hold retain the operation-39 contract. */
	put32(channels, 5770000); put32(channels + 4, 5839000);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 41, channels, 8) == 0);
	assert(((*reg(0x0a1109e0) >> 8) & 7) == 2);
	assert(((*reg(0x0a1109f8) >> 8) & 7) == 4); /* Power-search user unchanged. */
	assert((*reg(0x0a110800) & 7) == 4 && (*reg(0x0a110df0) & 255) == 0);
	assert(ar_rf_receive_information()->search_frame_us == 9427);
	ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->frequency_khz == 5839000);
	assert((*reg(0x0a110a90) & 0xffff) == 5839000 / 2000);
	assert(((*reg(0x0a1109e0) >> 8) & 7) == 2);
	assert(ar_rf_receive_information()->search_restarts == 1);
	ticks += 100 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->packets == 0 && ar_rf_receive_information()->fch_locked == 0);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 42, channels, 8) == 0);
	assert((*reg(0x0a1109e0) & 0xe707) == 0xa204);
	assert((*reg(0x0a110970) >> 16) == 4688);
	assert(ar_rf_receive_information()->search_frame_us == 7096);
	assert(ar_rf_receive_information()->search_dwell_ms == 9);
	ticks += 9 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->frequency_khz == 5839000);
	assert(ar_rf_receive_information()->search_dwell_ms == 9);
	ticks += 9 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->channel_wraps == 1);
	assert(ar_rf_receive_information()->search_dwell_ms == 16);
	assert((*reg(0x0a1109e0) & 0xe707) == 0xa204);
	inject_header = 1; ticks += 16 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_suspended == 1 && ar_rf_receive_information()->search_restarts == 2);
	assert(ar_rf_receive_information()->packets == 0 && (*reg(0x0a110800) & 7) == 4);
	ticks += 100 * 40000; ar_rf_poll();
	uint32_t old_profile = *reg(0x0a1109e0);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 43, channels, 8) == 0);
	assert(*reg(0x0a1109e0) == ((old_profile & 0xffffff) | 0x3c000000));
	/* Source 2 is only diagnostic evidence; every raw user/mode is bounded,
	 * first/last are distinct, selector state survives, and retries continue. */
	assert(offsetof(struct ar_rf_rx_info, source2_users) == 355 * 4);
	uint32_t old_selector = *reg(0x0a110c00);
	for (unsigned user = 0; user < 16; user++) {
		*reg(0x0a110e6c) = 0xa5001234 | user << 16 | (user & 3) << 20;
		*reg(0xe0801128) |= 1; ar_rf_poll();
		assert(ar_rf_receive_information()->source2_users[user] == 1);
		assert(*reg(0x0a110c00) == old_selector);
	}
	assert(ar_rf_receive_information()->irq_counts[2] == 16);
	for (unsigned mode = 0; mode < 4; mode++) {
		assert(ar_rf_receive_information()->source2_modes[mode] == 4);
	}
	assert(ar_rf_receive_information()->source2_command_first == 0xa5001234);
	assert(ar_rf_receive_information()->source2_command_last == 0xa53f1234);
	assert(ar_rf_receive_information()->fch_locked == 0 && ar_rf_receive_information()->receive_events == 0);
	assert(ar_rf_receive_information()->dma_completions == 0 && ar_rf_receive_information()->packets == 0);
	ticks += 9 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 1 && (*reg(0x0a1109e0) >> 24) == 0x3c);
	ticks += 100 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->packets == 0 && ar_rf_receive_information()->fch_locked == 0);
	/* Restore startup compensation fields after an operation that applied
	 * calibration. Preserve adjacent fields through the next command/retry. */
	*reg(0x0a110480) = 0x654321ff; *reg(0x0a1104a0) = 0x9876abcd;
	*reg(0x0a1104a4) = 0x1234fedc; *reg(0x0a11048c) = 0x876543ff;
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 44, channels, 8) == 0);
	assert(*reg(0x0a110480) == 0x654321fd && *reg(0x0a1104a0) == 0x98760000);
	assert(*reg(0x0a1104a4) == 0xfedc && *reg(0x0a11048c) == 0x87654333);
	assert((*reg(0x0a1109e0) >> 24) == 0x3c);
	assert(ar_rf_receive_information()->irq_counts[2] == 0 && ar_rf_receive_information()->source2_command_first == 0);
	*reg(0x0a110e6c) = 0x80000; inject_command = 1;
	ticks += 9 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 1);
	assert(*reg(0x0a110480) == 0x654321fd && *reg(0x0a1104a0) == 0x98760000);
	assert(*reg(0x0a1104a4) == 0xfedc && *reg(0x0a11048c) == 0x87654333);
	ticks += 100 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->packets == 0 && ar_rf_receive_information()->fch_locked == 0);
	*reg(0x0a110b4c) = 0x765432ff;
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 45, channels, 8) == 0);
	assert(*reg(0x0a110b4c) == 0x7654320f);
	assert((*reg(0x0a110480) & 2) == 0 && (*reg(0x0a1109e0) >> 24) == 0x3c);
	ticks += 9 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 1 && *reg(0x0a110b4c) == 0x7654320f);
	ticks += 100 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->packets == 0 && ar_rf_receive_information()->fch_locked == 0);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 46, channels, 8) == 0);
	assert((*reg(0x0a1109e0) & 0xff00e707) == 0x3c00c405);
	assert((*reg(0x0a110970) >> 16) == 8330 && ar_rf_receive_information()->search_frame_us == 9427);
	assert(ar_rf_receive_information()->search_dwell_ms == 11 && (*reg(0x0a110480) & 2) == 0);
	ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 1 && (*reg(0x0a1109e0) & 0xff00e707) == 0x3c00c405);
	ticks += 100 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->packets == 0 && ar_rf_receive_information()->fch_locked == 0);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 47, channels, 8) == 0);
	assert((*reg(0x0a110de0) & 255) == 0xcb && (*reg(0x0a110df0) & 255) == 0xcb);
	ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 1 && (*reg(0x0a110de0) & 255) == 0xcb);
	assert(offsetof(struct ar_rf_rx_info, header_users) == 377 * 4);
	assert(offsetof(struct ar_rf_rx_info, first_valid_control) == 429 * 4);
	assert(offsetof(struct ar_rf_rx_info, eop_snapshots) == 436 * 4);
	assert(offsetof(struct ar_rf_rx_info, before_monitor_header) == 480 * 4);
	assert(offsetof(struct ar_rf_rx_info, first_selector_old) == 489 * 4);
	static const uint16_t control_offsets[] = {0xf04, 0xf08, 0xf0c, 0xf10, 0xf34, 0xf38, 0xf3c};
	for (unsigned i = 0; i < 7; i++) { *reg(0x0a110000 + control_offsets[i]) = 0x12345670 + i; }
	check_header_bank = 1;
	uint32_t old_header_selector = *reg(0x0a110bb0);
	*reg(0x0a110e6c) = 0x180000;
	for (unsigned user = 0; user < 16; user++) {
		*reg(0x0a110f00) = user << 9 | 0x80;
		inject_header = 1; ar_rf_poll();
		for (unsigned i = 0; i < 7; i++) { *reg(0x0a110000 + control_offsets[i]) = 0xffffffff; }
		*reg(0x0a110f00) = user << 9;
		inject_header = 1; ar_rf_poll();
		*reg(0x0a110fbc) = user << 20 | 0x12345;
		*reg(0xe0801160) |= 1; ar_rf_poll();
		assert(ar_rf_receive_information()->header_users[user] == 2);
		assert(ar_rf_receive_information()->header_valid_users[user] == 1);
		assert(ar_rf_receive_information()->eop_users[user] == 1);
	}
	assert(*reg(0x0a110bb0) == old_header_selector);
	assert(header_bank_reads == 42 && header_bank0_reads == 7);
	for (unsigned i = 0; i < 7; i++) { assert(ar_rf_receive_information()->first_valid_control[i] == 0x12345670 + i); }
	assert(ar_rf_receive_information()->first_selector_old == old_header_selector);
	assert(ar_rf_receive_information()->first_selector_set == (old_header_selector | 0x80));
	assert(ar_rf_receive_information()->first_selector_delayed == (old_header_selector | 0x80));
	assert(ar_rf_receive_information()->first_bank_delay_status == 0);
	for (unsigned i = 0; i < 7; i++) {
		assert(ar_rf_receive_information()->first_control_bank0[i] == ((0x12345670 + i) ^ 0xffffffff));
		assert(ar_rf_receive_information()->first_control_delayed[i] == 0x12345670 + i);
	}
	for (unsigned i = 0; i < 4; i++) {
		const struct ar_rf_eop_snapshot *snapshot = &ar_rf_receive_information()->eop_snapshots[i];
		assert(snapshot->status == (i << 20 | 0x12345));
		assert(snapshot->header == i << 9 && snapshot->valid_indications == i + 1);
		assert(snapshot->elapsed_us >= 11000);
		for (unsigned j = 0; j < 7; j++) { assert(snapshot->control[j] == 0xffffffff); }
	}
	check_header_bank = 0;
	assert(ar_rf_receive_information()->first_valid_header == 0x80);
	assert(ar_rf_receive_information()->first_valid_command == 0x180000);
	assert(ar_rf_receive_information()->first_valid_elapsed_us >= 11000);
	assert(ar_rf_receive_information()->search_suspended == 1 && ar_rf_receive_information()->search_restarts == 1);
	assert(ar_rf_receive_information()->receive_events == 0 && ar_rf_receive_information()->packets == 0);
	ticks += 100 * 40000; ar_rf_poll();
	*reg(0x0a110f00) = 0x1080; /* Restore the shared synthetic header fixture. */
	uint32_t prior_rx8 = *reg(0x0a1109e0);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 48, channels, 8) == 0);
	assert(*reg(0x0a1109e0) == ((prior_rx8 & 0xffffff) | 0x20000000));
	assert((*reg(0x0a110de0) & 255) == 0 && (*reg(0x0a110df0) & 255) == 0);
	assert(ar_rf_receive_information()->search_dwell_ms == 11);
	ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 1 && (*reg(0x0a1109e0) >> 24) == 0x20);
	ticks += 100 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->packets == 0 && ar_rf_receive_information()->fch_locked == 0);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 49, channels, 8) == 0);
	assert(*reg(0x0a110a48) == 0 && (*reg(0x0a1109e0) >> 24) == 0x3c);
	assert((*reg(0x0a110de0) & 255) == 0 && (*reg(0x0a110df0) & 255) == 0);
	assert((*reg(0x0a110800) & 4) == 4 && ar_rf_receive_information()->search_dwell_ms == 11);
	ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 1 && *reg(0x0a110a48) == 0);
	*reg(0x0a110f00) = 0x101096;
	inject_header = 1; ar_rf_poll();
	assert(ar_rf_receive_information()->mcs_buffer_transitions == 0);
	ticks += 100 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->packets == 0 && ar_rf_receive_information()->fch_locked == 1);
	assert(offsetof(struct ar_rf_rx_info, mcs_buffer_transitions) == 507 * 4);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 50, channels, 8) == 0);
	assert(*reg(0x0a110a48) == 0 && (*reg(0x0a1109e0) >> 24) == 0x3c);
	*reg(0x0a110bf0) = 0xa5a5a5a4;
	*reg(0x0a110d08) = 0x5a5b5a5a;
	/* Invalid status, wrong user, and each nonmatching MCS-key component. */
	static const uint32_t rejected_headers[] = {
		0x101016, 0x100e96, 0x1010a6, 0x103096, 0x101097, 0x1010b6
	};
	for (unsigned i = 0; i < sizeof rejected_headers / sizeof rejected_headers[0]; i++) {
		*reg(0x0a110f00) = rejected_headers[i];
		inject_header = 1; ar_rf_poll();
		assert(ar_rf_receive_information()->mcs_buffer_transitions == 0);
		assert(*reg(0x0a110bf0) == 0xa5a5a5a4 && *reg(0x0a110d08) == 0x5a5b5a5a);
	}
	*reg(0x0a110f00) = 0x101096;
	inject_header = 1; ar_rf_poll();
	assert(ar_rf_receive_information()->mcs_buffer_transitions == 1);
	assert(ar_rf_receive_information()->mcs_buffer_header == 0x101096);
	assert(ar_rf_receive_information()->mcs_buffer_bf0_before == 0xa5a5a5a4);
	assert(ar_rf_receive_information()->mcs_buffer_bf0_after == 0xa5a5a5a5);
	assert(ar_rf_receive_information()->mcs_buffer_d08_before == 0x5a5b5a5a);
	assert(ar_rf_receive_information()->mcs_buffer_d08_after == 0x5a5a5a5a);
	assert(*reg(0x0a110bf0) == 0xa5a5a5a5 && *reg(0x0a110d08) == 0x5a5a5a5a);
	/* Repeat eligibility must not reapply or replace the recorded transition. */
	*reg(0x0a110bf0) = 0xa5a5a5a4;
	*reg(0x0a110d08) = 0x5a5b5a5a;
	inject_header = 1; ar_rf_poll();
	assert(ar_rf_receive_information()->mcs_buffer_transitions == 1);
	assert(*reg(0x0a110bf0) == 0xa5a5a5a4 && *reg(0x0a110d08) == 0x5a5b5a5a);
	assert(ar_rf_receive_information()->receive_events == 0 && ar_rf_receive_information()->packets == 0);
	ticks += 100 * 40000; ar_rf_poll();
	*reg(0x0a110f00) = 0x1080;
	assert(offsetof(struct ar_rf_rx_info, empty_control_headers) == 513 * 4);
	for (unsigned nonzero_word = 0; nonzero_word < 3; nonzero_word++) {
		assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 51, channels, 8) == 0);
		assert(*reg(0x0a110a48) == 0 && (*reg(0x0a1109e0) >> 24) == 0x3c);
		for (unsigned i = 0; i < 7; i++) { *reg(0x0a110000 + control_offsets[i]) = 0; }
		/* Bits outside the seven extracted bytes must not count as content. */
		*reg(0x0a110f04) = 0x0fffffff;
		*reg(0x0a110f0c) = 0xfff00000;
		*reg(0x0a110f00) = 0x101016;
		inject_header = 1; ar_rf_poll();
		assert(ar_rf_receive_information()->empty_control_headers == 0);
		*reg(0x0a110f00) = 0x101096;
		inject_header = 1; ar_rf_poll();
		assert(ar_rf_receive_information()->empty_control_headers == 1);
		assert(ar_rf_receive_information()->search_suspended == 0);
		ticks += 11 * 40000; ar_rf_poll();
		assert(ar_rf_receive_information()->search_restarts == 1);
		inject_header = 1; ar_rf_poll();
		assert(ar_rf_receive_information()->empty_control_headers == 2);
		ticks += 11 * 40000; ar_rf_poll();
		assert(ar_rf_receive_information()->search_restarts == 2);
		*reg(0x0a110000 + control_offsets[nonzero_word]) |=
			nonzero_word == 0 ? 0x10000000 : nonzero_word == 1 ? 0x80000000 : 0x80000;
		inject_header = 1; ar_rf_poll();
		assert(ar_rf_receive_information()->empty_control_headers == 2);
		assert(ar_rf_receive_information()->fch_locked == 3);
		assert(ar_rf_receive_information()->last_control_header == 0x101096);
		assert(ar_rf_receive_information()->last_control_words[nonzero_word] ==
		       *reg(0x0a110000 + control_offsets[nonzero_word]));
		assert(ar_rf_receive_information()->search_suspended == 1);
		ticks += 21 * 40000; ar_rf_poll();
		assert(ar_rf_receive_information()->search_restarts == 2);
		assert(ar_rf_receive_information()->mcs_buffer_transitions == 0);
		assert(ar_rf_receive_information()->receive_events == 0 && ar_rf_receive_information()->packets == 0);
		ticks += 100 * 40000; ar_rf_poll();
	}
	*reg(0x0a110f00) = 0x1080;
	put32(channels, 5770000); put32(channels + 4, 5839000); put32(channels + 8, 2100000);
	assert(offsetof(struct ar_rf_rx_info, bad_control_headers) == 522 * 4);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 52, channels, 8) == 0);
	for (unsigned i = 0; i < 7; i++) { *reg(0x0a110000 + control_offsets[i]) = 0; }
	*reg(0x0a110f00) = 0x101096;
	inject_header = 1; ar_rf_poll();
	assert(ar_rf_receive_information()->last_control_check == 0x1e);
	assert(ar_rf_receive_information()->bad_control_headers == 1);
	assert(ar_rf_receive_information()->search_suspended == 0);
	ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 1);
	/* Trial 88's nonzero but invalid seven-byte control contents. */
	*reg(0x0a110f04) = 0x371610;
	*reg(0x0a110f0c) = 0x4000;
	inject_header = 1; ar_rf_poll();
	assert(ar_rf_receive_information()->last_control_check == 0x0e);
	assert(ar_rf_receive_information()->bad_control_headers == 2);
	assert(ar_rf_receive_information()->search_suspended == 0);
	ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 2);
	/* Synthetic matching checksum suspends retries but creates no packet. */
	*reg(0x0a110f04) |= 0xe0000000;
	inject_header = 1; ar_rf_poll();
	assert(ar_rf_receive_information()->bad_control_headers == 2);
	assert(ar_rf_receive_information()->matching_control_headers == 1);
	assert(ar_rf_receive_information()->empty_control_headers == 1);
	assert(ar_rf_receive_information()->search_suspended == 1);
	ticks += 21 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_restarts == 2);
	assert(ar_rf_receive_information()->receive_events == 0 && ar_rf_receive_information()->packets == 0);
	ticks += 100 * 40000; ar_rf_poll();
	*reg(0x0a110f00) = 0x1080;
	before = writes;
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 54, channels, 8) == -22 && writes == before);
	assert(ar_rf_monitor_plan(5770000, 10001, 0xcb, 53, channels, 8) == -22 && writes == before);
	assert(ar_rf_monitor_plan(5770000, 1501, 0xcb, 52, channels, 8) == -22 && writes == before);
	assert(ar_rf_monitor_plan(5770000, 0, 0xcb, 53, channels, 8) == -22 && writes == before);
	/* Longer configured-profile search crosses timer wrap and must survive
	 * the historical 1.5-second boundary. Accepted stock cache bytes from
	 * trial 92 pass the same check without synthesizing a received packet. */
	ticks = UINT32_MAX - 40000000;
	assert(ar_rf_monitor_plan(5770000, 10000, 0xcb, 53, channels, 4) == 0);
	assert((*reg(0x0a1109e0) & 0xff00e707) == 0x3c00a204);
	assert((*reg(0x0a110970) >> 16) == 0x1250);
	assert(*reg(0x0a110a48) == 0x80808080);
	assert(ar_rf_receive_information()->search_frame_us == 7096);
	assert(ar_rf_receive_information()->search_dwell_ms == 9);
	for (unsigned i = 0; i < 7; i++) { *reg(0x0a110000 + control_offsets[i]) = 0; }
	*reg(0x0a110f00) = 0x101096;
	inject_header = 1; ar_rf_poll();
	assert(ar_rf_receive_information()->bad_control_headers == 1);
	ticks += 1500 * 40000; ar_rf_poll();
	assert(ar_rf_information()->phase == AR_RF_RECEIVING);
	assert(ar_rf_receive_information()->search_restarts == 1);
	assert(ar_rf_receive_information()->search_suspended == 0);
	*reg(0x0a110f0c) = 0xc000;
	inject_header = 1; ar_rf_poll();
	assert(ar_rf_receive_information()->last_control_check == 0);
	assert(ar_rf_receive_information()->matching_control_headers == 1);
	assert(ar_rf_receive_information()->search_suspended == 1);
	assert(ar_rf_receive_information()->packets == 0);
	ticks += 8500 * 40000; ar_rf_poll();
	assert(ar_rf_information()->phase == AR_RF_PSRAM);
	assert(ar_rf_receive_information()->monitor_elapsed_us >= 10000000);
	assert(ar_rf_receive_information()->monitor_elapsed_us < 10000100);
	*reg(0x0a110f00) = 0x1080;
	before = writes;
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 39, channels, sizeof channels) == -22);
	assert(writes == before); /* Validate the last entry before programming any PLL. */
	put32(channels + 8, 5770000);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 39, channels, sizeof channels) == -22 && writes == before);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 39, NULL, 4) == -22 && writes == before);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 39, channels, 11) == -22 && writes == before);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 39, channels, 172) == -22 && writes == before);
	assert(ar_rf_monitor_plan(5839000, 100, 0xcb, 39, channels, 8) == -22 && writes == before);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 38, channels, 8) == -22 && writes == before);
	put32(channels + 8, 5740000);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 39, channels, sizeof channels) == 0);
	memset(channels, 0, sizeof channels); /* Request buffer may be reused immediately. */
	assert(ar_rf_receive_information()->channel_count == 3 && ar_rf_receive_information()->channels[0].visits == 1);
	assert(ar_rf_receive_information()->channels[1].frequency_khz == 5839000);
	ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->channel_index == 1 && ar_rf_receive_information()->frequency_khz == 5839000);
	assert((*reg(0x0a110904) & 0xffffff00) == 0xf3230100 && (*reg(0x0a110908) & 0xffff) == 0x3333);
	assert((*reg(0x0a110a90) & 0xffff) == 5839000 / 2000);
	assert(ar_rf_receive_information()->search_dwell_ms == 11 && ar_rf_receive_information()->channel_wraps == 0);
	ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->channel_index == 2 && ar_rf_receive_information()->channels[2].visits == 1);
	ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->channel_index == 0 && ar_rf_receive_information()->channel_wraps == 1);
	assert(ar_rf_receive_information()->search_dwell_ms == 20 && ar_rf_receive_information()->channels[0].visits == 2);
	inject_header = 1; ticks += 21 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->search_suspended == 1 && ar_rf_receive_information()->channel_index == 0);
	assert(ar_rf_receive_information()->search_restarts == 3 && ar_rf_receive_information()->packets == 0);
	ticks += 100 * 40000; ar_rf_poll();
	put32(channels, 5770000); put32(channels + 4, 5839000);
	assert(ar_rf_monitor_plan(5770000, 100, 0xcb, 40, channels, 8) == 0);
	*reg(0x0a110e70) = 0x01000000; *reg(0xe0801144) |= 1; ar_rf_poll();
	ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_receive_information()->channel_index == 1);
	assert(ar_rf_receive_information()->channels[0].power_samples == 1 && ar_rf_receive_information()->channels[0].power_min == 1);
	*reg(0x0a110e70) = 0x03000000; *reg(0xe0801144) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->channels[1].power_samples == 1 && ar_rf_receive_information()->channels[1].power_max == 3);
	ticks += 30 * 40000; ar_rf_poll(); /* Delayed polling advances once. */
	assert(ar_rf_receive_information()->channel_index == 0 && ar_rf_receive_information()->channel_wraps == 1);
	assert(ar_rf_receive_information()->search_restarts == 2 && ar_rf_receive_information()->search_dwell_ms == 10);
	uint32_t old_pll = *reg(0x0a11090c);
	*reg(0x0a110fa8) = 0; ticks += 11 * 40000; ar_rf_poll();
	assert(ar_rf_information()->phase == AR_RF_PSRAM && ar_rf_receive_information()->search_error == (uint32_t)-110);
	assert(ar_rf_receive_information()->channel_index == 0 && ar_rf_receive_information()->search_restarts == 2);
	assert(*reg(0x0a11090c) == old_pll && (*reg(0x0a110800) & 9) == 9); /* Failed hold never retunes or releases. */
	*reg(0x0a110fa8) = 0x300;
	uint8_t maximum_channels[AR_RF_MAX_CHANNELS * 4];
	for (unsigned i = 0; i < AR_RF_MAX_CHANNELS; i++) { put32(maximum_channels + 4 * i, 4900000 + i * 30000); }
	assert(ar_rf_monitor_plan(4900000, 1500, 0xcb, 39, maximum_channels, sizeof maximum_channels) == 0);
	for (unsigned i = 1; i < AR_RF_MAX_CHANNELS; i++) { ticks += 11 * 40000; ar_rf_poll(); }
	assert(ar_rf_receive_information()->channel_index == 41 && ar_rf_receive_information()->channels[41].visits == 1);
	ticks += 1500 * 40000; ar_rf_poll();
	assert(ar_rf_information()->phase == AR_RF_PSRAM && ar_rf_receive_information()->search_restarts == 41);
	assert(ar_rf_monitor(5770000, 100, 0xcb, 36) == 0);
	*reg(0x0a110fa8) = 0; /* Failed periodic reset must stop within its deadline. */
	ticks += 12 * 40000;
	ar_rf_poll();
	assert(ar_rf_information()->phase == AR_RF_PSRAM);
	assert(ar_rf_receive_information()->search_error == (uint32_t)-110);
	assert(ar_rf_information()->error == (uint32_t)-110);
	assert(ar_rf_receive_information()->search_restarts == 0);
	assert((*reg(0x0a110800) & 1) != 0);
	/* Operation 54 consumes only validated, configured mode-0 controls. */
	*reg(0x0a110fa8) = 0x300;
	uint8_t dev_plan[28] = {'D','E','V','0',1,2,3,4, 0,0,0,0, 5,6,7,8};
	dev_plan[20] = 1; dev_plan[22] = 2; dev_plan[23] = 2;
	put32(dev_plan + 24, 5770000);
	uint8_t control[37] = {37,1,0,0x40,1,0,0,0,5,6,7,8,1};
	control[28] = 4; control[32] = 1; memcpy(control + 33, dev_plan + 4, 4);
	uint8_t checksum = 0;
	for (unsigned i = 0; i < sizeof control; i++) { checksum = ar_mac_crc8_step(checksum ^ control[i]); }
	control[2] = checksum;
	size_t packet_size;
	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 0, 0, control, sizeof control, NULL, 0, &packet_size) == 0);
	assert(packet_size == 48);
	unsigned before_writes = writes;
	assert(ar_rf_monitor_plan(5770000, 100, 5, 54, dev_plan, sizeof dev_plan) == -22);
	assert(writes == before_writes); /* Wrong local identity cannot reach MMIO. */
	assert(ar_rf_monitor_plan(5770000, 100, 4, 54, dev_plan, sizeof dev_plan) == 0);
	assert(ar_rf_receive_information()->sync_control_words[0] == 0x1e);
	assert(*reg(0x0a110b8c) == 0x1e && *reg(0x0a110b90) == 0);
	*reg(0x0a110f88) = 1; *reg(0x0a110f90) = 48;
	inject_valid = inject_receive = 1; injected = 0;
	ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->admission_attempts == 1);
	assert(ar_rf_receive_information()->acquisition_ready == 1);
	assert(ar_rf_receive_information()->transition_status == 1);
	assert(ar_rf_receive_information()->assigned_slot == 8);
	assert((*reg(0x0a110800) & 4) == 0 && (*reg(0x0a110bf0) & 3) == 1);
	assert(((*reg(0x0a110b7c) >> 16) & 255) == 12);
	assert((*reg(0x0a110a48)) == 0);
	/* First TX command loads the cached band compensation. Repeated commands
	 * at this fixed frequency must not reapply the first-band handshake. */
	*reg(0x0a110e6c) = 0; *reg(0xe080114c) |= 1; ar_rf_poll();
	assert((*reg(0x0a1104a0) & 0xff0000) == 0xff0000);
	*reg(0x0a1104a0) &= ~0xff0000u;
	*reg(0xe080114c) |= 1; ar_rf_poll();
	assert((*reg(0x0a1104a0) & 0xff0000) == 0);
	*reg(0x0a110f70) = 0; *reg(0x0a110f5c) = 64 << 16; *reg(0x0a110f58) = 0x30;
	*reg(0xe080117c) |= 1; ar_rf_poll(); /* Source 19. */
	assert(ar_rf_receive_information()->tx_packets == 1 && ar_rf_receive_information()->tx_owned == 1);
	assert(ar_rf_receive_information()->last_tx_request == 3);
	struct ar_mac_frame encoded;
	assert(ar_mac_decode(ar_rf_receive_information()->last_tx_packet, 24, &encoded) == 0);
	assert(encoded.outband_length == 16 && encoded.outband[4] == 0x21);
	assert(!memcmp(encoded.outband + 8, dev_plan + 4, 4));
	assert((*reg(0x0a110d40) & 0xff00ffff) == 16);
	*reg(0xe080115c) |= 1; ar_rf_poll(); /* Source 11. */
	assert(ar_rf_receive_information()->tx_end == 1 && ar_rf_receive_information()->tx_owned == 0);
	ticks += 100 * 40000; ar_rf_poll(); assert((*reg(0x0a110800) & 4) != 0);
	/* Unknown peer cannot release TX; a power handshake failure also leaves
	 * it held even when packet admission succeeded. */
	dev_plan[20] = 0;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 54, dev_plan, sizeof dev_plan) == 0);
	injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->admission_rejects == 1);
	assert(!ar_rf_receive_information()->acquisition_ready && (*reg(0x0a110800) & 4));
	ticks += 100 * 40000; ar_rf_poll(); dev_plan[20] = 1;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 54, dev_plan, sizeof dev_plan) == 0);
	*reg(0x0a110fa8) = 0x100; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->acquisition_ready == 1);
	assert(ar_rf_receive_information()->transition_status == (uint32_t)-110);
	assert(!ar_rf_receive_information()->tx_packets && (*reg(0x0a110800) & 4));
	ticks += 100 * 40000; ar_rf_poll(); inject_valid = 0;
	/* Operation 54 observes source 18 without changing its user field. */
	*reg(0x0a110fa8) = 0x300; inject_receive = 0;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 54, dev_plan, sizeof dev_plan) == 0);
	*reg(0x0a110b4c) = 0xa55a00ad; *reg(0x0a110f68) = 0x80;
	*reg(0xe0801178) |= 1; ar_rf_poll();
	assert(*reg(0x0a110b4c) == 0xa55a00ad);
	assert(ar_rf_receive_information()->interleaver_users[8] == 1);
	assert(ar_rf_receive_information()->interleaver_updates == 0);
	ticks += 100 * 40000; ar_rf_poll();
	assert(ar_rf_monitor_plan(5770000, 100, 4, 55, dev_plan, sizeof dev_plan) == 0);
	assert(ar_rf_receive_information()->search_frame_us == 7096);
	for (unsigned user = 0; user < 16; user++) {
		*reg(0x0a110b4c) = 0xa55a00ad;
		*reg(0x0a110f68) = 0xabc00001 | user << 4;
		*reg(0x0a110f64) = 0x43210000 | user;
		*reg(0xe0801178) |= 1; ar_rf_poll();
		assert(*reg(0x0a110b4c) == (0xa55a0005 | ((user * 2 + 1) << 3)));
		assert(ar_rf_receive_information()->interleaver_users[user] == 1);
	}
	assert(ar_rf_receive_information()->interleaver_updates == 16);
	assert(ar_rf_receive_information()->interleaver_first_f68 == 0xabc00001);
	assert(ar_rf_receive_information()->interleaver_last_f68 == 0xabc000f1);
	assert(ar_rf_receive_information()->interleaver_last_f64 == 0x4321000f);
	assert(ar_rf_receive_information()->irq_last_elapsed_us[18] > 0);
	assert(!ar_rf_receive_information()->acquisition_ready && (*reg(0x0a110800) & 4));
	inject_valid = inject_receive = 1; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->transition_status == 1);
	*reg(0xe080117c) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->tx_packets == 1);
	*reg(0xe080115c) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->tx_owned == 0);
	ticks += 100 * 40000; ar_rf_poll(); inject_valid = 0;
	assert((*reg(0x0a110800) & 4) != 0);
	/* Stock acquisition uses TX0 2234 us / internal MCS1. Preserve TX1
	 * duration, keep TX held until valid admission, and derive dwell from
	 * the corrected whole frame. This does not simulate over-air acceptance. */
	*reg(0x0a110a00) = 0xa5a5168f; inject_receive = 0;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 56, dev_plan, sizeof dev_plan) == 0);
	assert(*reg(0x0a110a00) == 0xa5a50da3);
	assert((*reg(0x0a110b00) & 0x3f700) == 0x13000);
	assert(ar_rf_receive_information()->search_frame_us == 5634);
	assert(ar_rf_receive_information()->search_dwell_ms == 7);
	assert((*reg(0x0a110800) & 4) && !ar_rf_receive_information()->tx_packets);
	inject_valid = inject_receive = 1; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->transition_status == 1);
	assert((*reg(0x0a110b00) & 0x3f700) == 0x13000);
	*reg(0xe080117c) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->tx_packets == 1);
	*reg(0xe080115c) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->tx_owned == 0);
	ticks += 100 * 40000; ar_rf_poll(); inject_valid = 0;
	assert((*reg(0x0a110800) & 4) != 0);
	/* First-login evidence survives later ordinary control messages. The
	 * observer must not change report state, assignment or connected MMIO. */
	assert(ar_rf_monitor_plan(5770000, 100, 4, 57, dev_plan, sizeof dev_plan) == 0);
	inject_valid = 1; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->lock_messages == 1);
	assert(ar_rf_receive_information()->lock_results[0] == 1);
	uint8_t login_control[43] = {43,1,0,0x40,1,0,0,0,5,6,7,8,5,9};
	login_control[28] = 1; login_control[32] = 1;
	login_control[33] = 9; login_control[34] = 7;
	memcpy(login_control + 35, dev_plan + 4, 4);
	memcpy(login_control + 39, "\1\2\3\4", 4);
	checksum = 0;
	for (unsigned i = 0; i < sizeof login_control; i++) { checksum = ar_mac_crc8_step(checksum ^ login_control[i]); }
	login_control[2] = checksum;
	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 1, 0, login_control, sizeof login_control, NULL, 0, &packet_size) == 0);
	*reg(0x0a110f90) = packet_size;
	injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->login_extension_packets == 1);
	assert(ar_rf_receive_information()->login_matching_packets == 1);
	assert(ar_rf_receive_information()->lock_results[2] == 1);
	assert(ar_rf_receive_information()->last_login_slot == 0);
	assert(ar_rf_receive_information()->last_login_token == 9);
	assert(ar_rf_receive_information()->last_login_value == 7);
	assert(ar_rf_receive_information()->last_login_ap_state == 2);
	assert(ar_rf_receive_information()->last_login_sync == 0x04030201);
	assert(ar_rf_receive_information()->first_login_length == sizeof login_control);
	assert(ar_rf_receive_information()->first_matching_length == sizeof login_control);
	assert(!memcmp(ar_rf_receive_information()->first_matching_control, login_control, sizeof login_control));
	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 2, 0, control, sizeof control, NULL, 0, &packet_size) == 0);
	*reg(0x0a110f90) = packet_size;
	injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->lock_messages == 3);
	assert(ar_rf_receive_information()->lock_results[0] == 2);
	assert(ar_rf_receive_information()->login_matching_packets == 1);
	assert(!memcmp(ar_rf_receive_information()->first_login_control, login_control, sizeof login_control));
	assert(ar_rf_receive_information()->lock_observed_extensions == 5);
	assert(ar_rf_receive_information()->assigned_slot == 8 && !(*reg(0x0a110cf4) & (1u << 21)));
	*reg(0xe080117c) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->last_tx_packet[12] == 0x21);
	ticks += 100 * 40000; ar_rf_poll(); inject_valid = 0;
	assert((*reg(0x0a110800) & 4) != 0);
	/* DEV1 supplies effective TX gains, independently of report status bits.
	 * Reject truncation/analog overflow before any peripheral write. */
	uint8_t gain_plan[32]; memcpy(gain_plan, dev_plan, 24); gain_plan[3] = '1';
	gain_plan[24] = 17; gain_plan[25] = 65; gain_plan[26] = 29; gain_plan[27] = 83;
	put32(gain_plan + 28, 5770000);
	before_writes = writes;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 58, dev_plan, sizeof dev_plan) == -22);
	assert(ar_rf_monitor_plan(5770000, 100, 4, 58, gain_plan, 28) == -22);
	gain_plan[24] = 64;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 58, gain_plan, sizeof gain_plan) == -22);
	gain_plan[24] = 17; gain_plan[26] = 64;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 58, gain_plan, sizeof gain_plan) == -22);
	gain_plan[26] = 29;
	assert(writes == before_writes);
	uint32_t gain_a_other = *reg(0x0a1109a4) & ~0x3fffu;
	uint32_t gain_b_other = *reg(0x0a110da4) & ~63u;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 58, gain_plan, sizeof gain_plan) == 0);
	assert(*reg(0x0a1109a4) == (gain_a_other | 17u << 8 | 65));
	assert(*reg(0x0a110da4) == (gain_b_other | 29));
	assert((*reg(0x0a110da0) >> 24) == 83);
	assert(ar_rf_receive_information()->search_frame_us == 5634);
	assert((*reg(0x0a110800) & 4) != 0);
	/* The caller's calibration buffer can be reused immediately. */
	memset(gain_plan + 24, 0, 4);
	inject_valid = 1; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->transition_status == 1);
	assert((*reg(0x0a1109a4) & 0x3fff) == (17u << 8 | 65));
	*reg(0xe080117c) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->last_tx_packet[14] == 0); /* Existing 2/2 raw statuses. */
	ticks += 100 * 40000; ar_rf_poll(); inject_valid = 0;
	/* Apply only supported mode-0 remote sync, retain ACK on a small budget,
	 * emit it once, and stop on a stale assignment. */
	uint8_t connected_plan[36]; memcpy(connected_plan, gain_plan, 28);
	connected_plan[3] = '2'; connected_plan[28] = 25;
	memset(connected_plan + 29, 0, 3); put32(connected_plan + 32, 5770000);
	before_writes = writes; connected_plan[28] = 64;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 59, connected_plan, sizeof connected_plan) == -22);
	connected_plan[28] = 25; connected_plan[31] = 1;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 59, connected_plan, sizeof connected_plan) == -22);
	connected_plan[31] = 0; assert(writes == before_writes);
	connected_plan[3] = '3'; connected_plan[29] = 128;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 61, connected_plan, sizeof connected_plan) == -22);
	assert(writes == before_writes); connected_plan[29] = 6;
	uint8_t mcs_plan[AR_RF_DEV4_PREFIX + 4] = {0};
	memcpy(mcs_plan, connected_plan, 32); mcs_plan[3] = '4';
	mcs_plan[32] = 1; mcs_plan[33] = 5; mcs_plan[34] = 2;
	mcs_plan[40] = mcs_plan[41] = 255; mcs_plan[42] = 1; mcs_plan[43] = 2;
	mcs_plan[48] = 1; mcs_plan[60] = 5;
	/* Synthetic, unique FCH keys; production plans take the release-bound
	 * original table from private input. Key1 and key65536 exercise MCS1/5. */
	for (unsigned i = 0; i < 25; i++) { put32(mcs_plan + 168 + i * 4, (i / 9) << 16 | ((i / 3) % 3) << 8 | i % 3); }
	put32(mcs_plan + 168 + 5 * 4, 65536); put32(mcs_plan + 168 + 9 * 4, 258);
	/* MCS2's normal modulation/rate key for the TX command test. */
	put32(mcs_plan + 168, 2); put32(mcs_plan + 168 + 2 * 4, 0);
	put32(mcs_plan + AR_RF_DEV4_PREFIX, 5770000);
	assert(ar_rf_monitor_plan(5770000, 100, 4, 67, connected_plan, sizeof connected_plan) == -22);
	mcs_plan[43] = 4;
	assert(ar_rf_monitor_plan(5770000, 100, 4, 67, mcs_plan, sizeof mcs_plan) == -22);
	mcs_plan[43] = 2; mcs_plan[44] = 101;   /* ranging offset above 100 */
	assert(ar_rf_monitor_plan(5770000, 100, 4, 67, mcs_plan, sizeof mcs_plan) == -22);
	mcs_plan[44] = 0;
	assert(writes == before_writes);
	assert(sizeof(struct ar_rf_mcs_info) == 264);
	for (unsigned operation = 65; operation <= 77; operation++) {
	event_count = 0; memset(event_data, 0, sizeof event_data);
	ar_rf_set_event_callback(receive_event, event_data);
	/* Independent hardware stimulus; the preceding case can leave a valid
	 * command in its synthetic decoder registers after the monitor stops. */
	for (unsigned i = 0; i < 3; i++) { *reg(0x0a110f04 + i * 4) = 0; }
	forwarded_count = 0; transmit_calls = 0; transmit_commits = 0;
	transmit_remaining = sizeof transmit_source;
	for (unsigned i = 0; i < sizeof transmit_source; i++) { transmit_source[i] = i ^ 0x5a; }
	ar_rf_set_tx_callbacks(transmit_peek, transmit_commit, transmit_source);
	login_control[39] = 1; login_control[2] = 0; checksum = 0;
	for (unsigned i = 0; i < sizeof login_control; i++) { checksum = ar_mac_crc8_step(checksum ^ login_control[i]); }
	login_control[2] = checksum;
	ar_rf_set_payload_callback(receive_payload, forwarded);
	if (operation >= 69) { ticks = UINT32_MAX - 500000; }
	uint64_t session_started = 0;
	uint8_t candidate_request[402] = {0, 2};
	memcpy(candidate_request + 2, mcs_plan + 12, 8);
	if (operation >= 76) {
		if (operation == 77) {
			mcs_plan[40] = 1; mcs_plan[41] = 0; /* Bounded coding-request timeout. */
			assert(ar_rf_prepare_host(5770000, 0, 4, mcs_plan, sizeof mcs_plan) == -22);
			assert(ar_rf_monitor_plan(5770000, 0, 4, 77, mcs_plan, sizeof mcs_plan) == -22);
			assert(ar_rf_prepare_host_continuous(5770000, 4, mcs_plan, sizeof mcs_plan) == 0);
		} else { assert(ar_rf_prepare_host(5770000, 100, 4, mcs_plan, sizeof mcs_plan) == 0); }
		assert(ar_rf_information()->phase == AR_RF_PSRAM && !event_count);
		assert((*reg(0x0a110800) & 5) == 5);
		unsigned prepare_writes = writes;
		for (unsigned i = 0; i < 50; i++) { ar_rf_poll(); }
		assert(writes == prepare_writes && !event_count);
		assert(ar_rf_host_status(0x3ff, host_status) == 0 && !host_status[252] && host_status[253] == 25);
		assert(ar_rf_host_candidates(NULL, 402) == -22);
		assert(ar_rf_host_candidates(candidate_request, 401) == -22);
		candidate_request[0] = 1; assert(ar_rf_host_candidates(candidate_request, 402) == -22);
		candidate_request[0] = 0; candidate_request[1] = 101;
		assert(ar_rf_host_candidates(candidate_request, 402) == -22 && writes == prepare_writes);
		candidate_request[1] = 2;
		assert(ar_rf_host_candidates(candidate_request, 402) == 0);
		assert(ar_rf_information()->phase == AR_RF_RECEIVING);
		session_started = ticks;
	} else {
		assert(ar_rf_monitor_plan(5770000, 100, 4, operation,
			operation >= 67 ? mcs_plan : connected_plan,
			operation >= 67 ? sizeof mcs_plan : sizeof connected_plan) == 0);
	}
	if (operation >= 67) {
		unsigned status_writes = writes;
		memset(host_status, 0xa5, sizeof host_status);
		assert(ar_rf_host_status(0x3ff, host_status) == 0 && writes == status_writes);
		assert(host_status[0] == 1 && !host_status[1] && !host_status[2] && host_status[3] == 1);
		assert(host_status[4] == 1 && host_status[5] == 1 && !memcmp(host_status + 6, mcs_plan + 4, 4));
		assert(!host_status[252] && host_status[253] == 25 && !memcmp(host_status + 254, mcs_plan + 8, 4));
		for (unsigned slot = 1; slot < 8; slot++) { assert(host_status[252 + slot * 6] == 3 && host_status[253 + slot * 6] == 25); }
		const uint8_t freq[4] = {0x10,0x0b,0x58,0};
		for (unsigned user = 0; user < 10; user++) {
			uint8_t parked[4]; put32(parked, 2100000);
			assert(!memcmp(host_status + 20 + 24*user, user == 0 ? freq : parked, 4));
			assert(!memcmp(host_status + 32 + 24*user, user == 8 ? freq : parked, 4));
			assert(host_status[24 + 24*user] == 25);
		}
		assert(ar_rf_host_status(0x100, host_status) == 0);
		for (unsigned i = 12; i < 252; i++) { if (i < 204 || i >= 228) { assert(host_status[i] == 0); } }
		uint32_t saved = *reg(0x0a1109e0); *reg(0x0a1109e0) = (saved & ~7u) | 7;
		assert(ar_rf_host_status(0x100, host_status) == -95); *reg(0x0a1109e0) = saved;
	}
	*reg(0x0a110f00) = 0x1096; *reg(0xe0801168) |= 1; ar_rf_poll();
	assert(!ar_rf_receive_information()->mcs_buffer_transitions);
	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 0, 0, control, sizeof control, NULL, 0, &packet_size) == 0);
	*reg(0x0a110f90) = packet_size; inject_valid = 1; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->dev_state == 1);
	if (operation >= 75) {
		assert(event_count == 2 && !event_ids[0] && !event_ids[1]);
		assert(!memcmp(event_data[0], (uint8_t[]){0,0,3}, 3));
		assert(!memcmp(event_data[1], (uint8_t[]){0,1,0}, 3));
	}
	*reg(0x0a110f5c) = 160 << 16; *reg(0xe080117c) |= 1; ar_rf_poll();
	assert(!transmit_calls && !transmit_commits);

	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 1, 0, login_control, sizeof login_control, NULL, 0, &packet_size) == 0);
	*reg(0x0a110f90) = packet_size; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->sync_unsupported == 1);
	assert(ar_rf_receive_information()->dev_state == 1 && ar_rf_receive_information()->assigned_slot == 8);
	memset(login_control + 39, 0, 4); login_control[2] = 0; checksum = 0;
	for (unsigned i = 0; i < sizeof login_control; i++) { checksum = ar_mac_crc8_step(checksum ^ login_control[i]); }
	login_control[2] = checksum;
	uint8_t port0[20] = {0x4a,2,16,0x58};
	struct ar_mac_payload part = {.data = port0, .length = sizeof port0, .port = 0, .flags = 15};
	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 2, 0, login_control, sizeof login_control, &part, 1, &packet_size) == 0);
	*reg(0x0a110f90) = packet_size; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->dev_state == 2 && ar_rf_receive_information()->assigned_slot == 0);
	if (operation >= 67) { assert(ar_rf_host_status(0, host_status) == 0 && host_status[252] == 2); }
	if (operation >= 75) {
		assert(event_count == 3 && !event_ids[2]);
		assert(!memcmp(event_data[2], (uint8_t[]){0,2,1}, 3));
	}
	if (operation == 76) {
		unsigned update_writes = writes;
		assert(ar_rf_host_candidates(candidate_request, 402) == 0 && writes == update_writes);
		candidate_request[1] = 0;
		assert(ar_rf_host_candidates(candidate_request, 402) == -95 && writes == update_writes);
		/* Rejected list replacement preserves admission and the live link. */
		assert(ar_rf_receive_information()->dev_state == 2 && event_count == 3);
	}
	assert(ar_rf_receive_information()->dev_token == 9 && ar_rf_receive_information()->sync_ack_pending == 1);
	uint32_t bf0_before = *reg(0x0a110bf0), d08_before = *reg(0x0a110d08);
	*reg(0x0a110f00) = 0x1094; *reg(0xe0801168) |= 1; ar_rf_poll();
	assert(!ar_rf_receive_information()->mcs_buffer_transitions);
	*reg(0x0a110f00) = 0x1096; *reg(0xe0801168) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->mcs_buffer_transitions == (operation == 65));
	assert(*reg(0x0a110bf0) == ((bf0_before & ~3u) | 1));
	assert(*reg(0x0a110d08) == (d08_before & ~(1u << 16)));
	*reg(0xe0801168) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->mcs_buffer_transitions == (operation == 65));
	assert((*reg(0x0a110cf4) & (1u << 21)) && (*reg(0x0a110b4c) & 0xf8) == 8);
	assert(ar_rf_receive_information()->port0_packets == 1 && ar_rf_receive_information()->last_port0_length == 20);
	assert(!memcmp(ar_rf_receive_information()->first_port0, port0, 20));
	*reg(0x0a110f5c) = 39 << 16; *reg(0xe080117c) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->sync_ack_pending && !ar_rf_receive_information()->sync_ack_submitted);
	assert(ar_rf_receive_information()->last_tx_length == 24);
	assert(ar_rf_receive_information()->last_tx_packet[12] == 2 && ar_rf_receive_information()->last_tx_packet[14] == 25);
	*reg(0x0a110f5c) = 40 << 16; *reg(0xe080117c) |= 1; ar_rf_poll();
	assert(!ar_rf_receive_information()->sync_ack_pending && ar_rf_receive_information()->sync_ack_submitted == 1);
	assert(ar_rf_receive_information()->last_tx_length == 33);
	assert(ar_mac_decode(ar_rf_receive_information()->first_sync_ack_packet, 33, &encoded) == 0);
	assert(ar_mac_payload_at(&encoded, 0, &part) == 0 && part.length == 5 && part.port == 0);
	assert(!memcmp(part.data, "\x4a\x04\x01\x4f\x00", 5));
	assert((*reg(0x0a110d40) & 0xff00ffff) == 5);
	assert(!transmit_calls && !transmit_commits); /* ACK priority, including insufficient budget. */
	if (operation >= 66) {
		*reg(0x0a110f5c) = 31 << 16; *reg(0xe080117c) |= 1; ar_rf_poll();
		assert(!transmit_calls && transmit_remaining == 140); /* Rounded budget is24. */
		*reg(0x0a110f5c) = 39 << 16; *reg(0xe080117c) |= 1; ar_rf_poll();
		assert(transmit_calls == 1 && transmit_commits == 1 && transmit_remaining == 136);
		const struct ar_rf_tx_payload_info *tx = ar_rf_tx_payload_information();
		assert(tx->packets == 1 && tx->bytes == 4 && tx->first_length == 32);
		assert(ar_mac_decode(tx->first_packet, tx->first_length, &encoded) == 0);
		assert(ar_mac_payload_at(&encoded, 0, &part) == 0 && part.port == 3 && part.length == 4);
		assert(!memcmp(part.data, transmit_source, 4));
		*reg(0x0a110f5c) = 160 << 16; *reg(0xe080117c) |= 1; ar_rf_poll();
		assert(transmit_remaining == 4 && tx->last_length == 160 && tx->bytes == 136);
		assert(ar_mac_decode(tx->last_packet, tx->last_length, &encoded) == 0);
		assert(ar_mac_payload_at(&encoded, 0, &part) == 0 && part.port == 3 && part.length == 132);
		assert(!memcmp(part.data, transmit_source + 4, 132));
		*reg(0xe080117c) |= 1; ar_rf_poll();
		assert(!transmit_remaining && transmit_commits == 3 && tx->bytes == 140);
		memset(transmit_source, 0, sizeof transmit_source);
		assert(tx->first_packet[28] == 0x5a && tx->last_packet[28] == (136 ^ 0x5a));
	}

	/* The same full login is idempotent once connected. */
	injected = 0; ar_rf_poll(); ar_rf_poll();
	*reg(0xe080117c) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->sync_ack_submitted == 1 && ar_rf_receive_information()->last_tx_length == 24);
	assert(ar_rf_receive_information()->port_packets[0] == 2);
	assert(ar_rf_receive_information()->port0_commands[2] == 2);
	/* Seven identical one-ID requests initialize then trip the configured
	 * six-miss watchdog. Only B00 bit30 changes, and the next two events
	 * restore it without retiring any owned DMA packet. */
	uint32_t b00_before = *reg(0x0a110b00);
	*reg(0x0a110f68) = 0; *reg(0x0a110f64) = 5u << 20;
	for (unsigned i = 0; i < 7; i++) { *reg(0xe0801178) |= 1; ar_rf_poll(); }
	assert(ar_rf_receive_information()->retx_disable == 1 && ar_rf_receive_information()->retx_phase == 2);
	assert(*reg(0x0a110b00) == (b00_before | (1u << 30)));
	unsigned owned_before = ar_rf_receive_information()->tx_owned;
	*reg(0xe0801178) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->retx_phase == 1 && !ar_rf_receive_information()->retx_enable);
	*reg(0xe0801178) |= 1; ar_rf_poll();
	assert(ar_rf_receive_information()->retx_enable == 1 && !ar_rf_receive_information()->retx_phase);
	assert(*reg(0x0a110b00) == b00_before && ar_rf_receive_information()->tx_owned == owned_before);
	if (operation >= 67) {
		const struct ar_rf_mcs_info *mcs = ar_rf_mcs_information();
		assert(mcs->state & 1); assert(!mcs->requests);
		*reg(0x0a110fbc) = 8 << 20; *reg(0x0a110f28) = 0; *reg(0x0a110ee0) = 500;
		for (unsigned i = 0; i < 8; i++) {
			*reg(0x0a110f00) = 0x1096; *reg(0xe0801168) |= 1; ar_rf_poll();
			*reg(0xe0801160) |= 1; ar_rf_poll();
		}
		mcs = ar_rf_mcs_information();
		assert(mcs->samples == 8 && mcs->requests == 1 && mcs->current == 5 && (mcs->state & 8));
		assert(!mcs->acknowledgements && !mcs->fch_writes);
		uint32_t sync_before = *reg(0x0a110b8c), d30_before = *reg(0x0a110d30);
		*reg(0xe080117c) |= 1; ar_rf_poll();
		assert(*reg(0x0a110b8c) == sync_before); /* Enqueue precedes source18 write. */
		assert(*reg(0x0a110d30) == ((d30_before & ~0xff0000u) | 0xff0000u));
		*reg(0x0a110f68) = 0; *reg(0xe0801178) |= 1; ar_rf_poll();
		uint8_t command[8] = {0,0xf3,5,0,0xc5,0,0,0}; command[0] = ar_rf_control_checksum(command);
		uint32_t expected0, expected1; memcpy(&expected0, command, 4); memcpy(&expected1, command + 4, 4);
		assert(*reg(0x0a110b8c) == expected0 && *reg(0x0a110b90) == expected1);
		mcs = ar_rf_mcs_information(); assert(mcs->fch_writes == 1 && mcs->advertisements == 1);
		if (operation == 77) {
			unsigned resets = mcs->rx_resets, completions = ar_rf_receive_information()->dma_completions;
			unsigned saved_valid = inject_valid, saved_receive = inject_receive;
			inject_valid = inject_receive = 0; delay_dma = 1;
			*reg(0x0a110f88) = 1; *reg(0x0a110f90) = 32; irq(16);
			ticks += 2 * 40000;
			for (unsigned i = 0; i < 19; i++) { irq(19); }
			mcs = ar_rf_mcs_information();
			assert(mcs->timeouts >= 2 && mcs->rx_resets == resets);
			assert(mcs->reset_pending && mcs->reset_deferred >= 2);
			assert(ar_rf_receive_information()->dma_completions == completions);
			irq(13); delay_dma = 0;
			assert(ar_rf_receive_information()->dma_completions == completions + 1);
			assert(mcs->rx_resets == resets + 1 && !mcs->reset_pending);
			inject_valid = saved_valid; inject_receive = saved_receive;
		}
		unsigned writes_before_ack = mcs->fch_writes, cancels_before_ack = mcs->cancellations;
		*reg(0x0a110f00) = 0x3080; *reg(0xe0801168) |= 1; ar_rf_poll();
		mcs = ar_rf_mcs_information();
		assert(mcs->acknowledgements == 1 && !(mcs->state & 8) && mcs->buffer_applies == 1);
		assert(mcs->observed_counts[5] == 1 && mcs->fch_writes == writes_before_ack + 1 && mcs->cancellations == cancels_before_ack + 1);
		if (operation >= 75) {
			assert(event_count == 4 && event_ids[3] == 1);
			assert(!memcmp(event_data[3], (uint8_t[]){0,1,5,25}, 4));
		}
		command[1] = 0xf0; command[0] = ar_rf_control_checksum(command); memcpy(&expected0, command, 4);
		assert(*reg(0x0a110b8c) == expected0 && *reg(0x0a110b90) == expected1);
		assert(*reg(0x0a110d30) == (d30_before & ~0xff0000u));
		if (operation >= 68) {
			assert(sizeof(struct ar_rf_phy_trace) == 3328);
			const struct ar_rf_phy_trace *trace = ar_rf_phy_trace_information();
			assert(trace->events && trace->buckets[0].count == 8 && !trace->buckets[1].count);
			assert(trace->buckets[0].snr_min == 500 && trace->buckets[0].snr_max == 500);
			assert(trace->buckets[0].zero_errors == 8);
			struct ar_rf_phy_event first = trace->first[0];
			uint32_t bb0 = *reg(0x0a110bb0) | 0x80, c00 = *reg(0x0a110c00) | 0x80;
			*reg(0x0a110bb0) = bb0; *reg(0x0a110c00) = c00;
			/* The MCS ACK FCH reset the EOP guard. First EOP consumes a
			 * sample; the next100 are retained only by the diagnostic. */
			for (unsigned i = 0; i < 101; i++) { *reg(0xe0801160) |= 1; ar_rf_poll(); }
			assert(ar_rf_phy_information()->samples == 9);
			assert(trace->buckets[0].count == 9 && trace->buckets[1].count == 100);
			assert(*reg(0x0a110bb0) == bb0 && *reg(0x0a110c00) == c00);
			assert(trace->first_count == 32 && trace->last_count == 32 && trace->last_head == (trace->events & 31));
			assert(!memcmp(&first, &trace->first[0], sizeof first));
			for (unsigned i = 0; i < 32; i++) {
				const struct ar_rf_phy_event *event = &trace->last[(trace->last_head + i) & 31];
				assert((event->flags & 255) == 12 && (event->flags & 256) && !(event->flags & (1u << 13)));
				assert(event->samples == 9 && (uint16_t)event->snr == 500 && event->end_us >= event->begin_us);
			}
		}
	}

	if (operation >= 69) {
		assert(sizeof(struct ar_rf_sync_info) == 128);
		const struct ar_rf_sync_info *sync = ar_rf_sync_information();
		uint32_t de0 = *reg(0x0a110de0), df0 = *reg(0x0a110df0);
		uint8_t identity[7] = {0x25,0x16,4,0x48,0xcb,0x6a,6};
		assert(ar_rf_control_checksum(identity) == identity[0]); /* Trial114 full-tail vector. */
		identity[0] ^= 1; receive_sync(identity, 0);
		assert(!sync->rx_writes && *reg(0x0a110de0) == de0);
		identity[2] = 2; receive_sync(identity, 1);
		assert(sync->rejected && !sync->rx_writes); /* Correct check, wrong length. */
		identity[2] = 4; identity[1] = 0x56; receive_sync(identity, 1);
		assert(sync->pending && !sync->rx_writes); /* Do not apply at TTL5. */
		ticks += 10 * 40000; ar_rf_poll(); assert(!sync->rx_writes);
		identity[1] = 0x16; receive_sync(identity, 1);
		assert(sync->rx_writes == 1 && !sync->pending && sync->identity == 0xcb && sync->user == 8);
		assert(*reg(0x0a110de0) == ((de0 & ~255u) | 0xcb));
		assert(*reg(0x0a110df0) == ((df0 & ~255u) | 0xcb));
		ticks += 20 * 40000; ar_rf_poll(); assert(sync->rx_writes == 1); /* Canceled fallback. */
		identity[1] = 0x26; identity[4] = 0x55; receive_sync(identity, 1);
		ticks += 8 * 40000; ar_rf_poll(); assert(sync->rx_writes == 1);
		ticks += 2 * 40000; ar_rf_poll(); /* Missing TTL1: 5ms frame +4ms fallback. */
		assert(sync->rx_writes == 2 && !sync->pending && sync->timer_fired == 1);
		assert((*reg(0x0a110de0) & 255) == 0x55);
		identity[4] = 0x66; receive_sync(identity, 1);
		uint8_t idle_header[7] = {0}; receive_sync(idle_header, 1);
		ticks += 10 * 40000; ar_rf_poll(); assert(sync->rx_writes == 2); /* Latest cache is idle. */
		identity[1] = 0xf6; identity[2] = 5; identity[3] = 0x58;
		uint32_t tx8 = *reg(0x0a110b84), txshared = *reg(0x0a110b60);
		receive_sync(identity, 1); assert(sync->tx_writes == 1);
		assert(*reg(0x0a110b84) == ((tx8 & ~0xff0000u) | 0x660000));
		assert(*reg(0x0a110b60) == ((txshared & ~0xff0000u) | 0x660000));
		identity[3] = 0x49; uint32_t rx9 = *reg(0x0a110de8), shared = *reg(0x0a110df0);
		receive_sync(identity, 1); assert(sync->rx_writes == 3);
		assert(*reg(0x0a110de8) == ((rx9 & ~255u) | 0x66) && *reg(0x0a110df0) == shared);
		identity[3] = 0x4a; receive_sync(identity, 1); assert(sync->invalid_users == 1 && sync->rx_writes == 3);
		/* AP time passes through the real FCH checksum/length gates. A valid
		 * time header replaces a queued identity without applying its timer. */
		identity[1] = 0x26; identity[2] = 4; identity[3] = 8;
		receive_sync(identity, 1); assert(sync->pending);
		uint8_t clock[7] = {0,0xa7,9,0x67,0x45,0x23,0xf1};
		receive_sync(clock, 1);
		uint32_t time_before = ar_rf_host_ap_time();
		assert(time_before - UINT32_C(0xa1234567) <= 1);
		assert(sync->pending && sync->rx_writes == 3);
		unsigned unsupported_before = sync->unsupported;
		ticks += 20 * 40000; ar_rf_poll();
		assert(!sync->pending && sync->rx_writes == 3 && sync->unsupported == unsupported_before);
		assert(ar_rf_host_ap_time() - time_before >= 20 && ar_rf_host_ap_time() - time_before <= 21);
		clock[3] = 0; /* Corrupt the check without recomputing it. */
		receive_sync(clock, 0);
		assert(ar_rf_host_ap_time() - time_before <= 21);
		clock[2] = 6; receive_sync(clock, 1); /* Correct check, wrong length. */
		assert(ar_rf_host_ap_time() - time_before <= 21);
		/* The original has no command9 callback, including shared expiry.
		 * A command0 wildcard length7 also safely replaces an armed cache. */
		unsigned dispatched_before = ar_rf_traffic_information()->commands[9].dispatched;
		uint8_t no_callback[7] = {0,0x29,0,0,0,0,0};
		receive_sync(no_callback,1); assert(sync->pending);
		ticks += 10 * 40000; ar_rf_poll();
		assert(!sync->pending && sync->unsupported == unsupported_before);
		no_callback[1] = 0xf9; no_callback[2] = 1; receive_sync(no_callback,1);
		assert(sync->unsupported == unsupported_before);
		assert(ar_rf_traffic_information()->commands[9].dispatched == dispatched_before);
		receive_sync(identity,1); assert(sync->pending);
		no_callback[1] = 0x10; no_callback[2] = 14; receive_sync(no_callback,1);
		ticks += 10 * 40000; ar_rf_poll();
		assert(!sync->pending && sync->rx_writes == 3 && sync->unsupported == unsupported_before);
	}
	if (operation >= 72) {
		assert(sizeof(struct ar_rf_tx_mcs_info) == 64);
		const struct ar_rf_tx_mcs_info *txm = ar_rf_tx_mcs_information();
		uint8_t request[7] = {0,0xf3,5,0,0xc2,0x83,6};
		uint32_t initial = *reg(0x0a110b00);
		/* Idempotent repeated request; a mode2 TX field becomes mode1 for
		 * normal-table MCS2. All unrelated fields and duration are retained. */
		*reg(0x0a110b00) = (initial & ~0xc00000u) | 0x800000u;
		uint32_t duration = *reg(0x0a110a00);
		receive_sync(request, 1);
		assert(txm->requests == 1 && txm->applied == 1 && txm->current == 2 && txm->mode_changed == 1);
		assert(*reg(0x0a110b00) == ((initial & ~0xc3f700u) | 0x406000u));
		assert(*reg(0x0a110a00) == duration);
		uint32_t applied = *reg(0x0a110b00);
		receive_sync(request, 1); assert(txm->applied == 1 && txm->unchanged == 1);
		request[3] = 1; receive_sync(request, 1); assert(txm->rejected == 1);
		request[3] = 0; request[4] = 0xd4; receive_sync(request, 1); assert(txm->rejected == 2); /* MCS 20; 25+ announces a width. */
		request[4] = 0xc1; receive_sync(request, 1); assert(txm->downgrades == 1 && txm->current == 2);
		assert(*reg(0x0a110b00) == applied);
		txm = ar_rf_tx_mcs_information(); assert(txm->pending == 0x10601);
		assert(sizeof(struct ar_rf_tx_history_info) == 32);
		const struct ar_rf_tx_history_info *history = ar_rf_tx_history_information();
		assert(!history->history && !history->eligible);
		uint32_t saved_budget = *reg(0x0a110f5c), saved_request = *reg(0x0a110f58);
		/* History is an attempt, even when zero budget prevents submission. */
		*reg(0x0a110f5c) = 0; *reg(0x0a110f70) = 0;
		unsigned packets_before = ar_rf_receive_information()->tx_packets;
		unsigned samples_before = history->samples;
		*reg(0xe080117c) |= 1; ar_rf_poll();
		assert(ar_rf_tx_history_information()->samples == samples_before + 1);
		assert(!history->history && !history->eligible); /* Drain has not started. */
		request[4] = 0xc0; receive_sync(request,1);
		assert(txm->busy_requests == 1 && ar_rf_tx_mcs_information()->pending == 0x10601);
		*reg(0x0a110f68) = 8u << 4; *reg(0xe0801178) |= 1; ar_rf_poll();
		assert(ar_rf_tx_mcs_information()->pending == 0x10601);
		*reg(0x0a110f68) = 0;
		for (unsigned i = 0; i < 6; ++i) {
			*reg(0x0a110f64) = (i + 6) << 20;
			*reg(0xe0801178) |= 1; ar_rf_poll();
			assert(txm->current == 2 && !txm->drained);
			assert(ar_rf_tx_mcs_information()->pending == (0x10001 | (5 - i) << 8));
			*reg(0x0a110b00) = (*reg(0x0a110b00) & ~16u) | (i ? 16u : 0);
			*reg(0x0a110f58) = i ? 1 : 0;
			*reg(0xe080117c) |= 1; ar_rf_poll();
			assert(ar_rf_tx_history_information()->history == (i ? 3u : 1u));
			assert(history->eligible == i + 1 && ((history->bookkeeping >> 8) & 0xffff) == 1);
		}
		unsigned eligible = history->eligible;
		*reg(0x0a110f70) = 15u << 12; *reg(0xe080117c) |= 1; ar_rf_poll();
		assert(ar_rf_tx_history_information()->last_user == 15 && history->other_users == 1);
		assert(history->eligible == eligible && history->history == 3);
		*reg(0x0a110f70) = 0;
		*reg(0x0a110b00) = (*reg(0x0a110b00) & ~16u) | (applied & 16u);
		*reg(0x0a110f64) = 12u << 20;
		*reg(0xe0801178) |= 1; ar_rf_poll();
		assert(txm->current == 1 && txm->drained == 1 && txm->applied == 2);
		assert(ar_rf_tx_mcs_information()->pending == 0x10000);
		assert((*reg(0x0a110b00) & 0x3f700u) == 0x13000 && *reg(0x0a110a00) == duration);
		*reg(0xe080117c) |= 1; ar_rf_poll();
		assert(ar_rf_tx_history_information()->history == 3 && history->eligible == eligible);
		assert(!(history->bookkeeping & 255) && history->mcs == 1 && history->previous_mcs == 2);
		assert(ar_rf_receive_information()->tx_packets == packets_before);
		*reg(0x0a110f5c) = saved_budget; *reg(0x0a110f58) = saved_request;
	}
	/* Compact CRC integrity and token continuity are separate gates. */
	uint8_t compact_control[6] = {0,23,9,2,0,0};
	for (unsigned i = 1; i < 6; i++) { compact_control[0] = ar_mac_crc8_step(compact_control[0] ^ compact_control[i]); }
	uint8_t data_payload[140]; memset(data_payload, 0xa6, sizeof data_payload);
	part.data = data_payload; part.length = sizeof data_payload; part.port = 1;
	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 3, 0, compact_control, 6, &part, 1, &packet_size) == 0);
	*reg(0x0a110f90) = packet_size; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->compact_valid == 1 && ar_rf_receive_information()->dev_state == 2);
	assert(ar_rf_receive_information()->first_data_port == 1 && ar_rf_receive_information()->first_data_length == 140);
	assert(ar_rf_receive_information()->port_bytes[1] == 140);
	assert(!memcmp(ar_rf_receive_information()->first_data, data_payload, 128));
	assert(forwarded_count == 1 && forwarded_slot == 0 && forwarded_port == 1 && forwarded_length == 140);
	assert(!memcmp(forwarded, data_payload, 140));
	uint8_t other_peer[sizeof login_control]; memcpy(other_peer, login_control, sizeof other_peer);
	other_peer[8] ^= 1; other_peer[2] = 0; checksum = 0;
	for (unsigned i = 0; i < sizeof other_peer; i++) { checksum = ar_mac_crc8_step(checksum ^ other_peer[i]); }
	other_peer[2] = checksum;
	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 4, 0, other_peer, sizeof other_peer, &part, 1, &packet_size) == 0);
	*reg(0x0a110f90) = packet_size; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(forwarded_count == 1 && ar_rf_receive_information()->dev_state == 2);
	/* One internal command split across two separate MAC descriptors/frames. */
	uint8_t split_command[8] = {0x4a,3,4,0x49,1,2,3,4};
	part.port = 0; part.data = split_command; part.length = 3;
	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 4, 0, compact_control, 6, &part, 1, &packet_size) == 0);
	*reg(0x0a110f90) = packet_size; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(!ar_rf_receive_information()->stream_commands[3] && ar_rf_receive_information()->stream_buffered == 3);
	part.data = split_command + 3; part.length = 5;
	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 5, 0, compact_control, 6, &part, 1, &packet_size) == 0);
	*reg(0x0a110f90) = packet_size; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->stream_commands[3] == 1 && !ar_rf_receive_information()->stream_buffered);
	assert(ar_rf_receive_information()->first_nonstats_length == 8);
	assert(!memcmp(ar_rf_receive_information()->first_nonstats, split_command, 8));
	compact_control[0] ^= 1;
	part.port = 4; part.data = data_payload; part.length = sizeof data_payload;
	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 4, 0, compact_control, 6, &part, 1, &packet_size) == 0);
	*reg(0x0a110f90) = packet_size; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->compact_invalid == 1 && ar_rf_receive_information()->dev_state == 2);
	assert(forwarded_count == 1); /* Bad control must not forward payload. */
	/* A valid compact with a changed token prevents further transmission. */
	if (operation != 77) {
	compact_control[2]++; compact_control[0] = 0;
	for (unsigned i = 1; i < 6; i++) { compact_control[0] = ar_mac_crc8_step(compact_control[0] ^ compact_control[i]); }
	assert(ar_mac_encode(valid_packet, sizeof valid_packet, 5, 0, compact_control, 6, &part, 1, &packet_size) == 0);
	*reg(0x0a110f90) = packet_size; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(ar_rf_receive_information()->link_invalidations == 1 && ar_rf_receive_information()->dev_state == 0);
	assert(forwarded_count == 1); /* Token invalidation precedes delivery. */
	if (operation >= 69) {
		uint8_t identity[7] = {0,0x16,4,8,0x77,0,0};
		unsigned count = ar_rf_sync_information()->rx_writes;
		receive_sync(identity, 1);
		assert(ar_rf_sync_information()->rx_writes == count && ar_rf_sync_information()->state_skips);
	}
	unsigned calls_before = transmit_calls;
	transmit_remaining = 1;
	*reg(0xe080117c) |= 1; ar_rf_poll();
	assert(transmit_calls == calls_before && transmit_remaining == 1);

	assert((*reg(0x0a110800) & 4) && !(*reg(0x0a110cf4) & (1u << 21)));
	}
	if (operation >= 71) {
		const struct ar_rf_traffic_info *traffic = ar_rf_traffic_information();
		assert(sizeof *traffic == 800 && traffic->version == 1);
		unsigned accepted = 0, dispatched = 0;
		for (unsigned i = 0; i < 10; i++) {
			const struct ar_rf_command_observation *command = &traffic->commands[i];
			accepted += command->accepted; dispatched += command->dispatched;
			if (!command->accepted) { continue; }
			assert((command->first[1] & 15) == i && (command->last[1] & 15) == i);
			assert(command->first_us <= command->last_us);
			assert(ar_rf_control_checksum(command->first) == command->first[0]);
			assert(ar_rf_control_checksum(command->last) == command->last[0]);
		}
		assert(accepted == ar_rf_sync_information()->accepted && dispatched >= 6);
		assert(traffic->commands[6].accepted >= (operation == 77 ? 7u : 8u));
		assert(traffic->commands[6].dispatched >= (operation == 77 ? 5u : 6u));
		for (unsigned i = 0; i < 5; i++) {
			unsigned total = 0;
			for (unsigned second = 0; second < 16; second++) { total += traffic->port_bytes_by_second[second][i]; }
			assert(total == traffic->ports[i].bytes && total == ar_rf_receive_information()->port_bytes[i]);
			assert(traffic->ports[i].packets == ar_rf_receive_information()->port_packets[i]);
			assert(traffic->ports[i].first_us <= traffic->ports[i].last_us);
		}
	}
	if (operation == 77) {
		/* Session survives more than one low-counter wrap. Control timers use
		 * elapsed milliseconds even after the diagnostic microseconds wrap. */
		const struct ar_rf_sync_info *sync = ar_rf_sync_information();
		uint8_t identity[7] = {0,0x26,4,0x48,0x55,0,0};
		for (unsigned epoch = 0; epoch < 3; epoch++) {
			uint64_t advance = epoch == 0 ? UINT64_C(240000) * 40000 :
				epoch == 1 ? (UINT64_C(1) << 32) * 40 : (UINT64_C(1) << 32) * 40000;
			if (epoch == 2) {
				/* Arm a 9-ms timer four milliseconds before 32-bit ms wrap. */
				ticks = session_started + ((UINT64_C(1) << 32) - 4) * 40000;
			} else { ticks += advance; }
			ar_rf_poll();
			assert(ar_rf_information()->phase == AR_RF_RECEIVING);
			/* Force a fresh EOP sample and check MCS uses the millisecond
			 * clock, not sample_elapsed_us / 1000 after that field wraps. */
			if (!epoch) { tear_timer_read = 1; } /* Low wraps inside high/low/high. */
			*reg(0x0a110f00) = 8 << 9; *reg(0xe0801168) |= 1; ar_rf_poll();
			*reg(0x0a110fbc) = 8 << 20; *reg(0xe0801160) |= 1; ar_rf_poll();
			uint32_t expected_ms = (uint32_t)((ticks - session_started) / 40000);
			uint32_t measured_ms = ar_rf_mcs_information()->sample_ms;
			assert((uint32_t)(measured_ms - expected_ms + 1) <= 2);
			unsigned fired = sync->timer_fired;
			receive_sync(identity, 1); assert(sync->pending);
			ticks += 8 * 40000; ar_rf_poll(); assert(sync->pending && sync->timer_fired == fired);
			ticks += 2 * 40000; ar_rf_poll(); assert(!sync->pending && sync->timer_fired == fired + 1);
		}
		/* One million polls is a diagnostic watchdog, never a normal-session
		 * lifetime. Keep the active radio alive beyond that count as well. */
		for (unsigned i = 0; i < 1000001; i++) {
			/* The peer remains present during this long-session test. */
			if (!(i % 100)) { *reg(0x0a110f00) = 0x1096; *reg(0xe0801168) |= 1; }
			ar_rf_poll();
		}
		assert(ar_rf_information()->phase == AR_RF_RECEIVING);
		assert(ar_rf_stop_host() == 0 && ar_rf_stop_host() == 0);
		assert(!sync->pending && (*reg(0x0a110800) & 5) == 5);
		unsigned stopped_writes = writes;
		ar_rf_poll(); assert(writes == stopped_writes);
	}
	ticks += 100 * 40000; ar_rf_poll(); inject_valid = 0;
	if (operation >= 67) { assert(ar_rf_host_status(0, host_status) == 0 && !host_status[252] && host_status[253] == 25); }
	if (operation >= 76) {
		assert(ar_rf_host_candidates(candidate_request, 402) == (operation == 77 ? 0 : -11));
		assert(ar_rf_information()->phase == AR_RF_PSRAM); /* Explicit stop does not auto-start. */
	}
	if (operation >= 75) {
		unsigned tx_events = 0, rx_events = 0;
		for (unsigned i = 3; i + 1 < event_count; i++) {
			assert(event_ids[i] == 1 && event_data[i][0] == 0);
			if (event_data[i][1]) { rx_events++; }
			else {
				const uint8_t expected[][4] = {{0,0,2,1}, {0,0,1,2}};
				assert(tx_events < 2 && !memcmp(event_data[i], expected[tx_events], 4));
				tx_events++;
			}
		}
		assert(tx_events == 2 && rx_events == ar_rf_mcs_information()->buffer_applies);
		assert(!event_ids[event_count - 1] && !memcmp(event_data[event_count - 1], (uint8_t[]){0,0,2}, 3));
	} else { assert(!event_count); }
	}
	ar_rf_set_event_callback(NULL, NULL);
	/* EOP measurement layout, selector restoration, user9 alias, and the
	 * duplicate guard reset by each FCH. These are synthetic register inputs. */
	inject_receive = 0; inject_header = 0;
	before = writes;
	assert(ar_rf_monitor_plan(5770000, 15001, 4, 70, mcs_plan, sizeof mcs_plan) == -22 && writes == before);
	assert(ar_rf_monitor_plan(5770000, 10001, 4, 69, mcs_plan, sizeof mcs_plan) == -22 && writes == before);
	assert(ar_rf_monitor_plan(5770000, 15000, 4, 70, mcs_plan, sizeof mcs_plan) == 0);
	ticks += 10001 * 40000; ar_rf_poll(); assert(ar_rf_information()->phase == AR_RF_RECEIVING);
	ticks += 5000 * 40000; ar_rf_poll(); assert(ar_rf_information()->phase == AR_RF_PSRAM);
	assert(ar_rf_monitor_plan(5770000, 100, 4, 64, connected_plan, sizeof connected_plan) == 0);
	assert(sizeof(struct ar_rf_phy_info) == 88);
	assert(!ar_rf_phy_trace_information()->events);
	assert(!ar_rf_sync_information()->accepted && !ar_rf_sync_information()->pending);
	assert(!ar_rf_traffic_information()->commands[6].accepted && !ar_rf_traffic_information()->ports[0].bytes);
	assert(!ar_rf_tx_mcs_information()->requests && ar_rf_tx_mcs_information()->current == 1);
	assert(!ar_rf_tx_history_information()->samples && !ar_rf_tx_history_information()->history);
	measurement_banks = 1;
	*reg(0x0a110f00) = (9 << 9) | 0x80;
	*reg(0xe0801168) |= 1; ar_rf_poll();
	*reg(0x0a110bb0) = 0x12348080; *reg(0x0a110c00) = 0x98765480;
	*reg(0x0a110808) = 0xdeadbeef;
	*reg(0x0a110f28) = 0x01230000; *reg(0x0a110ee0) = 0x98764567;
	*reg(0x0a110ee4) = 0x7654329a; *reg(0x0a110fbc) = 9 << 20;
	*reg(0xe0801160) |= 1; ar_rf_poll();
	const struct ar_rf_phy_info *m = ar_rf_phy_information();
	const uint8_t expected_sample[16] = {0x67,0x45,0,0,0x23,1,12,12,0x9a,0,0,0,0x56,0x78,0x12,0x34};
	assert(m->samples == 1 && !m->forced_errors);
	assert(!memcmp(m->sample, expected_sample, 16) && !memcmp(m->first_sample, expected_sample, 16));
	assert(*reg(0x0a110bb0) == 0x12348080 && *reg(0x0a110c00) == 0x98765480 && *reg(0x0a110808) == 0xdeadbeef);
	*reg(0xe0801160) |= 1; ar_rf_poll();
	assert(m->samples == 1 && m->duplicates == 1);
	*reg(0x0a110f00) = 8 << 9; *reg(0xe0801168) |= 1; ar_rf_poll();
	*reg(0xe0801160) |= 1; ar_rf_poll();
	assert(m->samples == 2 && m->forced_errors == 1 && m->sample[2] == 1);
	assert(!memcmp(m->first_sample, expected_sample, 16));
	*reg(0x0a110fbc) = 15 << 20; *reg(0xe0801160) |= 1; ar_rf_poll();
	assert(m->samples == 2 && m->other_users == 1);
	measurement_banks = 0; ticks += 100 * 40000; ar_rf_poll();
	assert(ar_rf_monitor_plan(5770000, 100, 4, 64, connected_plan, sizeof connected_plan) == 0);
	assert(!ar_rf_phy_information()->samples && !ar_rf_phy_information()->duplicates);
	ticks += 100 * 40000; ar_rf_poll();
	/* Normal config search uses the supplied list and keeps the DEV channel
	 * index synchronized with hardware retuning before admission. */
	uint8_t search_plan[sizeof mcs_plan + 4]; memcpy(search_plan,mcs_plan,sizeof mcs_plan);
	put32(search_plan + sizeof mcs_plan,5805000);
	struct ar_power_config loader_power = {.valid=0x7fffffff,.closed={1,1},
		.high_clock=1,.dev_role=1,.path_offset={-1,1},.csma_offset=8};
	for (unsigned i = 0; i < AR_POWER_LEVELS; i++) {
		loader_power.calibration[i][0] = 12+i;
		loader_power.calibration[i][1] = 2*i;
		loader_power.calibration[i][2] = 45-i;
		loader_power.calibration[i][3] = 3*i;
	}
	memcpy(loader_power.calibration[15],search_plan+24,2);
	memcpy(loader_power.calibration[17]+2,search_plan+26,2);
	struct ar_power_config saved_power = loader_power;
	/* An AP restart must still advertise the grounds supplied by the host,
	 * without requiring another SET candidates before they can log in. */
	{
		uint8_t ap_plan[sizeof search_plan];
		memcpy(ap_plan, search_plan, sizeof ap_plan); memcpy(ap_plan, "AP04", 4);
		struct ar_power_config ap_power = saved_power; ap_power.dev_role = 0;
		assert(!ar_rf_prepare_config(5770000,4,ap_plan,sizeof ap_plan,0,&ap_power));
		uint8_t grounds[402] = {0,2,0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88};
		assert(!ar_rf_host_candidates(grounds,sizeof grounds));
		assert(!ar_rf_host_control(0,0));
		for (unsigned cycle = 0; cycle < 3; cycle++) {
			*reg(0x0a110f70) = 8u << 12; *reg(0x0a110f5c) = 128u << 16;
			*reg(0xe080117c) |= 1; ar_rf_poll();
			const struct ar_rf_ap_info *ap = ar_rf_ap_information();
			struct ar_mac_frame frame;
			assert(!ar_mac_decode(ap->last_packet,sizeof ap->last_packet,&frame));
			assert(frame.outband_length >= 41 && frame.outband[32] == 2);
			assert(!memcmp(frame.outband + 33,grounds + 2,8));
			assert(!ar_rf_host_control(1,0));
			if (cycle < 2) { assert(!ar_rf_host_control(0,0)); }
		}
	}
	before_writes = writes;
	assert(ar_rf_prepare_config(5770000,4,search_plan,sizeof search_plan,1,NULL)==-22);
	loader_power.valid &= ~(1u<<15);
	assert(ar_rf_prepare_config(5770000,4,search_plan,sizeof search_plan,1,&loader_power)==-95);
	loader_power = saved_power; loader_power.calibration[15][1] ^= 1;
	assert(ar_rf_prepare_config(5770000,4,search_plan,sizeof search_plan,1,&loader_power)==-22);
	loader_power = saved_power; loader_power.closed[1] = 0;
	assert(ar_rf_prepare_config(5770000,4,search_plan,sizeof search_plan,1,&loader_power)==-22);
	assert(writes==before_writes);
	loader_power = saved_power;
	assert(ar_rf_prepare_config(5770000,4,search_plan,sizeof search_plan,1,&loader_power)==0);
	/* The caller can reuse its storage; runtime owns calibration and cache. */
	memset(&loader_power,0,sizeof loader_power);
	assert(!memcmp(&ar_rf_power_information()->config,&saved_power,sizeof saved_power));
	assert(ar_rf_power_information()->level[0]==25 && !ar_rf_power_information()->level[11]);
	assert((*reg(0x0a1109a4)&0x3fff)==((unsigned)search_plan[24]<<8|search_plan[25]));
	assert((*reg(0x0a110da4)&63)==search_plan[26] && (*reg(0x0a110da0)>>24)==search_plan[27]);

	assert(*reg(0x0a110810) == 0x183a113a && *reg(0x0a110814) == 0x00802309);
	assert(*reg(0x0a110870) == 0x0f00dd88 && *reg(0x0a1108a4) == 0xe000);
	assert_channel_plls(5770000);
	uint8_t candidates[402]={0,0};
	assert(ar_rf_host_candidates(candidates,sizeof candidates)==0);
	assert(ar_rf_receive_information()->channel_count==2 && ar_rf_receive_information()->frequency_khz==5770000);
	ticks += 11*40000; ar_rf_poll();
	assert(ar_rf_receive_information()->channel_index==1 && ar_rf_receive_information()->frequency_khz==5805000);
	assert_channel_plls(5805000);
	assert(*reg(0x0a110800) & 4); /* TX stays held until admission. */
	const struct ar_rf_rx_info *search = ar_rf_receive_information();
	unsigned restarts = search->search_restarts;
	uint8_t candidate_header[7] = {0};
	receive_sync(candidate_header,1);
	ticks += search->search_frame_us * 40;
	receive_sync(candidate_header,1);
	assert(search->search_suspended && search->search_restarts == restarts);
	ticks += (search->search_frame_us + 1000) * 40;
	receive_sync(candidate_header,1);
	assert(search->search_restarts == restarts + 1 && !search->search_suspended);
	assert_channel_plls(5770000); /* Repeated good headers did not extend grace. */
	/* A completed but unadmitted packet cannot latch search forever. Pending
	 * DMA does prevent retuning even well beyond the ordinary dwell. */
	assert(ar_mac_encode(valid_packet,sizeof valid_packet,0,0,control,sizeof control,NULL,0,&packet_size)==0);
	*reg(0x0a110f88) = 1; *reg(0x0a110f90) = packet_size;
	inject_valid = inject_receive = delay_dma = 1; injected = 0;
	ar_rf_poll(); restarts = search->search_restarts;
	ticks += 100 * 40000; ar_rf_poll();
	assert(search->search_suspended && search->search_restarts == restarts);
	delay_dma = 0; *reg(0xe0801164) |= 1; ar_rf_poll();
	assert(search->admission_rejects == 1 && !search->acquisition_ready);
	assert(search->search_restarts == restarts + 1 && search->channel_index == 1);
	assert_channel_plls(5805000);
	/* Admission on the second channel sees the updated DEV channel index,
	 * releases TX and prevents further scan retuning. */
	candidates[1] = 1; memcpy(candidates+2,control+8,4);
	assert(!ar_rf_host_candidates(candidates,sizeof candidates));
	control[6] = control[7] = 1; control[2] = 0; checksum = 0;
	for (unsigned i = 0; i < sizeof control; i++) { checksum = ar_mac_crc8_step(checksum ^ control[i]); }
	control[2] = checksum;
	assert(!ar_mac_encode(valid_packet,sizeof valid_packet,1,0,control,sizeof control,NULL,0,&packet_size));
	*reg(0x0a110f90) = packet_size; injected = 0; ar_rf_poll(); ar_rf_poll();
	assert(search->acquisition_ready && search->transition_status == 1 && !(*reg(0x0a110800) & 4));
	restarts = search->search_restarts; ticks += 100 * 40000; ar_rf_poll();
	assert(search->search_suspended && search->search_restarts == restarts);
	/* Load the calibrated context through a real preparation notification.
	 * Restart and recovery must retain it even if pre-encode is polled before
	 * the next preparation IRQ. A register equality check after reloading
	 * would miss the unsafe interval, so also count compensation writes. */
	/* Out-of-order TX cannot select the RX entry or load compensation. */
	unsigned prepare_before = compensation_writes, board_before = board_prepare_writes;
	*reg(0x0a110e6c)=0; inject_command=1; ar_rf_poll();
	assert(ar_rf_command_information()->cursor==-1);
	assert(compensation_writes==prepare_before && board_prepare_writes==board_before);
	prepare_frame();
	assert(board_prepare_writes == board_before + 6);
	assert(*reg(0x0a110480) & 2);
	const uint32_t compensation_addresses[] = {0x0a110480,0x0a11048c,0x0a1104a0,0x0a1104a4,0x0a1104a8};
	uint32_t compensation_saved[5];
	for (unsigned i=0;i<5;i++) { compensation_saved[i]=*reg(compensation_addresses[i]); }
	unsigned loaded_writes = compensation_writes;
	board_before = board_prepare_writes;
	inject_command = 1; ar_rf_poll();
	assert(ar_rf_command_information()->cursor==-1 && compensation_writes == loaded_writes);
	prepare_frame();
	assert(compensation_writes==loaded_writes && board_prepare_writes==board_before);
	/* The measurement PLL follows its own two-channel sweep after link lock.
	 * Delayed power before a fresh prepare is counted as unattributed. */
	const struct ar_rf_scan_info *scan=ar_rf_scan_information();
	assert(scan->version==2 && scan->count==2 && scan->frequency_khz==5770000);
	assert(!scan->score_rounds && !scan->score_ready && !scan->channel_scores[0] && !scan->channel_scores[1]);
	unsigned scan_samples=scan->samples, scan_dropped=scan->dropped;
	*reg(0xe0801144)|=1; ar_rf_poll(); assert(scan->dropped==scan_dropped+1);
	for (unsigned cycle=0;cycle<20;cycle++) {
		*reg(0x0a110e6c)=0x380000; inject_command=1; ar_rf_poll();
		*reg(0x0a110e6c)=0x180000; inject_command=1; ar_rf_poll();
		assert(scan->channel==(cycle+1)%2 && scan->phase==1);
		assert(ar_rf_command_information()->entries[1].frequency==scan->frequency_khz);
		assert(ar_rf_command_information()->entries[0].frequency==5805000);
		assert(ar_rf_command_information()->entries[2].frequency==5805000);
		*reg(0x0a110e6c)=0x3d0000; inject_command=1; ar_rf_poll();
		assert(scan->phase==2);
		*reg(0x0a110e70)=0x78563412; *reg(0xe0801144)|=1; ar_rf_poll();
		assert(scan->samples==scan_samples+cycle+1 && !scan->phase);
		if (cycle<4) { assert(scan->channel_scores[scan->channel]==(cycle<2?-90:-85)); }
		uint32_t score_rounds=scan->score_rounds, score_ring=scan->score_ring;
		int32_t score=scan->channel_scores[scan->channel];
		*reg(0xe0801144)|=1; ar_rf_poll(); /* Duplicate cannot be assigned. */
		assert(scan->samples==scan_samples+cycle+1);
		assert(scan->score_rounds==score_rounds && scan->score_ring==score_ring && scan->channel_scores[scan->channel]==score);
		*reg(0x0a110e6c)=0; inject_command=1; ar_rf_poll();
	}
	assert(scan->channel_samples[0]==10 && scan->channel_samples[1]==10);
	assert(scan->score_rounds==10 && !scan->score_ring && scan->score_ready==1);
	assert(compensation_writes==loaded_writes);
	const struct ar_rf_lifecycle_info *life = ar_rf_lifecycle_information();
	assert(life->version == 1 && life->state == 2);
	unsigned starts = life->starts, stops = life->stops, cancelled = life->cancelled_rx;
	before = writes;
	assert(ar_rf_host_control(0,0)==-7 && ar_rf_host_control(2,0)==-7 && ar_rf_host_control(3,0)==-7);
	assert(ar_rf_host_control(1,1)==-22 && ar_rf_host_control(4,0)==-95 && writes==before);
	for (unsigned cycle=0; cycle<3; cycle++) {
		uint8_t clock[7] = {0,7,8,0x40,0x42,0x0f,0}; /* 1,000,000 ms. */
		receive_sync(clock,1);
		uint32_t old_time = ar_rf_host_ap_time(); uint64_t old_ticks = ticks;
		assert(old_time - 1000000 <= 1);
		/* A DMA-complete indication is consumed without forwarding its packet
		 * during stop; no ring slot can be reused while DMA remains owned. */
		if (cycle == 1) { injected = 0; ar_rf_poll(); }
		assert(ar_rf_host_control(1,0)==0 && life->stops==stops+cycle+1);
		assert(ar_rf_lifecycle_information()->state==1 && (*reg(0x0a110800)&13)==13);
		unsigned stopped_writes=writes;
		ar_rf_poll(); assert(writes==stopped_writes);
		assert(ar_rf_host_control(1,0)==-7);
		assert(!ar_rf_host_status(0x3ff,host_status) && !host_status[252]);
		assert(!ar_rf_host_candidates(candidates,sizeof candidates));
		assert(ar_rf_information()->phase==AR_RF_PSRAM);
		if (cycle == 1) {
			assert(!ar_rf_host_control(3,0) && ar_rf_lifecycle_information()->state==0);
			assert(ar_rf_host_control(3,0)==-7 && ar_rf_host_control(0,0)==-7);
			assert(ar_rf_host_status(0x3ff,host_status)==-11);
			assert(ar_rf_host_candidates(candidates,sizeof candidates)==-11);
			assert(!ar_rf_host_control(2,0) && ar_rf_lifecycle_information()->state==1);
			assert(ar_rf_host_ap_time()==UINT32_MAX);
			assert(!ar_rf_host_candidates(candidates,sizeof candidates));
			assert(ar_rf_information()->phase==AR_RF_PSRAM); /* Init is not start. */
		}
		assert(!ar_rf_host_control(0,0) && life->starts==starts+cycle+1);
		assert(!scan->score_rounds && !scan->score_ready && !scan->channel_scores[0] && !scan->channel_scores[1]);
		assert(ar_rf_lifecycle_information()->state==2 && !search->acquisition_ready);
		assert(compensation_writes == loaded_writes);
		for (unsigned i=0;i<5;i++) { assert(*reg(compensation_addresses[i])==compensation_saved[i]); }
		if (cycle == 1) { assert(ar_rf_host_ap_time()==UINT32_MAX); }
		else {
			uint32_t elapsed = (uint32_t)((ticks-old_ticks)/40000);
			assert(ar_rf_host_ap_time()-old_time-elapsed <= 1);
		}
		assert_channel_plls(5770000);
		ticks += 11*40000; ar_rf_poll(); assert_channel_plls(5805000);
		/* The latest host candidates survived every native state rebuild. */
		injected = 0; ar_rf_poll(); ar_rf_poll();
		assert(search->acquisition_ready && search->transition_status==1);
		*reg(0xe080117c) |= 1; ar_rf_poll(); /* Pre-encode precedes prepare. */
		assert(compensation_writes == loaded_writes && (*reg(0x0a110480)&2));
		prepare_frame();
		assert(compensation_writes == loaded_writes);
	}
	assert(life->cancelled_rx==cancelled+1);
	/* Normal sessions reacquire autonomously. Unrelated PHY users cannot
	 * refresh logical user8, and loss does not require any MAC/video packet. */
	const struct ar_rf_recovery_info *recovery = ar_rf_recovery_information();
	unsigned recovered = recovery->completed, lost = recovery->fch_losses;
	inject_receive = 0;
	*reg(0x0a110f00) = 0x1096; *reg(0xe0801168) |= 1; ar_rf_poll();
	unsigned threshold = recovery->monitor_threshold, interval = recovery->monitor_interval_ms;
	assert(threshold && interval);
	for (unsigned i=0;i<threshold;i++) {
		ticks += interval * 40000;
		*reg(0x0a110f00) = 0x80; *reg(0xe0801168) |= 1; ar_rf_poll();
	}
	assert(recovery->completed==recovered+1 && recovery->fch_losses==lost+1 && !recovery->failures);
	assert(compensation_writes == loaded_writes);
	for (unsigned i=0;i<5;i++) { assert(*reg(compensation_addresses[i])==compensation_saved[i]); }
	assert(ar_rf_information()->phase==AR_RF_RECEIVING && !search->acquisition_ready && !search->dev_state);
	assert_channel_plls(5770000);
	/* Candidate removal drains a pending DMA without delivering its old
	 * packet. Latest configuration remains available for later re-admission. */
	for (unsigned cycle=0;cycle<2;cycle++) {
		ticks += 11*40000; ar_rf_poll(); assert_channel_plls(5805000);
		assert(!ar_mac_encode(valid_packet,sizeof valid_packet,0,0,control,sizeof control,NULL,0,&packet_size));
		*reg(0x0a110f90)=packet_size; inject_receive=1; injected=0; ar_rf_poll(); ar_rf_poll();
		assert(search->acquisition_ready && search->dev_state==1);
		/* Complete login, then invalidate with a new peer token on cycle 1. */
		if (cycle==1) {
			login_control[6]=login_control[7]=1; memset(login_control+39,0,4);
			login_control[2]=0; checksum=0;
			for(unsigned i=0;i<sizeof login_control;i++) { checksum=ar_mac_crc8_step(checksum^login_control[i]); }
			login_control[2]=checksum;
			assert(!ar_mac_encode(valid_packet,sizeof valid_packet,1,0,login_control,sizeof login_control,NULL,0,&packet_size));
			*reg(0x0a110f90)=packet_size; injected=0; ar_rf_poll(); ar_rf_poll();
			assert(search->dev_state==2);
			uint8_t changed[6]={0,23,10,2,0,0};
			for(unsigned i=1;i<6;i++) { changed[0]=ar_mac_crc8_step(changed[0]^changed[i]); }
			assert(!ar_mac_encode(valid_packet,sizeof valid_packet,2,0,changed,sizeof changed,NULL,0,&packet_size));
			*reg(0x0a110f90)=packet_size; injected=0; ar_rf_poll(); ar_rf_poll();
			assert(recovery->completed==recovered+3 && recovery->last_reason==2 && recovery->invalidations);
		} else {
			delay_dma=1; injected=0; ar_rf_poll(); /* DMA still in flight. */
			complete_dma_reads=3;
			unsigned cancelled_before=life->cancelled_rx;
			candidates[1]=0; assert(!ar_rf_host_candidates(candidates,sizeof candidates));
			assert(recovery->pending==3 && (*reg(0x0a110800)&4));
			ar_rf_poll();
			assert(recovery->completed==recovered+2 && life->cancelled_rx==cancelled_before+1);
			assert(!complete_dma_reads && recovery->stop_stage==3 && recovery->stop_pending_rx==1);
			delay_dma=0;
			assert(recovery->last_reason==3 && recovery->candidate_removals);
			inject_receive=0;
			/* A removed peer cannot regain admission until explicitly restored. */
			ticks += 11*40000; ar_rf_poll();
			inject_receive=1; injected=0; ar_rf_poll(); ar_rf_poll();
			assert(!search->acquisition_ready && search->admission_rejects);
			assert(!ar_rf_stop_host());
			candidates[1]=1; assert(!ar_rf_host_candidates(candidates,sizeof candidates));
			assert(!ar_rf_host_control(0,0));
		}
	}
	assert(compensation_writes == loaded_writes);
	assert(!memcmp(&ar_rf_power_information()->config,&saved_power,sizeof saved_power));
	assert(ar_rf_power_information()->level[0]==25);
	/* Bounded diagnostics own their field setup independently. Returning to
	 * normal operation must await a fresh preparation, not reuse stale cache. */
	/* Binding must admit a previously unknown peer without a local-position
	 * list, then expose a result only after matching login. */
	uint8_t pairing_request[14] = {1,1}, pairing_result[164];
	uint8_t pair_advert[32] = {32,1,0,0x40,1,0,0,0,9,10,11,12,9};
	uint8_t pair_login[43]; memcpy(pair_login,login_control,sizeof pair_login);
	memcpy(pair_login+8,pair_advert+8,4);
	pair_login[6]=pair_login[7]=0; pair_login[12]=13;
	memset(pair_login+39,0,4);
	unsigned binding_writes=writes;
	assert(ar_rf_host_pairing(pairing_request,13)==-22);
	assert(ar_rf_host_peer(pair_advert+8,3)==-22 && writes==binding_writes);
	pairing_request[0]=0;
	assert(!ar_rf_host_pairing(pairing_request,14) && writes==binding_writes);
	pairing_request[0]=1;
	inject_receive=0;
	assert(!ar_rf_host_pairing(pairing_request,14));
	assert(recovery->pending==4);
	ar_rf_poll();
	assert(!recovery->pending && recovery->last_reason==4);
	assert(!(*reg(0x0a110de0)&255) && !(*reg(0x0a110df0)&255));
	assert(!ar_rf_host_pairing_result(pairing_result) && !pairing_result[0]);
	for (unsigned step=0;step<2;step++) {
		uint8_t *p=step?pair_login:pair_advert;
		size_t size=step?sizeof pair_login:sizeof pair_advert;
		p[2]=0; checksum=0;
		for(unsigned i=0;i<size;i++) { checksum=ar_mac_crc8_step(checksum^p[i]); }
		p[2]=checksum;
		assert(!ar_mac_encode(valid_packet,sizeof valid_packet,step,0,p,size,NULL,0,&packet_size));
		*reg(0x0a110f90)=packet_size; inject_receive=1; injected=0; ar_rf_poll(); ar_rf_poll();
		assert(search->dev_state==step+1);
		assert(!ar_rf_host_pairing_result(pairing_result));
		assert(pairing_result[0]==step);
	}
	assert(!memcmp(pairing_result+1,pair_advert+8,4));
	/* The host clears candidates after success, before disabling pairing
	 * and selecting the peer. That must not tear down the new link. */
	candidates[1]=0;
	assert(!ar_rf_host_candidates(candidates,sizeof candidates) && !recovery->pending);
	pairing_request[0]=0;
	assert(!ar_rf_host_pairing(pairing_request,14));
	assert(!ar_rf_host_pairing_result(pairing_result) && !pairing_result[0]);
	/* SET 3 records the peer only - a host may name
	 * its stored AP at every start, often not the unit linked, and retargeting
	 * the identity to it stopped the link on the bench (ar_rf_host_peer). */
	open8030_linked_tests();
	assert(!ar_rf_host_peer(pair_advert+8,4));
	assert(!ar_rf_stop_host());
	assert(!ar_rf_host_control(0,0));
	assert(!ar_rf_host_status(0,host_status) && !memcmp(host_status+254,pair_advert+8,4));
	/* Restore the fixture's peer/candidates for subsequent failure tests. */
	assert(!ar_rf_host_peer(search_plan+8,4));
	candidates[1]=1;
	inject_receive=0;
	pairing_request[0]=1;
	assert(!ar_rf_host_pairing(pairing_request,14)); ar_rf_poll();
	assert(!search->dev_state);
	pairing_request[0]=0;
	assert(!ar_rf_host_pairing(pairing_request,14) && recovery->pending==4);
	ar_rf_poll();
	assert(!recovery->pending && !ar_rf_host_pairing_result(pairing_result) && !pairing_result[0]);
	/* Normal search restores the zero acquisition identity, rather than
	 * leaving the configured-peer-only hash from cancelled pairing. */
	assert(!(*reg(0x0a110de0)&255) && !(*reg(0x0a110df0)&255));
	inject_receive=1;
	assert(!ar_rf_stop_host());
	assert(!ar_rf_monitor_plan(5770000,100,4,64,connected_plan,sizeof connected_plan));
	assert(!(*reg(0x0a110480)&2) && compensation_writes > loaded_writes);
	ticks += 100*40000; ar_rf_poll();
	assert(!ar_rf_power_information()->config.valid && !ar_rf_power_information()->level[0]);
	assert(!ar_rf_prepare_config(5770000,4,search_plan,sizeof search_plan,1,&saved_power));
	assert(!memcmp(&ar_rf_power_information()->config,&saved_power,sizeof saved_power));
	assert(!ar_rf_host_candidates(candidates,sizeof candidates));
	assert(!(*reg(0x0a110480)&2));
	/* Recovery failure cannot reuse an outstanding DMA slot or resume TX. */
	ticks+=11*40000; ar_rf_poll();
	assert(!ar_mac_encode(valid_packet,sizeof valid_packet,0,0,control,sizeof control,NULL,0,&packet_size));
	*reg(0x0a110f90)=packet_size; injected=0; ar_rf_poll(); ar_rf_poll();
	assert(search->acquisition_ready);
	loaded_writes = compensation_writes;
	prepare_frame();
	assert((*reg(0x0a110480)&2) && compensation_writes > loaded_writes);
	/* A stuck DMA must fail stop within its bound, keep its storage owned and
	 * refuse start/init. A late completion cannot revive the stopped session. */
	delay_dma=1; injected=0; ar_rf_poll();
	candidates[1]=0; assert(!ar_rf_host_candidates(candidates,sizeof candidates));
	ar_rf_poll();
	assert(recovery->failures==1 && (int32_t)recovery->last_error==-110 && ar_rf_lifecycle_information()->state==3);
	assert(recovery->stop_stage==1 && recovery->stop_pending_rx==1);
	before=writes;
	assert(ar_rf_host_control(0,0)==-110 && ar_rf_host_control(2,0)==-110 && writes==before);
	*reg(0xe0801164)|=1; ar_rf_poll(); assert(writes==before);
	assert(ar_rf_probe() == -110); /* Dead OTP does not hang or claim completion. */
	assert(ar_rf_information()->phase == AR_RF_OFF);
	puts("synthetic RF peripheral contracts passed");
	return 0;
}
