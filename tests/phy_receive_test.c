/* SPDX-License-Identifier: MIT */
#include "ar_phy_receive.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static struct ar_phy_control control;
static struct ar_phy_receive receiver;
static uint8_t packet[7], admitted[150];
static unsigned depth, timer_active, starts, stops, calls, preparations, fields, admissions;
static uint32_t delay;
static int read_status;
static void lock(enum ar_phy_receive_lock which, void *context)
{
	assert(context == &control && depth == (which == AR_PHY_RECEIVE_CONTROL ? 0u : 1u)); depth++;
}
static void unlock(enum ar_phy_receive_lock which, void *context)
{
	assert(context == &control && depth == (which == AR_PHY_RECEIVE_CONTROL ? 1u : 2u)); depth--;
}
static int read(uint8_t output[7], void *context)
{
	assert(context == &control && depth == 1); memcpy(output,packet,7); return read_status;
}
static void prepare(void *context)
{
	assert(context == &control && depth == 1 && !receiver.retransmit_ready); preparations++;
}
static void field(uint32_t address, uint32_t value, void *context)
{
	assert(context == &control && depth == 2 && address == 0x0cf41501 && value == 1);
	assert(!(receiver.dev_flags & 0x80)); fields++;
}
static void dispatch(unsigned command, uint8_t *data, size_t length, void *context)
{
	/* Command 4 (3 bytes) in the stock cases; command 9 empty or 4 bytes. */
	assert(context == &control && depth == 1 && data == control.rx+3 &&
	       ((command == 4 && length == 3) || (command == 9 && (length == 0 || length == 4)))); calls++;
}
static void stop(void *context) { assert(context == &control && depth == 1); timer_active = 0; stops++; }
static void start(uint32_t ms, void *context)
{
	assert(context == &control && depth == 1); timer_active = 1; starts++; delay = ms;
}
static int admit(const uint8_t record[150], void *context)
{
	assert(context == &control && depth == 1); memcpy(admitted,record,150); admissions++;
	receiver.dev_state = 2; return -6;
}
static uint32_t now(void *context) { assert(context == &control && depth == 1); return 100; }
static const struct ar_phy_receive_ops ops = {lock,unlock,read,prepare,field,dispatch,stop,start,admit,now};
static void control_lock(void *context) { lock(AR_PHY_RECEIVE_CONTROL,context); }
static void control_unlock(void *context) { unlock(AR_PHY_RECEIVE_CONTROL,context); }
static void unused_field(uint32_t a, uint32_t v, void *context) { (void)a; (void)v; (void)context; assert(0); }
static void unused_write(const uint8_t data[7], void *context) { (void)data; (void)context; assert(0); }
static void unused_idle(uint8_t data[7], void *context) { (void)data; (void)context; assert(0); }
static void control_dispatch(unsigned outgoing, unsigned command, uint8_t *data, size_t length, void *context)
{
	assert(!outgoing); dispatch(command,data,length,context);
}
static const struct ar_phy_control_ops control_ops = {control_lock,control_unlock,unused_field,unused_write,control_dispatch,unused_idle};
static void seal(void)
{
	unsigned high = packet[0] & 0xf0;
	packet[0] = ar_phy_control_checksum(packet);
	if (control.mode == 3) { packet[0] = high | (packet[0] & 15); }
}
static int process(void) { int result = ar_phy_receive_process(&control,&receiver,&ops,&control); assert(!depth); return result; }
static void reset(void)
{
	memset(&control,0,sizeof control); memset(&receiver,0,sizeof receiver); memset(packet,0,7);
	control.role = 1; control.callbacks = 0x17e; control.frame_us = 10000;
	memcpy(receiver.mac,(uint8_t[]){0x66,1,2,3},4); receiver.position = 3; receiver.slot = 3; receiver.dev_state = 2;
	depth = timer_active = starts = stops = calls = preparations = fields = admissions = 0; read_status = 0;
}
int main(void)
{
	reset(); packet[1] = 0x44; packet[2] = 6; seal();
	assert(!process() && timer_active && starts == 1 && delay == 34 && !calls);
	uint8_t saved[7]; memcpy(saved,control.rx,7); packet[0] ^= 1;
	assert(process() == -74 && !memcmp(saved,control.rx,7) && timer_active);
	packet[1] = 0x17; packet[2] = 8; packet[3] = 5; seal();
	assert(!process() && control.time_valid && control.time_value == 0x10000005u-100);
	assert(timer_active && !stops && !ar_phy_control_expire(&control,&control_ops,&control) && !calls);
	packet[1] = 0x10; packet[2] = 14; seal();
	assert(!process() && !ar_phy_control_expire(&control,&control_ops,&control) && !calls && timer_active);
	packet[1] = 0xf4; packet[2] = 7; seal();
	assert(!process() && calls == 1 && !stops && timer_active);
	packet[1] = 0x14; packet[2] = 6; seal();
	assert(!process() && calls == 2 && stops == 1 && !timer_active);
	memcpy(saved,control.rx,7); packet[1] = 0x18; packet[2] = 8; packet[6] = 12; seal();
	assert(process() == -22 && !memcmp(saved,control.rx,7));
	read_status = -6; assert(process() == -6 && !memcmp(saved,control.rx,7));
	/* Command 9: stock's empty one, and Open8030's 4-byte TX power
	 * announcement (ar_pwr_auto.h), both dispatched; any other length is not. */
	read_status = 0; control.callbacks |= 1u << 9;   /* rf_device's PHY_CALLBACKS */
	{
		const unsigned before = calls;
		memset(packet,0,7); packet[1] = 0x19; packet[2] = 0; seal();
		assert(!process() && calls == before + 1);
		memset(packet,0,7); packet[1] = 0x19; packet[2] = 4 << 1; packet[3] = 0xad; packet[4] = 1; seal();
		assert(!process() && calls == before + 2 && control.rx[3] == 0xad);
		memset(packet,0,7); packet[1] = 0x19; packet[2] = 2 << 1; seal();
		assert(process() == -22 && calls == before + 2);
	}
	reset(); control.mode = 3; receiver.dev_flags = 0x1280;
	packet[0] = 0xf0; packet[1] = 4; packet[2] = 6; seal();
	assert(!process() && preparations == 1 && fields == 1 && receiver.dev_flags == 0x1200);
	assert(receiver.retransmit_ready && receiver.retransmit_phase == 3);
	receiver.retransmit_flags = 0xff;
	assert(!process() && receiver.retransmit_flags == 0xfb && preparations == 1 && fields == 1);
	receiver.retransmit_flags = 0xff; packet[0] &= 15; seal();
	assert(!process() && receiver.retransmit_flags == 0xff && receiver.retransmit_phase == 3);
	reset(); receiver.dev_state = 0; packet[1] = 0x10; packet[2] = 7;
	packet[3] = 0x66; packet[4] = 4; packet[5] = 3; seal();
	assert(!process() && admissions == 1 && receiver.dev_state == 2 && !starts && !calls);
	uint8_t expected[150] = {0}; expected[0] = 0x20; expected[3] = expected[4] = 3;
	expected[7] = 17; memcpy(expected+8,receiver.mac,4); expected[12] = expected[14] = expected[16] = expected[18] = expected[32] = 1;
	assert(!memcmp(admitted,expected,150));
	assert(ar_phy_receive_process(NULL,&receiver,&ops,&control) == -22 && !depth);
	puts("PHY receive validation, cache replacement and synchronization tests passed");
	return 0;
}
