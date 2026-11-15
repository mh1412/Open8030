/* SPDX-License-Identifier: MIT */
#include "ar_phy_receive.h"
#include "ar_ap_time.h"
#include <string.h>

static void dispatch(struct ar_phy_control *control,
	const struct ar_phy_receive_ops *ops, void *context)
{
	unsigned command = control->rx[1] & 15;
	if (command < 10 && (control->callbacks & (1u << command))) {
		ops->dispatch(command,control->rx+3,(control->rx[2] >> 1) & 7,context);
	}
}
int ar_phy_receive_process(struct ar_phy_control *control,
	struct ar_phy_receive *receiver, const struct ar_phy_receive_ops *ops, void *context)
{
	if (!control || !receiver || !ops || !ops->lock || !ops->unlock || !ops->read ||
		!ops->prepare || !ops->field || !ops->dispatch || !ops->stop_timer ||
		!ops->start_timer || !ops->admit || !ops->now_ms) { return -22; }
	ops->lock(AR_PHY_RECEIVE_CONTROL,context);
	uint8_t packet[7];
	int result = ops->read(packet,context);
	if (result) { goto done; }
	uint8_t check = ar_phy_control_checksum(packet);
	if ((packet[0] ^ check) & (control->mode == 3 ? 15 : 255)) { result = -74; goto done; }
	static const uint8_t lengths[10] = {255,1,4,2,3,3,2,4,4,0};
	unsigned command = packet[1] & 15, length = (packet[2] >> 1) & 7;
	/* Command 9 carries Open8030's own TX power announcement (4 bytes,
	 * ar_pwr_auto.h) besides stock's empty one. */
	if (command > 9 || (lengths[command] != 255 && length != lengths[command] &&
		!(command == 9 && length == 4)) ||
		(command == 8 && (packet[6] & 12))) { result = -22; goto done; }
	memcpy(control->rx,packet,7);
	if (control->role == 1) {
		if (control->mode == 3) {
			if (!receiver->retransmit_ready) {
				ops->prepare(context);
				receiver->retransmit_ready = 1;
				receiver->retransmit_flags = receiver->retransmit_phase = 0;
			}
			if (receiver->slot < 4) {
				unsigned bit = (packet[0] >> (4+receiver->slot)) & 1;
				ops->lock(AR_PHY_RECEIVE_RETRANSMIT,context);
				if (!(receiver->retransmit_phase & 1)) {
					receiver->retransmit_phase = (receiver->retransmit_phase & 0xfc) | (bit << 1) | 1;
				} else if (((receiver->retransmit_phase >> 1) & 1) == bit) {
					receiver->retransmit_flags &= (uint8_t)~4;
				}
				ops->unlock(AR_PHY_RECEIVE_RETRANSMIT,context);
			}
			unsigned mask = control->rx[2] >> 4;
			ops->lock(AR_PHY_RECEIVE_DEV,context);
			if (receiver->slot < 4) {
				unsigned disabled = (mask >> receiver->slot) & 1;
				if (disabled != !!(receiver->dev_flags & 0x80)) {
					receiver->dev_flags ^= 0x80;
					ops->field(0x0cf41501,!disabled,context);
				}
			}
			ops->unlock(AR_PHY_RECEIVE_DEV,context);
		}
		const uint8_t *rx = control->rx;
		uint8_t peer = receiver->mac[0] ^ receiver->mac[1] ^ receiver->mac[2] ^ receiver->mac[3];
		if (!(rx[1] & 15) && (control->mode == 3 || ((rx[2] >> 1) & 7) == 3) &&
			!receiver->dev_state && (!(rx[6] & 3) || !(rx[6] & 12)) && rx[3] == peer &&
			(rx[4] == receiver->position || rx[5] == receiver->position)) {
			uint8_t record[150] = {0};
			record[0] = 0x20; record[3] = record[4] = 3;
			record[7] = (rx[1] >> 4) | (rx[2] & 1) << 4;
			memcpy(record+8,receiver->mac,4);
			for (unsigned i = 12; i <= 18; i += 2) { record[i] = 1; }
			record[32] = rx[4] != receiver->position;
			(void)ops->admit(record,context);
		}
	}
	command = control->rx[1] & 15;
	unsigned life = (control->rx[1] >> 4) | (control->rx[2] & 1) << 4;
	if (command == 7) {
		struct ar_ap_time time;
		ar_ap_time_receive(&time,control->rx,ops->now_ms(context));
		control->time_value = time.offset_ms; control->time_valid = time.valid;
	} else if (command) {
		if (life == 31) { dispatch(control,ops,context); }
		else if (life == 1) { ops->stop_timer(context); dispatch(control,ops,context); }
		else if (life > 1) { ops->start_timer(((life-1)*control->frame_us)/1000+4,context); }
	}
done:
	ops->unlock(AR_PHY_RECEIVE_CONTROL,context);
	return result;
}
