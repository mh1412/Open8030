# Receive-only spectator mode

LST1 supports fixed-channel reception of an already-connected AP's payload
without DEV admission or uplink traffic. LST2 adds receive-only following of subsequent AP channel/width controls.
Automatic initial session acquisition is not implemented.
Normal admission must not be bypassed by forcing connected state: that path
also enables sync acknowledgements and periodic transmit reports.

## Bounded listener diagnostic

RF monitor request `0x0c5a0016`, operation **77**, accepts the existing 16-byte
monitor prefix (little-endian frequency in kHz, duration in milliseconds,
identity hash, operation), followed by this exact 12-byte payload:

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 4 | ASCII `LST1` |
| 4 | 1 | RX8 bandwidth code, 0–5 |
| 5 | 1 | RX8 selector, 4 or 5 |
| 6 | 1 | RX8 interleave selector, 5 or 6 |
| 7 | 1 | Physical buffer-0 mode, 0 or 1 |
| 8 | 4 | RX duration in hardware counts, little-endian, 1–65535 |

Operation 77 duration is limited to 1–10,000 ms. Operation **78** uses the
same payload with duration zero for a continuous fixed-channel listener.
The normal host stop request (`0x0b000001`) drains/stops either listener
without permitting normal start/association requests. The radio must already be calibrated,
pins configured, and stopped in the PSRAM-ready phase. AP contexts are rejected.
This does not initialize or replace the radio firmware.

The diagnostic reuses passive operation 53 internally, retains hardware TX hold,
sets the supplied identity in both receive filters, and fixes the receive
context for the whole trial. Search retries are suspended because they clear
the buffer map. It does not run normal admission, sync-command processing,
automatic bandwidth changes, or control/telemetry delivery. Valid video payloads
(MAC port 4, host socket 3) can be captured through the normal host socket API.
Operation 77 stops automatically; operation 78 runs until stopped.

Read the ordinary RX diagnostic counters afterward. Compare valid MAC packets,
DMA completions, flagged/busy/other drops, and payload descriptors. Header
reception alone is not payload success. TX counters and hold readback are useful
checks but cannot replace independent RF emission measurements.

For a controlled comparison, hold the pilot's bandwidth fixed, use the AP's
current identity hash, and vary one receive setting at a time. The historical
20 MHz profile uses bandwidth 4, selector 5, interleave 6 and duration `0x208a`;
these are experiment inputs, not a universally correct spectator profile.

Kestrel's native receiver UI integrates the continuous listener. Initial tuning is manual; Kestrel Air periodic recovery pictures support late
joining. An 8-bit identity hash is not
authentication or sufficient proof that a payload belongs to the desired AP.

## Host capability and lifecycle API

Read-only request `0x0c5a0037`, empty input, returns 28 bytes:
`LST1`, little-endian API revision (2), bandwidth-code bitmap (bit 5: 40 MHz),
then four little-endian counters: DEV state, TX packets, preencode operations,
and sync ACKs submitted. Nonempty input or a request error status is rejected.
Clients must check this capability before configuring the listener. Monitor
77/78 retain their existing payload format for diagnostic compatibility.

STOP remains valid for a listener. START never enables association. After STOP,
an explicit INIT restores the loader's saved pilot configuration when one is
available; the following START enters the ordinary paired path. INIT while
actively listening is rejected. Neither a timeout nor an idle rearm performs
this pilot transition automatically.

## Receive-only following (LST2)

Use the same 12-byte monitor payload with magic `LST2`. The loader must have
supplied a DEV4 channel catalog containing the initial frequency. That catalog
must match the AP's channel index ordering. No admission is synthesized.

Only checksum-valid user-8 PHY command 4 (broadcast RX direction, valid catalog
index) and command 5 (type 0, broadcast slot 8, widths 5/10/20/40 MHz) are used.
Their countdown schedules an RX-only retune, deferred over pending receive DMA.
The handler never calls normal sync/admission callbacks, never alters TX PLL or
identity, and preserves TX hold. LST1 keeps its fixed-channel behavior.

Read-only request `0x0c5a0038` returns eight LE32 words: following active,
current frequency kHz, bandwidth code, completed hops, completed width changes,
rejected controls, pending command, and channel count. Empty input is required.
Kestrel uses this state for live UI reporting and rearming the last followed
frequency/width after a receive stall; the original manual settings stay saved.

This is following, not discovery. Missing a hop announcement, a different
channel catalog, or an unannounced AP restart can require manual retuning.
Transitions may drop pictures; seamless video is not guaranteed. TX counter
checks do not replace an independent RF emission measurement.
