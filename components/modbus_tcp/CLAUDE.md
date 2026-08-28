# `modbus_tcp` — design reference

A Modbus TCP transport for ESPHome's stock Modbus client hub. Lets
`growatt_master` (or any `modbus::ModbusClientDevice`) talk to a device over
TCP/502 with no change to its own code.

Status: working prototype, validated against a Growatt Shine WiLAN-X2 dongle.
Not yet upstreamed. Depends on a three-line patch to the core `modbus`
component — see `../modbus/PATCH.md`.

Target: ESP32 / ESP-IDF only. Verified on ESPHome 2026.8.0, ESP-IDF v5.5.5.

---

## Why this exists

ESPHome's `modbus` component is RTU-over-UART only. Every existing TCP client
for ESPHome (creepystefan, Gucioo/GiuseppeP96, and the bridge-based approaches)
works around this by cloning the component under a new name, which means none of
them can host a `modbus::ModbusClientDevice`. Components already written against
the stock hub cannot use them.

This component takes the opposite approach: it *is* a `ModbusClientHub`, so
existing devices bind to it unchanged.

## Architecture

**The transport is welded to the hub by inheritance, not hidden behind an
interface.**

```
Modbus : public uart::UARTDevice, public Component
  └── ModbusClientHub : public Modbus
        └── ModbusTcpClientHub          <- this component
```

`ModbusClientDevice::parent_` is typed `ModbusClientHub *`, so subclassing is the
only way in. `parent_` (the UART) stays null throughout; anything that touches it
must be overridden.

**The key design decision: translate at the boundary, not at the parser.**

The hub hands down an RTU frame and expects RTU frames back in `rx_buffer_`.
Rather than reimplementing the parser for MBAP, we convert at the two seams:

- outbound: strip the CRC, prepend a 7-byte MBAP header
- inbound: strip the header, append unit ID + PDU + a *recomputed* CRC

`parse_modbus_frames()` and everything above it never learn the transport
changed. The queue, the scheduler, the send-wait watchdog, response validation
and the whole `ModbusClientDevice` callback surface remain upstream's.

The recomputed CRC is redundant work on a link that already guarantees
integrity. It buys an unmodified parser, which was the better trade for a first
version — but see *Known limitations* for what it costs.

## The three overrides

| Method | Why |
|---|---|
| `send_frame_()` | Builds MBAP, writes to socket. |
| `receive_bytes_()` | Reads socket, lifts complete MBAP frames, converts to RTU shape. |
| `timeout_()` | Upstream compares `rx_buffer_.size()` against the UART's `rx_full_threshold` — a null dereference with no UART. |

Plus `setup()`, `loop()`, `tx_blocked()` and `dump_config()`, which are already
virtual upstream.

**`timeout_()` returns `true` unconditionally.** Not a shortcut:
`deliver_frame_()` only ever appends whole frames, so a partial response cannot
exist in `rx_buffer_` by construction. There is also nothing to measure — TCP has
no inter-character silence, which is the only thing upstream's version times.

**`setup()` deliberately does not call `Modbus::setup()`.** That derives the RTU
inter-frame delay from the UART baud rate. `frame_delay_ms_`,
`long_rx_buffer_delay_ms_` and `turnaround_delay_ms_` are all set to zero — the
honest values for a link with no shared pair and no silence between characters.

## Connect completion detection

**This is the part that took three attempts. Do not "simplify" it.**

The problem: detecting when a non-blocking `connect()` has actually completed.

- ❌ **`SO_ERROR == 0`** — reports a *pending error*, so it reads back as zero
  while the handshake is still in flight. Cannot distinguish "connected" from
  "not finished yet". Declares success on the first loop pass; the first write
  dies with `ENOTCONN`.
- ❌ **`getpeername()` succeeding** — lwIP stores the remote address on the pcb
  when `connect()` is *called*, before SYN is answered. Returns the peer
  successfully against a host that is switched off.
- ✅ **`select()` with the socket in `writefds`** — a non-blocking connect signals
  completion by making the socket writable. This is the only signal that appears
  *after* the handshake rather than before. Then check `SO_ERROR` for the
  verdict, because a refused connection also arrives as writable.

`socket::Socket` is `BSDSocketImpl` on ESP-IDF, which exposes `get_fd()`.

Both failed approaches shipped and crashed the node before being caught. The
symptom of a null `parent_` dereference is `LoadProhibited` with a small
`excvaddr` (under 0x100) — if that reappears, look for a UART method reachable
from a non-virtual path.

## Transaction IDs and late-response detection

Each request gets an incrementing TID; the response must match. On a
point-to-point link with one client, a reply carrying a TID we are no longer
waiting for has one plausible cause: the peer's own response timeout is longer
than ours, so we gave up and the answer turned up late.

This is logged with the likely cause named in the message, and counted in
`get_late_response_count()`. Expose it as a diagnostic sensor **per hub** — with
several dongles you need to know *which* one lags.

Observed firing in exactly the predicted condition once two devices shared one
connection (see *Field findings*).

## Deliberate non-goals (first version)

- **One transaction in flight**, inherited from the hub, even though MBAP
  transaction IDs permit pipelining. Changing the transport without changing the
  timing model was the whole point; pipelining comes after the numbers justify
  it.
- **No reconnect backoff curve.** `reconnect_interval` is a flat retry and also
  bounds the connect attempt. Note the two add up: after a timeout the next
  attempt is one full interval later, so the real retry period is ~2×.
- **Hostnames are accepted by the schema but resolution is not guaranteed** —
  `socket::set_sockaddr` is what actually parses `host`. Use IPs.

## Known limitations

**Our CRC no longer verifies anything.** On RS485 the CRC covered the whole path
to the inverter. Now it covers only what we constructed. The dongle↔inverter
RS485 hop is validated by the dongle, and if that validation is lax, corruption
passes silently.

One observation consistent with this: reading the model string from slot 0 gave
`PV   _000`, where byte 6 was `0x5F` in the position where every other unit has
`0x36` (`'6'`). Identical layout, one byte different. **Unresolved** — re-read
register 0x7D several times; if the value oscillates it is corruption and this
design needs revisiting, if it is stable it is what the inverter reports.

**A malformed MBAP header drops the connection** rather than resynchronising.
Unlike RTU there is no silence to re-anchor on; the connection is the only thing
that can be resynchronised.

## Field findings — Growatt Shine WiLAN-X2

**The dongle ignores the unit ID.** It echoes it correctly in the MBAP header but
does not route on it. A request addressed to unit 5 (an Eastron SDM630) returned
the *host inverter's* registers — byte-for-byte identical to the unit 11
response, in Growatt integer format (`13.89` = 5001 = 50.01 Hz) rather than the
IEEE754 floats a real SDM630 returns.

It does not return an exception for an unknown address. It returns plausible
data. What caught it was `growatt_meter`'s plausibility check
(`no voltage on any phase`), not anything at the protocol level.

**Consequence: one dongle serves exactly one inverter.** TCP does not give you an
alternative bus, it gives you N point-to-point links. `MULTI_CONF = True` is set
for this reason.

**Latency, single device, ~3 minutes of traffic (n=28):** median 433 ms, mean
475 ms, P90 669 ms, max 1059 ms. Compare 100–250 ms for the same blocks on
RS485 — the dongle costs roughly 2–3×.

**Latency, two devices on one connection:** 390–1656 ms plus frequent
`Stop waiting for response ... 2003ms`. The single in-flight slot becomes the
bottleneck immediately, and the starved device eventually drops offline.

**Still open:** P99 under normal cloud traffic over a full day. Three minutes is
not a sample.

## Configuration

```yaml
modbus_tcp:
  - id: bus_growatt03
    host: 192.168.20.195      # IP, not hostname
    port: 502
    send_wait_time: 2000ms    # the only response deadline that exists over TCP
    reconnect_interval: 5s    # also bounds the connect attempt
```

`ModbusTcpClientHub` is declared as a subclass of `modbus.ModbusClient`, so
`cv.use_id(modbus.ModbusClient)` accepts it. Consumer components need no Python
changes.

## Upstream plan

The seam is three `virtual` keywords, but a PR that only adds them, with no
in-tree consumer, is hard to defend at review — it asks a maintainer to freeze an
internal API for something they cannot see, and nothing in-tree would keep it
from being refactored away.

The `modbus` component is currently mid-refactor by a single contributor
(exciton) with an explicit ordered PR chain and "merge after N" annotations.
Anything touching the same files gets slotted into that chain rather than jumped
ahead of it. In particular #12421 (interrupt-driven receive queue) rewrites the
very path we make virtual — building against the current one and rebasing later
would mean rewriting, not rebasing.

Sequence:

1. Ask exciton whether a transport seam is in scope for the series, and where it
   would sit relative to #12421. **Before writing any upstream code.**
2. Keep this vendored prototype running meanwhile; it is what makes the argument
   concrete.
3. Propose a behaviour-preserving refactor separating transport from framing,
   demonstrated green on the existing `tests/components/modbus/` suite.
4. TCP transport as a second PR.

Bar for a new component in this area, from the review of the closed
`modbus_tcp_slave` PRs: tests under `tests/components/`, `CODEOWNERS`, a separate
docs PR, `cv.only_on_esp32` / framework guards in the schema, and dependencies
via `add_idf_component`.

**Note the framing for that conversation:** the argument is not "some methods
should be virtual". It is that transport and framing are interleaved in places
that are not visible until you try — `timeout_()` was found by crashing a live
node twice, not by reading the code.
