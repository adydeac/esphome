# Modbus API migration — what changed

Clean break: no compatibility shims, no `#if ESPHOME_VERSION_CODE`.

## Base class

All three device classes: `modbus::ModbusDevice` → `modbus::ModbusClientDevice`.

## Callbacks

| Old | New |
|---|---|
| `on_modbus_data()` + `on_modbus_error()` | `on_read_registers(EntityType, start_address, span<const uint16_t>, ResponseStatus)` |
| (write echo handled inside `on_modbus_data`) | `on_write_single_register()` / `on_write_multiple_registers()` |
| (short reply caught by size guards) | `on_custom_response()` |

`on_read_registers()` is the shared base of the holding and input variants. The
identification sequences interleave both tables, so one handler keeps the
existing single-entry state machine intact.

## Requests

| Old | New |
|---|---|
| `send(CMD_READ_INPUT, b, n)` | `read_input_registers(b, n)` |
| `send(CMD_READ_HOLDING, b, n)` | `read_holding_registers(b, n)` |
| `send(r.function, ...)` (dump) | `read_entities(EntityType, ...)` |
| `send(CMD_WRITE_SINGLE, a, 1, 2, payload)` | `write_single_register(a, v)` |
| `send(CMD_WRITE_MULTI, a, n, n*2, payload)` | `write_multiple_registers(a, span)` |

## Parsing

Registers now arrive in host byte order as `span<const uint16_t>`. Because every
call site was already register indexed, only the helpers changed shape:

- `reg16(d, r)` → `d[r]`
- `reg32(d, r)` → unchanged (built on `reg16`)
- `fp32(d, r)` → `(d[r] << 16) | d[r+1]`
- `reg16m(d, r)` → `d[r]`
- `ascii_from()` → two characters per word, high byte first
- `data.size() >= CNT * 2` → `data.size() >= CNT`
- `handle_dump_`: `regs = data.size() / 2` → `data.size()`

## Refused requests

The typed helpers return `bool`. `false` means refused at the door with no
callback ever following, so `waiting_` must not be set. Both device classes route
every send through `queued_(bool)`, which mirrors the existing timeout recovery
case for case, including its ordering — a write is abandoned only after the retry
budget, exactly as a timed-out one is.

Note this is close to unreachable: `try_send_()` already gates on
`ready_for_immediate_send()`, which requires an empty queue, so neither
"queue full" nor "over-cap duplicate" can normally apply. Handled anyway,
because a silent stall is the failure mode hardest to read from a log.

## Behavioural notes

1. **The write-fallthrough bug is now structurally impossible.** A rejected write
   can no longer reach the identification state machine, because write responses
   have their own callbacks. The `if (writing_)` ordering that used to be load
   bearing is gone.

2. **One deliberate behaviour change, needs a decision.** The old
   `on_modbus_error()` did not check `probing_`, so an exception during an
   offline probe fell through to `advance_(false)` — advancing the identification
   state machine and setting `ident_incomplete_` on the strength of a probe,
   while leaving `probing_` set. `on_modbus_data()` did check it. The unified
   handler checks `probing_` first for both outcomes, matching the data path.
   Reachable in practice: the probe reads input register 0, and per CLAUDE.md
   register support varies by model and is not even consistent within one unit.
   Say the word and I will restore the old asymmetry.

3. `finish_()` in the address tool now also calls `clear_tx_queue_for_device()`.
   `address_ = 0` was kept alongside it, as agreed.

4. `last_update_` on an unsolicited frame: the old `on_modbus_data()` refreshed
   it even when `!waiting_`, `on_modbus_error()` did not. The unified path
   follows the data behaviour. With per-request correlation in the new hub an
   unsolicited frame is no longer delivered as a response at all, so this is
   dead code either way.

## Python side

Almost nothing to do: `GrowattInverter` and `GrowattMeter` were already declared
against `modbus.ModbusClientDevice`, and all three registrations already went
through `register_modbus_client_device()`. The C++ side was the half that
lagged, which is the disagreement CLAUDE.md describes — in the other direction
from how it is worded there.

One line changed: `GrowattAddressTool` was still declared with
`modbus.ModbusDevice` (line 164). Now `ModbusClientDevice`, matching the header.

### Bus id validation tightened

`modbus/__init__.py` declares `Modbus` as the base and `ModbusClient`
(`ModbusClientHub`) / `ModbusServer` (`ModbusServerHub`) as the two roles, with
`role: client` declaring its id as `ModbusClient`. So `cv.use_id(modbus.Modbus)`
accepted a `role: server` hub, and the mistake surfaced in C++ rather than in
validation.

All five bus references now use `cv.use_id(modbus.ModbusClient)`:
`modbus_id` on the inverter and meter schemas, and `modbus_id`,
`inverters_modbus_id`, `meters_modbus_id` on the hub schema.

Not adopted: `modbus.final_validate_modbus_device(name, role="client")`, the
component's standard helper. It only inspects the single `CONF_MODBUS_ID` key,
so it cannot see the split-bus keys. `use_id` on the client type covers all
three uniformly and rejects at the same point.

## Still to do
- **Hand arbitration to the hub (separate task).** `on_no_response()`,
  `on_sent()` and `on_not_sent()` are now available. The component's own
  watchdog, `ready_for_immediate_send()` gating and `BUS_YIELD_MS` duplicate
  what the hub's queue does natively. Deferred so this commit stays verifiable:
  the API moved, the timing did not.
