# growatt_master

ESPHome external components for a fleet of Growatt inverters and Eastron
SDM630 meters sharing one or more Modbus buses: `growatt_master` (the hub and
the zero-export control loop), `growatt_inverter`, `growatt_meter`, and a
vendored `modbus_tcp` transport for WiLAN-X2 dongles.

Design notes, rationale and the register conventions live in
`components/growatt_master/CLAUDE.md`. This file is the field reference: which
of an inverter's registers and entities actually mean anything on which model.

## Inverter families

A slot is identified at boot and after every recovery. Three shapes come out of
it, and almost every difference below follows from which one a slot is:

| Family | `storage_family` | Models | Detected by |
| --- | --- | --- | --- |
| Grid tie | `STORAGE_NONE` | MID TL3-X, MOD TL3-X, MIN TL-X | holding 1000 answers empty and input 3118 has no BDC |
| SPH | `STORAGE_SPH` | SPH TL3 BH-UP and the rest of the SPH line | the holding 1000 block answers with a work mode |
| TL-XH | `STORAGE_TLXH` | MIN TL-XH, MID TL3-XH | input 3118 reports a connected BDC |

The two storage families are mutually exclusive: a unit that answers the 1000
block has no 3000 block and the other way round. This is why the component
carries two register maps rather than one with holes in it.

Nothing in the component keys off a model name - there is no model name over
Modbus worth trusting - so the tables below are per family and a model matters
only through the family it lands in. A MID TL3-X and a MOD TL3-X are the same
device to every address here: three phase, no storage blocks, the whole 1000 and
3000 range skipped. A MID TL3-XH has not been seen on this fleet; the detection
at 3118 will place it in the TL-XH family on its own, but see the EPS note below
for the one place where that is not enough.

## Polling blocks

| Block | Grid tie (MID/MOD TL3-X, MIN TL-X) | SPH | TL-XH (MIN, MID TL3-XH) |
| --- | --- | --- | --- |
| Fast main, input 0..56 | yes | yes | yes |
| Fast status, input 101..105 | yes | yes | yes |
| Slow main, input 57..124 | yes | yes | yes |
| Fast battery | – | input 1009..1014 | input 3167..3181 |
| Fast UPS/EPS | – | input 1067..1081 | – (see below) |
| Slow storage | – | input 1000..1096 | input 3125..3231 |
| Settings, holding 1070..1108 | – | yes | read, but see below |

The EPS block at input 3145..3161 exists throughout the TL-XH family, and
`has_ups_block()` tests the family rather than the terminal: it returns true
only for `STORAGE_SPH`, so the block is skipped on every TL-XH. That is right
for a MIN 6000TL-XH, which has no EPS terminal and would only ever read zeros
there, and wrong for a MID TL3-XH, which has one - its EPS output would simply
never be published. Closing that means testing for the terminal rather than
assuming it from the family, which no register read has been shown to answer
yet.

## Writable settings

Everything in the first holding group is family independent - it applies to a
MID, a MOD and a MIN alike; everything in the 1070 block is SPH.

| Entity | Register | Grid tie | SPH | TL-XH |
| --- | --- | --- | --- | --- |
| `active_power_rate` | holding 3 | yes | yes | yes |
| `pv_start_voltage` | holding 17 | yes | yes | yes |
| `start_time` | holding 18 | yes | yes | yes |
| `restart_delay` | holding 19 | yes | yes | **rejected** (exception 1) |
| `grid_v_low` / `grid_v_high` | holding 52 / 53 | yes | yes | yes |
| `grid_f_low` / `grid_f_high` | holding 54 / 55 | yes | yes | yes |
| `export_limit_rate` | holding 123 | yes | yes | yes |
| `grid_first_discharge_rate` | holding 1070 / 3036 | – | yes | yes |
| `grid_first_stop_soc` | holding 1071 / 3037 | – | yes | yes |
| `battery_first_charge_rate` | holding 1090 / 3047 | – | yes | yes |
| `battery_first_stop_soc` | holding 1091 / 3048 | – | yes | yes |
| `ac_charge` | holding 1092 / 3049 | – | yes | yes |
| Grid-first time windows | holding 1080..1088 / 3038, 3040, 3042 | – | yes | yes |
| Battery-first time windows | holding 1100..1108 / 3044, 3050, 3052 | – | yes | yes |
| Load-first time windows | holding 1110..1118 / 3054, 3056, 3058 | – | yes | yes |

Identification used to read the 1070 block on any slot with storage, including a
TL-XH, which answers it with zeros rather than an exception, so the entities
existed, read back as zero and accepted writes that went nowhere. They now read
and write the family's own block. The rates, stop SOCs and AC charge are plain
values at a different address and are treated exactly as the SPH ones are - the
same trust, since the same doubt applies to both.

The windows are written a pair at a time, because the nine are not contiguous
and there is no block to send in one frame the way the SPH blocks are sent.
Applying one mode is three writes of two registers, each a single 0x10: start
and stop have to change together, or the window spends a frame as a new start
against an old stop and the inverter acts on it.

Applying a mode is also the one moment the period convention is allowed to
correct the register. The flags share the word with the start time, so a write
composes them rather than preserving them, and the priority it writes is the one
the period stands for. Windows nobody applies keep whatever ShinePhone put in
them.

Whether the addresses are right at all is answered by looking, not by refusing:
after identification the read back values should match what ShinePhone shows for
the same unit. If they do, the block is mapped correctly and writing it is as
safe as writing the SPH equivalent.

### The TL-XH storage settings block

From the protocol document. The block is read at identification and published;
none of it is written yet.

| Register | Meaning | SPH equivalent |
| --- | --- | --- |
| holding 3018 | `bWorkMode` - 0 default, 1 system retrofit, 2 multi-parallel, 3 retrofit simplified | none |
| holding 3036 | grid-first discharge power rate | 1070 |
| holding 3037 | grid-first stop SOC | 1071 |
| holding 3038..3045 | time windows 1..4, two registers each | 1080..1088 |
| holding 3046 | unknown |  |
| holding 3050..3059 | time windows 5..9, two registers each | 1100..1108 |
| holding 3047 | battery-first charge power rate | 1090 |
| holding 3048 | battery-first stop SOC | 1091 |
| holding 3049 | AC charge enable | 1092 |
| holding 3070 | battery type - 0 lithium, 1 lead acid, 2 other | 1048 |
| input 3144 | priority: load first, battery first, grid first | input 1000 work mode |

#### Time window encoding, and why it is not the SPH shape

A window is two registers rather than the SPH's three, because the flags ride in
the start word:

| Bits | Start register (3038, 3040) | Stop register (3039, 3041) |
| --- | --- | --- |
| 0..7 | minute | minute |
| 8..12 | hour | hour |
| 13..14 | priority: 0 load, 1 battery, 2 grid | reserved |
| 15 | 0 prohibited, 1 enabled | reserved |

This is a different model, not a different address. On an SPH the priority of a
period is decided by which block it sits in - 1080 is grid first, 1100 is
battery first - and a unit has three periods of each. A TL-XH has nine windows
in one list, four at 3038..3045 and five at 3050..3059 with the battery-first
parameters wedged between them, and each entry names its own priority. What
stays per mode is the rest: 3036 and 3037 are the grid-first rate and stop SOC,
3047 to 3049 the battery-first rate, stop SOC and AC charge, and they apply to
whichever window is in force.

The two shapes meet at three periods per mode, which is what the SPH has of each
and what fits three times into the TL-XH's nine:

| Periods | Mode | SPH | TL-XH |
| --- | --- | --- | --- |
| 1..3 | grid first | 1080, 1083, 1086 | 3038, 3040, 3042 |
| 4..6 | battery first | 1100, 1103, 1106 | 3044, 3050, 3052 |
| 7..9 | load first | 1110, 1113, 1116 | 3054, 3056, 3058 |

On the TL-XH side that split is a convention this component imposes, not one the
firmware enforces: any of the nine windows may carry any priority, and the bits
are what decide. The convention buys one entity set for both families at the
cost of nine grid-first windows being unreachable, which no installation here
wants. What it does not buy is agreement with whatever ShinePhone last wrote -
see the mismatch note below.

Two consequences for the write path:

The hour occupies bits 8..12, so `start >> 8` - which is what the SPH parser
does - reads the flags as part of the hour and returns 135 for an enabled
07:35 grid-first window. The mask is `(v >> 8) & 0x1F`, and applying it on both
families costs nothing because the SPH leaves those bits clear.

Enable and priority live in the same word as the start time, so any write that
does not carry them forward silently turns the window off or moves it to another
priority. The component holds the window state already, which makes composing
the whole word the natural thing to do, but the pair still wants a single 0x10
write rather than two 0x06s: start and stop change together or the window exists
briefly as a combination nobody asked for.

And the priority bits read back may disagree with the slot the window sits in,
because ShinePhone can put any priority in any window. Rewriting them at
identification would mean overwriting a configuration somebody made deliberately
with a convention they never agreed to, which is the same mistake as a switch
restoring its state into a live register. The reading here is: publish what the
register says, warn once when an enabled window's priority does not match its
slot, and correct it only when that window is next written.

#### One register to read and not write

`bWorkMode` at 3018 is not the priority and is not a control lever: it selects
what the inverter is in the installation - standalone, retrofit, one of several
in parallel. Reading it is diagnostics; writing it reconfigures the unit's role.
Nothing reads or writes it today.

Battery type is the exception in that group: it is the same setting the SPH
keeps at 1048, it is already a select entity, and it is now resolved per family
rather than written to the SPH address on every model. The settings read runs to
3070 to cover it.

"Rejected" means the inverter answered the write with Modbus exception 1. The
component records the address and stops writing it until the next
identification, which is why `slot N: register 19 rejected the write` appears
once per slot per identification rather than on every cycle.

## Battery and storage entities

A grid tie unit of any size - MID, MOD or MIN TL-X - publishes none of these,
because the blocks they come from are never read on it.

| Entity | Grid tie | SPH | TL-XH |
| --- | --- | --- | --- |
| `battery_voltage`, `battery_soc` | – | input 1013, 1014 | input 3169 (0.01 V), 3171 |
| `bat_charge_power`, `bat_discharge_power` | – | input 1009, 1011 | input 3178, 3180 |
| `battery_temperature` | – | input 1040 (slow) | input 3176 (fast) |
| `fault_word` | – | input 1001 | input 3167 |
| `bms_soc`, `bms_voltage`, `bms_current`, `bms_temperature` | – | input 1086..1089 | input 3215..3218 |
| `battery_cycles`, `battery_health` | – | input 1095, 1096 | input 3221, 3222 |
| `charge_energy_*`, `discharge_energy_*` | – | input 1052..1059 | input 3125..3132 |
| `battery_capacity` | – | input 1090 (see below) | – |
| `system_work_mode` | – | input 1000 | – |
| `power_to_user`, `power_to_grid`, `local_load_power` | – | input 1021, 1029, 1037 | – |
| `e_to_user_*`, `e_to_grid_*`, `e_load_*` | – | input 1044..1051, 1060..1063 | – |
| UPS voltage/current/power per phase, load, PF | – | input 1067..1081 | – |
| `ups_total_power`, `ups_load_avg`, `ups_max_power` | – | derived | – |
| `battery_modules` | – | derived | derived |

## Derived values

Two entities are computed rather than read, and one register is suspect.

`battery_modules` is `round(battery_voltage / module_voltage)`, with
`module_voltage` configurable per slot because it depends on the battery, not on
the inverter. Default 53.75 V, which is a Growatt ARK 2.5H module. A pack at
352.9 V gives 6.57, so 7 modules.

`ups_max_power` is the largest load the pack can hold for `discharge_hours`,
expressed as a percentage of the inverter rating:
`modules * module_capacity / discharge_hours / normal_power`. It is only
published on SPH, because it is the UPS output it describes.

`battery_capacity` is **not** computed and should not be read as installed
energy. It is input register 1090 with a kWh unit attached, and 1090 sits
directly after the BMS group (SOC, voltage, current, temperature at 1086..1089)
where Growatt's map continues with current and gauge figures, not energy. On the
development fleet it reads 25.0 for a pack of 7 ARK 2.5H modules, which is 17.5
kWh installed - a value that reads far more like amperes than kilowatt hours.
Treat it as an unconfirmed raw register until a dump of 1086..1096 has been
compared against what the BMS reports.

## Unmapped registers on TL-XH

The slow storage block is 107 registers wide and twenty of them are used. The
rest arrives every cycle and is discarded, so anything identified in this range
costs nothing to publish - it is already on the wire.

| Range | State | Believed content |
| --- | --- | --- |
| input 3125–3132 | mapped | charge and discharge energy, today and total |
| input 3133–3144 | read, unused | AC charge energy and the remaining counters; see 3144 below |
| input 3145–3161 | read, unused | EPS output (dead on a MIN, real on a MID TL3-XH) |
| input 3162–3166 | read, unused | unknown |
| input 3167–3171 | mapped | fault, warning, pack voltage, current, SOC |
| input 3172–3175 | read, unused | probably bus voltages and a second temperature |
| input 3176 | mapped | pack temperature |
| input 3177 | read, unused | unknown |
| input 3178–3181 | mapped | charge and discharge power |
| input 3182–3195 | read, unused | unknown |
| input 3196–3214 | read, unused | BMS group: cell extremes, and any current or power limit |
| input 3215–3222 | mapped | BMS SOC, voltage, current, temperature, cycles, SOH |
| input 3223–3231 | read, unused | tail of the BMS group |
| input 3232–3249 | not polled | the block may continue; the dump covers it, the poll does not |

Input 3144 is documented as the priority - load first, battery first, grid
first - and sits inside the block already read every slow cycle, at offset 19,
so publishing it would cost no bus time. It is deliberately left alone until
somebody has watched it: whether it reports the priority in force right now or
a configured default is the difference between a free acceptance test for every
write into the settings block and a number that means nothing.

What would be worth having, in the order it would change behaviour:

1. **A BMS current or power limit.** For a storage unit the binding constraint
   is usually the pack, not the inverter, and a stated limit would give
   `update_capability_()` a real figure instead of extrapolating from output
   that is not rate limited in the first place.
2. **The settings block at holding 3036..3049 and the windows around it.**
   Without them there is no charge control on this family at all, and charge
   rate is the largest unused lever on a site that imports while its batteries
   fill from PV.
3. **A BDC derate reason.** A battery that will not discharge currently looks
   identical to a unit short of sun, and the two want opposite decisions.
4. **AC charge energy**, for the energy balance rather than for control.
