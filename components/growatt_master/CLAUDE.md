# Working on this project with Claude

This component was written from scratch with Claude AI, over a long series of
build, test and correct cycles against real hardware. This file exists so the
next person, working with Claude or not, starts from what was learned rather
than from the protocol document alone.

Read the "Hard won facts" section before changing any register handling. Most
of it contradicts the official documentation, and all of it was verified on
real inverters.

## What the component is

An ESPHome external component that replaces a Growatt ShineMaster: it manages
several Growatt inverters and Eastron smart meters over Modbus RTU - on one bus
or on two, one per device family - and runs a self-consumption controller that
keeps the property from exporting.

Development hardware, at 9600 baud:

| Unit | Model | DTC | Notes |
|---|---|---|---|
| growatt01 | MOD 40K | 5001 | three phase, 8 trackers, 4 connected |
| growatt02 | SPH 10000TL3 BH-UP | 3601 | three phase, storage, UPS |
| growatt03 | MIN 6000TL-X | 5100 | single phase, 2 trackers |
| grid meter | Eastron SDM630 | — | three phase |

## File layout

```
components/growatt_master/
  __init__.py            all schemas and code generation
  growatt_master.h/.cpp  hub: thresholds, aggregates, controller
  growatt_inverter.h/.cpp  one inverter, its entities
  growatt_meter.h/.cpp     one meter, its entities
```

The hub forward declares the inverter and meter classes and only dereferences
them in its `.cpp`, which is what keeps the includes acyclic.

Three tables have to stay in step between Python and C++, and nothing checks
them at compile time: `SettingField` / `SETTING_ADDR` / `SETTING_SCALE`, the
sensor kind indices (`KIND_*`, `MK_*`), and the enum orders mirrored at the top
of `__init__.py`. Reordering one without the others silently writes the wrong
register.

## Hard won facts about the protocol

These cost real debugging time. None of them are in the manufacturer document,
and several contradict it.

### Absent registers answer with zeros, not exceptions

A Growatt inverter asked for a register it does not implement usually replies
with zeros rather than raising a Modbus exception. **"Did it answer" is never a
capability test.** Every probe must inspect the returned data.

The one observed exception: input registers at 3000 on an SPH do raise
exception 2, while holding registers at 3000 return zeros. Behaviour is not
even consistent within one unit.

### The Storage family deviates from the documented map

Everything below is correct on MOD and MIN units and wrong on SPH:

| Register | Documented | On SPH |
|---|---|---|
| 44 (TP) | trackers and phases | garbage, `0x00C9` |
| 6-7 (normal power) | 0.1 VA | whole VA, so ten times low |
| 125-132 (INV Type) | model string | all zeros |
| 183 (PvStrScan) | string count | 0 |
| 185 (PackNum) | battery modules | 0 even with a working battery |
| 112-115 | warning codes | AC charge energy |
| 40-41, 44-45, 48-49 (Pac1-3) | power per phase | whole output in Pac1, zeros in Pac2 and Pac3 |

When something reads implausibly on one model and correctly on another, assume
the Storage family is the odd one out and check there first.

The nameplate case is handled without a model list: a figure under 500 VA is
multiplied by ten, and anything outside 500 VA to 100 kVA is marked invalid so
the controller falls back to fixed steps instead of scaling by nonsense.

### Capabilities come from measurements, not declarations

Because of the above, the component detects rather than looks up:

- **Phases**: count grid voltages above 10 V at input 38, 42, 46. Two or more
  means three phase.
- **Strings**: count PV inputs above 10 V at 3, 7, 11, ... Keep the *highest
  populated index*, not the number of populated ones, so a gap in the middle
  does not shift the later strings down. Keep the highest count ever seen,
  because strings read zero at night.
- **Storage**: read holding 1000-1060 and OR the first eight words together.
  The battery configuration defaults there are non-zero on storage units.
- **UPS**: bit 0 of holding 1060, only meaningful once storage is confirmed.
- **Battery present**: input 1013 and 1014, live voltage and SOC. `PackNum` is
  useless.

`TP` (holding 44) is filtered for plausibility — low byte 1 or 3, high byte 1
to 8 — and then used only to fill gaps no measurement can: the installed
tracker count always, and the phase count only when the inverter is off grid
and there are no voltages to count.

### Per phase AC power is not per phase on every unit

The SPH 10000TL3 reports its whole AC output in Pac1 and leaves Pac2 and Pac3
at zero, while Iac2 and Iac3 carry real current — so the phases are genuinely
working and it is only the power registers that are not what the map says. A
MOD 40K on the same bus populates all three correctly, so this is not a
property of three phase units and must not be keyed off the phase count.

Detection needs all three of: current present on at least two phases, power
present on exactly the first, and that first figure sitting within 10 % of the
total AC output at input 35-36. The picture has to hold for three consecutive
cycles before it is latched, and it stays latched until the next
identification, so the conclusion survives the night when nothing can be
measured.

The repeat requirement is the real defence, because the obvious guard is
weaker than it looks. The tolerance check is nearly vacuous on its own: if Pac2
and Pac3 both read under a watt then Pac1 equals the total by arithmetic,
whatever the register means. What actually goes wrong is an inverter idling at
first light with reactive current on every phase and active power still
rounding to zero — currents present, one power register populated, Pac1
matching the total. That is the shape being searched for, produced by a
perfectly normal unit.

`phase_power_detect_threshold` (default 100 W, per inverter) is the output
below which the question is simply not asked, because no answer there could
mean anything. Setting it to 0 is allowed and leans entirely on the three
confirmations; it is only safe on a unit that never idles with reactive current
flowing. Lower it for a small inverter whose normal output never reaches 100 W,
raise it if the warning ever appears on a unit that is fine.

When it fires, all three per phase power sensors are published as NaN, not just
the two reading zero: Pac1 is the total, not the first phase's share of it, and
the total already has its own sensor in `grid_active_power`. Voltages and
currents keep being published normally.

The controller is unaffected either way — it works from the meter for per phase
figures and only ever asks an inverter for its total output.

### `real_power_percent` (input 101) reads zero

Observed as zero on all three units even at 100 % setpoint with real output.
Do not build logic on it. Use the setpoint and `grid_active_power` instead.
Its scale is also unsettled: the document says 1 % per count, older YAML
configurations used 0.1.

### Derating mode 7 means "because you told me to"

Register 104 reports why the inverter is limiting itself. Mode 7 is our own
active power command, which is the normal state whenever the setpoint is below
100 %. Treating any non-zero derating as "cannot produce more" deadlocks the
controller after its first reduction.

Mode 1 ("PV") is still ambiguous: it may mean high PV voltage or simply not
enough sun. It is currently treated as blocking. If an inverter reporting
derating 1 refuses to rise on a clear day, that is the explanation.

### Some inverters want line voltage in their protection registers

MOD 40K reports 230 V phase voltages **and** 400 V line voltages, and expects
registers 52 and 53 written in line terms. Magnitude alone is not a safe test:
the detection is "does it populate input 50-52 above 100 V", with a fallback to
"a phase register reading above 300 V can only be a line voltage", and a per
inverter override (`voltage_convention: auto | phase | line`).

Because writing these registers wrong can stop the inverter connecting — or
stop it disconnecting when it should — the automatic adjustment is opt in per
inverter (`auto_protection_limits`, default off).

### Holding 2 controls whether settings survive a power cycle

It decides whether registers 3, 4, 5 and 99 are remembered. Clearing it has two
benefits: the frequent power rate writes never reach the EEPROM, and if the
controller stops running the inverter comes back unrestricted instead of stuck
at whatever limit was last applied. `protect_eeprom: true` clears it during
identification whenever it is found set.

### Holding 0 is a command register, not a state

1 and 0 start and stop the inverter, 3 and 2 do the same for the BDC. That is
why it appears twice in `REGISTER_SWITCHES` with different value pairs.

### Time windows must be written atomically, and must not overlap

Grid First and Battery First periods are only accepted when all three periods
and their enable flags arrive together, as one function 0x10 write. The blocks
are contiguous: 1080-1088 and 1100-1108. Windows may wrap past midnight; an
earlier belief that they may not was tested and disproved — the inverters ship
with 23:00-06:59 configured.

The firmware also rejects an enabled Grid First period that overlaps an enabled
Battery First one. That is checked before writing, with the wrapped windows
split at midnight into up to two same day segments before comparing, so the
user gets a readable message instead of a bare Modbus exception.

### Address ranges differ by vendor, and the standard is stricter than both

Modbus over Serial Line V1.02 reserves 248-255 and defines 1-247 for slaves,
with 0 as broadcast. The reservation is a specification decision, not a
technical limit - the address field is a full byte. Growatt allows up to 254,
Eastron specifies 1-247 across the whole SDM family, and plenty of vendors use
the reserved range anyway, 255 being a common commissioning wildcard.

So the limits are per device type rather than one number: inverters accept
0-254, meters 0-247. Capping meters at 254 would only let someone enter an
address no Eastron can be set to. Worth remembering if a unit ever turns up
answering nowhere: it may have shipped on 255, which nothing here can reach.

### There is no model name over Modbus

Identity is the device type code (holding 43) plus the serial number (holding
23-27). The strings that exist are manufacturer and firmware, not model. Do not
build a model table; there is nothing reliable to key it on.

### Smaller ones worth knowing

- Holding 45 holds the **full year** (2026), not an offset from 2000.
- The UPS frequency register (input 1067) reads 0 while the UPS output is idle.
  That is not a frequency of zero, so it is published as NaN and the sensor
  goes unavailable instead of plotting a drop to zero.
- Status codes 0-3 come from the document; 4-12 were established by watching
  storage units in the field. Codes 7, 8 and 9 all mean "running off grid".
- The document contradicts itself on battery type: the writable register (1048)
  and the input register that reports it do not agree. The select labels follow
  the writable register, because that is the one being written.

## The meter side

All Eastron SDM meters and their clones share one input register map — which is
exactly why they are drop in replacements for each other, and exactly why the
map cannot tell models apart. What actually differs is the phase count, and
that is detectable: count phase voltages above 50 V, two or more means three
phase. The model select is therefore an override, not a lookup; picking one
fixes the phase count, `Auto` hands it back to detection.

Values are IEEE754 floats, big endian, function 0x04.

The reads are split the same way as the inverters': a fast block (input 0-53,
about 126 ms at 9600 baud) carrying the per phase values and total active
power, and a slow block (54-79) with the totals, frequency and energy counters.
Two further blocks cost a transaction each and are **only requested when at
least one of their sensors is declared in YAML**: line to line voltages at
0x00C8 (three phase only) and total energies at 0x0156.

The identification block at holding 0xFC00-0xFC03 is read, logged and shown raw.
No documented mapping was available, so nothing is inferred from it.

## One bus or two

`modbus_id` on its own keeps everything on one bus, exactly as before.
`inverters_modbus_id` and `meters_modbus_id` split them, and either can be given
alone with the other falling back to `modbus_id`. Any single device can override
its type's bus with its own `modbus_id`.

Splitting is worth it because the meter feeds every control cycle while the
inverters are read on a slower schedule, and because a mute device stalls the
whole bus for the duration of its timeouts - which was observed making a silent
meter slow down healthy inverters. Two buses also let a meter and an inverter
share an address without conflict, which simplifies commissioning.

What it costs: a second transceiver and UART, and the loss of a property that
comes free with one bus - a single broken cable stops everything and is obvious,
whereas half a system can run for days unnoticed.

## Commissioning and the address tools

There is a bus level address changer - a button, a "from" and a "to" number, and
a status text sensor - deliberately unrelated to the devices declared in the
configuration. Its whole purpose is the unit that is not in the configuration
yet, or one sitting on an address that clashes.

`GrowattAddressTool` is a `ModbusClientDevice` in its own right, registered at
address 0. Being the broadcast address, 0 never matches an incoming frame, so it
stays inert until it points itself at a real address and returns to 0 when
finished. There is one instance per bus, because a `ModbusClientDevice` belongs
to exactly one bus and a hub spanning two of them cannot be the tool itself.

The two families differ in three ways at once, which is why the profile is
configured rather than assumed: Growatt keeps its address as a plain integer at
holding 30 and takes function 6, while Eastron keeps it as a float32 across
holding 20-21, accepts only function 16, and rejects any request for an odd
number of registers. The probe reads two registers for that last reason.

Before writing, the target address is probed with function 3 at register 0,
about the most universal question there is: a device that implements it answers
with data, one that does not answers with an exception, and either way it has
revealed itself. An exception therefore counts as "occupied", not as failure.
Silence is only accepted as proof the address is free after three attempts,
because one lost frame would fake it.

The write itself is not verified. A missing echo is reported as FAILED because
that is what can be proven, but it is genuinely ambiguous: a unit that adopts
the new address before replying answers from an address nobody was listening
on. Check both before concluding anything.

The tool cannot fix the case that most often creates the need for it. Two units
sharing a factory default address cannot be told apart, so neither can be
addressed individually. The only way out is one device on the bus at a time.

## Persistence

Three structures, one per class, each under its own key: `growatt_hub_settings`
holds the thresholds and the offline action, `growatt_slot_N` the address, phase
and string overrides, wiring and safe rate, and `growatt_meter_N` the address
and model. Nothing else is stored.

Every one of them starts with a version byte and ends with reserved space, and
that is not decoration. ESPHome keys a preference by type, so changing `sizeof`
makes `load()` fail wholesale - not "the new field takes its default" but "every
stored value is lost, including the addresses, which no longer have a YAML
fallback". Adding a field therefore means spending reserved bytes, never
extending the structure. The version byte covers the other case: a field whose
size is unchanged but whose meaning is not. Bump `PREFS_VERSION` then, and every
device falls back to its configured defaults with a warning rather than
misreading old data.

An earlier version of this component bolted new settings on as separate
preference keys precisely to dodge the size problem. That works, but it scatters
one device's state across several keys and the reason is invisible at the call
site. The reserved block is the same idea made explicit.

## Configuration that lives in entities, not YAML

Almost all tuning is a runtime entity now, not a compile time option. The
reasoning is the same one that produced the register dump button: the loop of
guess a value, flash, watch, guess again is slow enough that it discourages
tuning at all, and a controller nobody tunes is worse than one with an awkward
setting. YAML supplies the starting point; flash wins after that.

On the hub that is the whole of `settings:` - thresholds, gains, steps,
intervals, timeouts, margins. The mechanism was already there for the voltage
and SOC thresholds; the tunables just joined the same table, which is why they
persist and publish without any new code. The one wrinkle is that the
thresholds are read out of `values_` where they are used, while the tunables are
held in members the control path reads every pass, so `apply_setting_()` pushes
a change into the member. `update_interval` needs more than that: a
`PollingComponent` will not notice a new interval, so the poller is stopped,
retimed and started again.

Per inverter: min and max power rate, safe rate, both poll intervals, voltage
convention, automatic protection limits, EEPROM setting memory. Per meter: both
poll intervals. Changing a bound applies it immediately if the current setpoint
falls outside it, because a bound that does not contain the setpoint is not a
bound. Changing the voltage convention or turning automatic protection on
clears `protection_applied_`, so the trip thresholds go out again in the new
terms rather than at the next identification.

Three things are exposed as entities rather than fixed values, and all three
are persisted in flash, which wins over anything the configuration says:

- **`address`**, per inverter and per meter. Required, and the only way to set
  one: there is no numeric fallback. A slot pointed at the wrong address cannot
  be reached at all, so leaving the only remedy in a rebuild was not tenable.
  New devices sit at 0, which every path reads as an empty slot and never puts
  on the bus.
- **`phase_count`** and **`pv_strings`**, the two capability overrides.
  Changing either restarts identification, because that is what re-derives
  everything that followed from the old answer.

The entities are republished whenever the underlying value changes, not only at
boot. An entity left showing a stale address is worse than no entity, because
it looks authoritative.

## Health, presence and probing

Inverters and meters use the same idea of "stalled" and "offline", pushed down
from the hub: online under 10 s, stalled under 20 s, offline beyond that. Not
because they share a bus - they may not - but because the question being asked
is the same one, and a second set of timeouts would be two things to tune where
one will do.

Offline is not just a label. A model without storage simply shuts down when the
panels go dark, and each pointless query then costs more bus time in timeouts
than a whole valid cycle. So an offline inverter stops being polled and is
checked with a one register probe every 60 s instead. When it answers, it is
re-identified from scratch, because everything it was told may have been lost
across the power cycle — and that also re-applies the trip limits.

An identification run that *times out* is different from one that reads zeros.
Zeros are a definitive "not supported"; silence tells us nothing, and treating
the two alike would quietly downgrade a storage inverter to grid tie after one
lost frame. An incomplete run is retried up to three times, 60 s apart, and
then says so loudly rather than running on wrong assumptions.

Writes jump ahead of polls so a user action is not delayed by a cycle, and a
write that is never acknowledged is dropped with an error rather than retried
forever.

## The power controller

It runs on the hub's update tick (default 2 s) but only acts every
`step_interval` (default 6 s), so the effective cadence is the step interval
rounded up to a multiple of the update interval. `dump_config` prints the
figure it actually works out to.

Order of business in one cycle:

1. **Off grid** — the mains contactor input says the grid is gone. Export is
   impossible and the house needs everything, so every inverter goes to
   `offgrid_power_rate` (default 100 %).
2. **Meter offline** — while connected to the mains we can no longer tell
   whether we are exporting. What to do about that is a judgement the site
   owner has to make, so `meter_offline_action` offers three:

   - **Stop** puts everything to zero. Safe for an export limited site and the
     default, but a comms fault at midday throws away real production.
   - **Hold** keeps the last setpoints. Right where the site imports far more
     than it generates, since the previous setpoints could not have been
     exporting either. Wrong if load can vanish while the meter is away.
   - **Hold then reduce** holds for `meter_offline_hold`, then walks each
     inverter down by `min_step` per cycle to its own `safe_power_rate`. Most
     outages are brief, and a stepwise descent means little is lost if the
     meter returns mid-way.

   `safe_power_rate` is per inverter and editable. Raising it above what a unit
   is currently doing applies immediately, since the point of raising it is to
   get production back; lowering it does not, because that would cut output
   while the meter is healthy.
3. **Meter stalled** — a few missed frames are not a reason to move anything.
   Hold.
4. **Above the contractual export cap** (`grid_export_limit`, 0 disables) —
   per phase trimming is the right shape for ordinary export, but a breach of a
   contractual limit is a different problem, and one inverter per cycle is far
   too slow. Every unit actually feeding the grid comes down by `max_step` at
   once.
5. **Stop exporting**, worst phase first. One inverter per phase per cycle. The
   inverter list is walked *backwards*, so the last declared unit is the first
   to give way. Two rounds per phase: single phase units wired to that phase
   first, three phase units only after, because trimming a three phase unit
   also cuts phases that may still be importing.
6. **One increase**, whichever inverter can absorb the most.
7. **Rebalance**, only if nothing above applied.

Two numbers do most of the work. `headroom_up_` asks how much more an inverter
could deliver: a three phase unit spreads evenly, so it is bound by the phase
with the least headroom and needs three times that to move one phase, while a
single phase unit only looks at the phase it feeds. `step_for_` turns watts
into a percentage step, `gain × |W| / nameplate × 100`, clamped between
`min_step` and `max_step`, falling back to `min_step` when the nameplate figure
was implausible.

**Every step is rounded to whole percent and floored at one.** The active power
register takes whole percent and `apply_power_rate()` rounds to it, so a step
below half a percent lands the inverter on the value it already had: the cycle
spends a write, changes nothing, and the next cycle sees the same state and
computes the same useless step. `min_step` is validated at one or more for the
same reason. If a system needs finer control than one percent of nameplate, the
answer is a longer `step_interval`, not a smaller step.

Three rules that came out of watching it misbehave:

- **Only units actually injecting can reduce an export.** A storage inverter
  charging its battery at full PV power reports 100 % commanded and sends
  nothing out; reducing it achieves nothing. The threshold is 50 W to the grid.
- **`can_produce_more` needs a source, not just headroom.** Without the
  daylight test the controller walks every grid tie inverter up to 100 %
  through the night, and they are all wide open at first light. A storage unit
  is exempt: it can keep supplying from the battery after dark, as long as its
  SOC is above the Grid First stop SOC.
- **Raising output raises voltage**, so an inverter on a phase already near the
  high limit is skipped rather than pushed further.

Rebalancing is the last resort and runs after the increase pass on purpose.
When nothing can be raised, it is usually a three phase unit pinned by whichever
phase has the least headroom — and single phase inverters loading the quieter
phases are what keeps that headroom small. Giving up some of their output buys
three times as much from a three phase unit. Every phase meaningfully below the
busiest one is treated, because two lightly loaded phases block just as
effectively as one, and it only proceeds if some three phase unit is actually
able to take up the slack.

**Grid voltage is watched from both ends.** The meter is the grid reference,
but it is not the first to see a rise: the drop across the AC cabling means an
inverter measures more at its own terminals than the meter does, and it is the
inverter that trips at fault 300. So increases are blocked when either source
is over the limit, each compared against the threshold in its own convention -
`reports_line_voltage()` already resolves that for the protection registers and
is reused here.

Blocking is not enough on its own, though. If the setpoint is already high when
the voltage climbs, refusing to raise it further leaves the unit sitting exactly
where it is until the hardware trips. So when the meter is over the limit and no
inverter is, every active unit gives up one `min_step`. That case means the rise
is at the point of connection with no single unit to blame. It is deliberately
gentle, because voltage responds to the whole neighbourhood and cutting hard
would throw away production for a problem that may not be ours.

Note what this does *not* cover: when an inverter reports over voltage, nothing
is reduced, only blocked. See "Started but not built".

Two safety behaviours worth keeping: everything starts at `startup_power_rate`
(default 0) after a reboot, so a restart never leaves the inverters running
unsupervised at whatever they had before; and the current setpoint is rewritten
every `refresh_interval` (default 60 s) even when unchanged, in case the
firmware expects to hear from us. With holding 2 cleared those writes stay out
of the EEPROM.

Setting `min_power_rate` and `max_power_rate` to the same value takes an
inverter out of automatic control without removing it from the configuration.

## Hub thresholds

Every limit is written down once, in the hub, persisted to flash, and exposed as
an editable number: phase and line voltage windows, UPS instantaneous and
average load, battery SOC minimum and maximum, and the export cap. The derived
binary sensors (`grid_over_voltage`, `ups_overloaded`, `battery_below_min`, ...)
let the YAML side consume the resulting condition without repeating the number
anywhere.

Grid voltage is judged from the reference meter rather than from each inverter:
the readings differ by a few volts between units, and the meter is the one
consistent source.

The same thresholds, widened by `inverter_protection_margin` (default 10 %),
are what gets written into each inverter's own trip registers, so there is room
to react before the hardware disconnects and locks us out for minutes.

## Design principles worth keeping

**Detect from data, not from declarations.** Applies to inverter capabilities,
meter phase count, voltage convention, and whether the per phase power
registers mean anything. It is the only thing that survived contact with three
different families. Each of these was first written as "the SPH does X" and
each had to be rewritten the moment a fourth unit turned up.

**A missing answer is not a negative answer.** An identification step that
times out must not silently downgrade capabilities.

**Import and export are summed separately across phases.** With no netting at
the meter, +500 W on L1 and -300 W on L2 is billed as 500 W drawn and 300 W
given away, not 200 W net. Averaging or netting them first hides exactly the
condition the controller exists to prevent.

**Asymmetry where the costs are asymmetric.** Stopping export is urgent and
gets a higher gain; raising production is cautious because overshooting costs
money. Increases walk the inverter list forwards, decreases walk it backwards.

**Compare opportunities rather than applying fixed preferences.** The single
versus three phase choice for an increase is settled by asking which inverter
can absorb more power, which handles both balanced and lopsided imports without
a tuning constant.

**Hardware constants belong in configuration.** Battery module voltage and
capacity, protection margins, gains. The defaults match the development
hardware but nothing assumes them.

**Plausibility guards on anything used for arithmetic.** Nameplate power, TP,
and the meter phase count are all checked before anything is derived from them.

**A conclusion drawn from one sample is a guess.** Several of the detections
here look at conditions that a healthy inverter can imitate for a moment —
zero power with current flowing, no PV voltage, a phase reading nothing. Where
the mistake would be latched, the evidence is required to repeat before it is
believed, and the threshold that decides "there is enough signal to judge at
all" is configurable rather than a constant someone has to recompile.

**Cost bus time only for what is asked for.** The meter's line and energy
blocks, and every storage block on the inverters, are skipped entirely when
nothing needs them. That is the concrete payoff of capability detection.

## Bus timing

At 9600 baud a byte is 1.04 ms and a response costs about
`(3 + 2 × registers + 2) × 1.04` ms.

Reads are split into a fast cycle carrying only what the controller needs and a
slow cycle for counters and diagnostics, on a separate slow interval (default
30 s). Both are editable per device at runtime. Fast blocks per inverter: input
0-56, input 101-105, plus input 1009-1014 and 1067-1081 on storage units.

The figures below describe one bus. Splitting the meter onto its own removes it
from this budget entirely, which is most of the point: it is polled far more
often than the inverters and it is the one reading the controller cannot work
without. Each bus also gets its own `send_wait_time` and `turnaround_time`, so
the meter no longer has to wait out settings chosen for the slowest inverter.

`turnaround_time` in the modbus component applies to every transaction
regardless of size and dominates the budget. Four devices at 450 ms saturate
the bus; 50 ms is comfortable and still an order of magnitude above the 3.5
character silence the standard requires.

`BUS_YIELD_MS` (15 ms, matched in the meter) is a short pause a device takes
after its own transaction. Without it, the component registered first would
chain its blocks back to back and starve the others, because ESPHome runs
`loop()` in registration order. It costs nothing on a bus with one device, so it
is not conditional on how the buses are split.

The response timeout is 1.5 s with two retries, per the document's note that
commands should be at least 850 ms apart.

## How to extend

### Adding a sensor

1. Add the member and a `GI_SETTER` line in `growatt_inverter.h`.
2. Publish it from the parse function for whichever block contains its
   register.
3. Add one row to `SENSORS` in `__init__.py`: key, setter, unit, decimals,
   device class, state class.

If the register is outside every block currently read, extend a block rather
than adding a transaction. Check the bus budget first.

### Adding an editable setting

1. Add an entry to `SettingField` in the header.
2. Add its address and engineering scale to `SETTING_ADDR` and `SETTING_SCALE`
   in the `.cpp`, in the same order.
3. Add a row to `SETTING_NUMBERS` in `__init__.py` with range, step and unit.
4. Mirror the new index in the enum block near the top of `__init__.py`.

Initial values are picked up automatically: the parse functions loop over the
table and read anything whose address falls inside the block they just
received.

### Adding a register backed switch or select

Use `GrowattRegisterSwitch` or `GrowattRegisterSelect`; they take an address
and write to it directly. Add a row to `REGISTER_SWITCHES` or pass the address
in `to_code`. Read back is automatic through `publish_reg_entities_()` for any
address inside any block that is read, not just the first holding group.

### Adding a hub threshold

Add to `HubSetting`, then one row to `HUB_SETTINGS` with its default, range and
unit. Persistence, publishing and the number entity follow from the table. If
it should also drive a condition, add the binary sensor to `HUB_BINARY_SENSORS`
and evaluate it in `update_conditions_()`.

### Adding an identification step

Add to `IdentStep`, send in `send_step_()`, parse in `on_modbus_data()`, and
chain it in `advance_()`. Remember that a step which fails marks the whole run
incomplete and triggers a retry.

## Testing method that worked

**Use a bench inverter, not the production bus.** The USB port that the
ShineWiFi dongle plugs into is a plain TTL serial port speaking the same Modbus
at 115200 baud, on the inverter's own configured address. One ESP32 and four
wires gives a bus with a single master and no contention, which makes every
problem attributable.

**Dump before guessing.** The `dump_registers` button sweeps holding and input
at 0, 1000 and 3000 in chunks of 25 and prints them in a format that pastes
into an issue. Several of the facts above came from reading a dump rather than
from the document.

**Let the log explain the decision.** Every control cycle prints what it read,
what each inverter could do and why the others were skipped — rate, bounds,
watts injected, derating with its meaning spelled out, headroom, and room to
grow. When behaviour looks wrong, the log usually already contains the answer.

**Change one thing.** Most wrong turns in this project came from changing
several files and then not knowing which change broke the build. When several
files change together, replace all of them.

## Still unverified

- Whether derating mode 1 blocks legitimately or just means low sun.
- Whether the Storage family expects phase or line voltage in registers 52-53;
  it is currently forced to phase in the development configuration.
- The scale of input 101, and why it reads zero on every unit here.
- Whether the SPH exposes its per phase AC power anywhere else. The storage
  block has per phase figures for the UPS output but nothing equivalent for the
  grid side, so per phase production on that unit is currently unobtainable.
- The meaning of the meter identification block at holding 0xFC00-0xFC03. The
  raw words are logged rather than mapped, because no documented mapping was
  available.
- Whether Growatt firmware genuinely requires periodic setpoint refreshes. The
  component rewrites every 60 s as a precaution; registers 42, 304 and 307
  relate to communication loss handling and would settle the question.
- Whether the rebalancing gains are worth their complexity on a system whose
  single phase inverter is small relative to the three phase ones.

## Started but not built

**Battery charge rate as a controllable load.** When the property is importing
and no inverter can raise its output, the remaining lever is the storage unit's
charge rate: reducing it frees whatever the battery was absorbing, whether that
came from PV or from the grid. Raising it back when exporting soaks surplus up
instead of curtailing production, which is why the restore step has to run
before any curtailment.

Three things are unsettled. Whether it should be opt in per inverter or on by
default for any storage unit; whether the floor is fixed at 0 or configurable;
and, most importantly, whether writes to 1090 reach the EEPROM. Holding 2
protects only registers 3, 4, 5 and 99, so the charge rate probably does not
enjoy that protection - at controller cadence that would be roughly 600 flash
writes an hour, which destroys the part in weeks. The intended answer is a
separate, much slower interval for charge rate changes plus writing only on a
real change, but the premise needs checking on the bench first.

Also still unknown: what `grid_active_power` reports on an SPH drawing from the
grid. The export reduction step skips units injecting under 50 W, so the answer
decides whether a charging SPH is correctly passed over.

**Reducing when an inverter reports over voltage.** Today that case only blocks
increases. If the setpoint is already high the unit will trip at 300 anyway,
which is a real hole: a MOD 40K was observed sitting in fault for over an hour,
and no restart cleared it - only removing the PV input, because the fault was
never in the inverter, it was the voltage it measured. Whether to reduce there
too, and whether that should be the offending unit alone or everything on its
phases, is unsettled.

**Meter model select as an entity.** The address moved out of YAML into an
entity on both device types; the meter's model override did not, and there is
no particular reason for the inconsistency beyond nobody having asked.

## Working with Claude on this

What made the process work:

- **Bring real logs.** A single control cycle at debug level answers more
  questions than any description of the symptom.
- **Say when something contradicts what was assumed.** Several bugs were found
  because an observation did not match the model, and following that up was
  faster than debugging around it.
- **Push back on design.** The proportional step, the "only reduce inverters
  that are injecting" rule and the phase priority ordering all came from the
  hardware owner disagreeing with a first proposal.
- **Ask why before accepting a mechanism.** The derating explanation led
  directly to finding a bug that would have deadlocked the controller.
