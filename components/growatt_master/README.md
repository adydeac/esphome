# ESPHome Growatt Master

An ESPHome external component that manages several Growatt inverters and
Eastron smart meters on one Modbus RTU bus, with a built-in self-consumption
controller. It is a replacement for a Growatt ShineMaster in installations
where multiple inverters need to be coordinated against a single grid meter.

## What it does

- **Identifies inverters automatically.** Phase count, PV strings, battery and
  UPS support are worked out from live measurements rather than from a model
  table, so unknown models generally work without changes.
- **Reads everything.** Around 85 optional sensors per inverter and 25 per
  meter. Declaring one in YAML is what makes it appear.
- **Writes settings.** Power rate, charge and discharge limits, protection
  thresholds, and the Grid First / Battery First time windows, which are sent
  as one atomic Modbus block because the firmware requires it.
- **Controls production.** Adjusts each inverter's active power rate per phase
  to match production to consumption, prioritising the elimination of export.
- **Stays inside the bus budget.** Reads are split into a fast cycle for
  control-relevant registers and a slow cycle for counters and diagnostics.

## Tested hardware

| Inverter | DTC | Notes |
|---|---|---|
| Growatt MOD 40K | 5001 | three phase, 8 MPPT trackers |
| Growatt SPH 10000TL3 BH-UP | 3601 | three phase, storage, UPS |
| Growatt MIN 6000TL-X | 5100 | single phase |
| Eastron SDM630 | — | three phase meter |

Other Growatt models are likely to work: the component detects capabilities
instead of relying on a model list. Reports from other hardware are welcome,
and the `Dump Registers` button produces output that can be pasted straight
into an issue.

## Installation

```yaml
external_components:
  - source: github://adydeac/esphome-growatt-master
    components: [ growatt_master ]
```

## Minimal configuration

```yaml
uart:
  - id: uart_growatt
    tx_pin: GPIO16
    rx_pin: GPIO17
    baud_rate: 9600
    stop_bits: 1

modbus:
  - id: modbus_growatt
    uart_id: uart_growatt
    role: client
    send_wait_time: 400ms
    turnaround_time: 50ms

growatt_master:
  id: master
  modbus_id: modbus_growatt
  max_inverters: 4
  update_interval: 2s

  inverters:
    - id: inverter1
      address: 11
      info:
        name: "Inverter 1 Info"
      grid_active_power:
        name: "Inverter 1 Grid Power"
      energy_today:
        name: "Inverter 1 Energy Today"

  meters:
    - id: grid_meter
      address: 2
      phase_a:
        active_power:
          name: "Grid L1 Power"
```

A complete configuration covering three inverters, a meter and the controller
is in [`example/growatt-master.yaml`](example/growatt-master.yaml).

Full documentation, in the ESPHome format, is in
[`docs/growatt_master.rst`](docs/growatt_master.rst).

## Wiring

The component speaks Modbus RTU over RS485, normally to the inverter's COM
port at 9600 baud. A MAX485 or MAX3485 transceiver is enough; connect its
driver enable pin to a GPIO and declare it as `flow_control_pin` on the UART if
your board needs one.

For bench work, the USB-A socket that the ShineWiFi dongle plugs into is a
plain TTL serial port speaking the same protocol at 115200 baud, on the
inverter's own configured address. Four wires from that socket to an ESP32
give a bus with a single device and no contention, which makes debugging much
easier.

## A note on how this was written

This component was written in its entirety with the assistance of Claude AI,
over many cycles of building, testing against real inverters, and correcting.
A substantial part of what it knows contradicts the manufacturer's protocol
document, and all of it was verified on hardware.

Those findings, the reasoning behind the design, and guidance for extending the
component the same way are in [`CLAUDE.md`](CLAUDE.md). Read it before changing
any register handling.

## Contributing

The most useful contributions are reports from hardware that is not in the
table above. If a value looks wrong or a capability is misdetected:

1. Press `Dump Registers` and capture the log.
2. Enable `growatt_master: DEBUG` and capture one control cycle.
3. Open an issue with both, plus the model name and what you expected.

Before assuming a register behaves as documented, check the "Hard won facts"
section of `CLAUDE.md`. Several registers return zeros instead of exceptions,
and the Storage family deviates from the documented map in at least six places.

## License

Licensed under the GNU General Public License v3.0, the same license as
ESPHome. See [`LICENSE`](LICENSE).
