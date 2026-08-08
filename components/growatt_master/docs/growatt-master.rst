Growatt Master
==============

.. seo::
    :description: Instructions for setting up Growatt solar inverters and Eastron
        smart meters on a shared Modbus bus, with automatic self-consumption control.
    :image: solar-power.svg
    :keywords: Growatt, ShineMaster, SDM630, Modbus, solar, inverter

The ``growatt_master`` component manages one or more Growatt inverters and one
or more Eastron style smart meters sharing a single Modbus RTU bus, and includes
a self-consumption controller that adjusts inverter output to keep the property
from exporting to the grid.

It is a replacement for a Growatt ShineMaster in installations where several
inverters must be coordinated against a single grid meter.

.. figure:: images/growatt-master.png
    :align: center
    :width: 80.0%

.. note::

    This component was written in its entirety with the assistance of Claude AI.
    Every register mapping was verified against real hardware; see
    ``CLAUDE.md`` in the repository for the design notes and protocol findings
    that guided it, and for guidance on extending the component the same way.

Overview
--------

A single ``growatt_master`` entry owns everything on the bus:

- **Inverters**, identified automatically. The component works out how many
  phases and PV strings each unit has, whether it has a battery, and whether it
  supports UPS output, all from live measurements rather than a model table.
- **Meters**, polled for per phase power and voltage. Import and export are
  summed separately so installations without netting between phases are billed
  correctly.
- **A controller** that raises and lowers each inverter's active power rate to
  match production to consumption, one phase at a time.

.. code-block:: yaml

    # Example configuration entry
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
      max_inverters: 8
      update_interval: 2s

      inverters:
        - id: inverter1
          address: 11
          info:
            name: "Inverter 1 Info"
          grid_active_power:
            name: "Inverter 1 Grid Power"

      meters:
        - id: grid_meter
          address: 2
          phase_a:
            active_power:
              name: "Grid L1 Power"

Configuration variables
-----------------------

- **id** (*Optional*, :ref:`config-id`): Manually specify the ID for this
  component.
- **modbus_id** (*Optional*, :ref:`config-id`): The Modbus bus every inverter
  and meter is attached to.
- **max_inverters** (**Required**, int): Upper bound on the number of inverter
  slots, 1 to 32. Configuring more inverters than this is a validation error.
- **inverters** (**Required**, list): One entry per inverter. Order matters: it
  is the priority order used by the controller.
- **meters** (*Optional*, list): One entry per smart meter. The first meter is
  the grid reference used by the controller.
- **update_interval** (*Optional*, :ref:`config-time`): How often aggregates,
  conditions and the controller are evaluated. Defaults to ``2s``.
- **grid_power_sensor_id** (*Optional*, :ref:`config-id`): A
  :doc:`binary sensor </components/binary_sensor/index>` reading a contactor
  that tells the component whether the mains is connected. When omitted the
  grid is assumed to always be available.
- **average_samples** (*Optional*, int): Window length for the import and
  export averages, 1 to 60. Defaults to ``60``.
- **meter_stalled_timeout** (*Optional*, :ref:`config-time`): After this long
  without a meter frame the controller stops adjusting but takes no other
  action. Defaults to ``10s``.
- **meter_offline_timeout** (*Optional*, :ref:`config-time`): After this long
  the meter is considered gone and, if the mains is connected, all inverters
  are taken to zero. Defaults to ``20s``.

Controller options
******************

- **step_interval** (*Optional*, :ref:`config-time`): Minimum time between
  adjustments. Defaults to ``6s``. The effective period is this value rounded
  up to a multiple of ``update_interval``.
- **import_threshold** (*Optional*, float): Import in watts that must be
  exceeded before production is raised. Defaults to ``100``.
- **export_threshold** (*Optional*, float): Export in watts that triggers a
  reduction. Defaults to ``0``.
- **increase_gain** (*Optional*, float): Fraction of the measured deficit each
  upward step tries to close, 0.05 to 1.0. Defaults to ``0.5``.
- **decrease_gain** (*Optional*, float): Same for downward steps. Defaults to
  ``0.8``. The two are deliberately different: overshooting upwards means
  exporting, which costs money, while overshooting downwards only means
  importing a little longer.
- **min_step** / **max_step** (*Optional*, float): Bounds on a single
  adjustment, in percent. Default to ``1`` and ``20``.
- **startup_power_rate** (*Optional*, int): Rate every inverter is set to when
  the component starts. Defaults to ``0``, so a reboot never leaves inverters
  running unsupervised.
- **offgrid_power_rate** (*Optional*, int): Rate used while the grid power
  sensor reports no mains. Defaults to ``100``.
- **refresh_interval** (*Optional*, :ref:`config-time`): How often the current
  setpoint is rewritten even when unchanged, in case the inverter expects
  regular contact. Defaults to ``60s``.
- **inverter_protection_margin** (*Optional*, float): How far outside the
  controller's own voltage window each inverter's trip thresholds are pushed,
  in percent. Defaults to ``10``.
- **inverter_restart_delay** (*Optional*, int): Value written to each
  inverter's restart delay register, in seconds. Defaults to ``30``.

Thresholds
**********

Each of these is an optional :doc:`number </components/number/index>` with a
box input. Values are kept by the component and survive a reboot; the YAML
figure is only the first boot default.

- **grid_phase_voltage_low** (default 207 V) and **grid_phase_voltage_high**
  (default 253 V): phase to neutral limits.
- **grid_line_voltage_low** (default 360 V) and **grid_line_voltage_high**
  (default 440 V): line to line limits, used for inverters that expect their
  protection registers in line terms.
- **ups_max_load** (default 95 %) and **ups_max_load_avg** (default 90 %).
- **battery_soc_min** (default 10 %) and **battery_soc_max** (default 100 %).
- **grid_export_limit** (default 0 W): hard cap the controller enforces itself.
  Zero disables it. This is never written to the inverters' own export limit
  registers.

Hub sensors
***********

- **meter_import** / **meter_export**: positive phase powers and negative phase
  powers summed separately.
- **meter_import_average** / **meter_export_average**: rolling averages of the
  above.
- **meter_state** (:doc:`text sensor </components/text_sensor/index>`):
  ``online``, ``stalled`` or ``offline``.
- **controller_state** (text sensor): what the controller is doing, for example
  ``balanced`` or ``reducing to stop export``.

Hub binary sensors
******************

Derived from the thresholds above so the limits live in one place. Useful for
grid switching logic written in YAML.

- **grid_power**: what the component believes about the mains.
- **grid_over_voltage** / **grid_under_voltage**: any meter phase outside the
  phase voltage window.
- **ups_overloaded** / **ups_overloaded_average**.
- **battery_below_min** / **battery_above_max**.

Inverter configuration
----------------------

.. code-block:: yaml

    growatt_master:
      inverters:
        - id: inverter1
          address: 11
          update_interval: 6s
          slow_update_interval: 40s
          info:
            name: "Inverter 1 Info"

- **id** (*Optional*, :ref:`config-id`): Needed if lambdas refer to this
  inverter.
- **address** (*Optional*, int): Modbus address, 0 to 254. ``0`` means the slot
  is empty and is never polled. The value stored in flash takes precedence, so
  this is only the first boot default; the address can be changed at runtime
  through a template number calling ``change_address()``.
- **update_interval** (*Optional*, :ref:`config-time`): Fast cycle carrying
  everything the controller needs. Defaults to ``10s``; ``2s`` to ``6s`` is
  usual.
- **slow_update_interval** (*Optional*, :ref:`config-time`): Cycle for
  counters, temperatures and diagnostics. Defaults to ``30s``.
- **min_power_rate** / **max_power_rate** (*Optional*, int): Bounds the
  controller must stay inside. Setting both to the same value takes this
  inverter out of automatic control.

Capability overrides
********************

Everything here is detected automatically. Setting a value forces it.

- **phases** (*Optional*, int): ``0`` for automatic, or ``1`` or ``3``.
  Detected by counting live grid voltages.
- **strings** (*Optional*, int): ``0`` for automatic, or 1 to 8. Detected by
  counting PV inputs carrying voltage; the highest count seen is kept, since
  strings read zero at night.
- **ups** / **battery** (*Optional*, boolean): Omit for automatic detection.
- **phase** (*Optional*, select): Which mains phase a single phase inverter
  feeds, ``L1`` to ``L3``. This cannot be detected, because the inverter only
  ever sees one voltage. Ignored on three phase units.

Protection limits
*****************

- **auto_protection_limits** (*Optional*, boolean): When true, the component
  widens this inverter's own trip thresholds beyond the controller's window so
  there is room to reduce power before the hardware disconnects. Defaults to
  ``false``.
- **voltage_convention** (*Optional*, string): ``auto``, ``phase`` or ``line``.
  Some three phase units report 230 V phase voltages but expect their
  protection registers written in 400 V line terms; ``auto`` picks line
  whenever the unit populates its line to line registers.
- **protect_eeprom** (*Optional*, boolean): Clears holding register 2 at
  identification so the frequent power rate writes stay volatile. Defaults to
  ``false``.

.. warning::

    Registers 52 and 53 are grid code protection thresholds. A wrong value can
    stop the inverter connecting, or stop it disconnecting when it should,
    which is why ``auto_protection_limits`` is off by default.

Battery geometry
****************

Only used by the derived calculations. Defaults match Growatt ARK 2.5 modules.

- **battery_module_voltage** (default 53.75 V)
- **battery_module_capacity** (default 2.5 kWh)
- **battery_discharge_hours** (default 2.5 h)
- **ups_load_average_samples** (default 60)

Inverter sensors
****************

Every sensor is optional; declaring one is what makes it appear as an entity.
All accept the usual :ref:`sensor <config-sensor>` options.

Read on every fast cycle:

``status_code``, ``pv_active_power``, ``grid_active_power``, ``frequency``,
``energy_today``, ``energy_total``, ``real_power_percent``,
``output_max_power``, ``derating_mode``, ``fault_code``, ``fault_subcode``,
``warning_bits``, ``warning_subcode``, ``warning_code``.

Groups of three, each with ``voltage``, ``current`` and ``active_power``:
``phase_a``, ``phase_b``, ``phase_c``, ``pv1`` through ``pv8``,
``ups_phase_a`` through ``ups_phase_c``.

Single values: ``line_voltage_ab``, ``line_voltage_bc``, ``line_voltage_ca``,
``pv1_energy_today`` through ``pv8_energy_total``.

Read on the slow cycle: ``work_time_total``, ``pv_energy_total``,
``temperature``, ``ipm_temperature``, ``boost_temperature``,
``battery_voltage_dsp``, ``bus_voltage_p``, ``bus_voltage_n``,
``output_power_factor``.

Storage models only: ``system_work_mode``, ``fault_word``, ``battery_voltage``,
``battery_soc``, ``battery_charge_power``, ``battery_discharge_power``,
``battery_capacity``, ``battery_cycles``, ``battery_health``,
``battery_temperature``, ``charge_energy_today`` and its variants,
``power_to_user``, ``power_to_grid``, ``local_load_power``, the matching energy
counters, ``ups_frequency``, ``ups_load``, ``ups_power_factor``, and the
``bms_*`` family.

Computed by the component: ``ups_total_power``, ``ups_load_average``,
``ups_max_power``, ``battery_modules``.

Read once during identification: ``normal_power``, ``modbus_version``,
``active_rate``, ``reactive_rate``, ``power_factor_set``,
``pv_nominal_voltage``, ``com_address``, ``pf_model``, ``tracker_model``.

Inverter text sensors
*********************

``info``, ``firmware``, ``firmware_build``, ``serial_number``,
``manufacturer``, ``model``, ``bootloader``, ``system_time``, ``status``,
``fault``, ``derating``.

.. note::

    Growatt does not expose a commercial model name over Modbus. ``model``
    reads the INV Type string where it exists and is empty on the Storage
    family; ``info`` falls back to the serial number and always shows the
    device type code, which is the only reliable family discriminator.

Inverter controls
*****************

Settings written immediately with function 0x06, all
:doc:`numbers </components/number/index>` with a box input:

``active_power_rate``, ``grid_first_discharge_rate``, ``grid_first_stop_soc``,
``battery_first_charge_rate``, ``battery_first_stop_soc``,
``pv_start_voltage``, ``start_time``, ``restart_delay``, ``grid_voltage_low``,
``grid_voltage_high``, ``grid_frequency_low``, ``grid_frequency_high``,
``export_limit_rate``.

Switches: ``ac_charge``, ``setting_memory``, ``inverter_power``, ``bdc_power``,
``ups_enable``.

Selects: ``battery_type_select``, ``export_limit_mode``, ``phase``.

Buttons: ``refresh`` re-runs identification, ``dump_registers`` prints the
register map to the log.

.. warning::

    ``inverter_power`` and ``bdc_power`` write holding register 0, which turns
    the unit on and off. ESPHome switches default to
    ``restore_mode: RESTORE_DEFAULT_OFF``, which would shut the inverter down
    on every reboot. Set ``restore_mode: RESTORE_DEFAULT_ON`` or leave these
    undeclared.

Time windows
************

Grid First and Battery First periods must reach the inverter as one atomic
block, so edits are staged in memory and sent together when the matching
``apply`` button is pressed, using function 0x10.

.. code-block:: yaml

    grid_first:
      apply:
        name: "Grid First Apply"
      period1:
        start_hour:
          name: "Grid First P1 Start Hour"
        start_minute:
          name: "Grid First P1 Start Minute"
        stop_hour:
          name: "Grid First P1 Stop Hour"
        stop_minute:
          name: "Grid First P1 Stop Minute"
        enabled:
          name: "Grid First P1 Enabled"

``battery_first`` takes the same shape. Periods may wrap past midnight. The
component refuses to write when an enabled Grid First period overlaps an
enabled Battery First period, and says which two in the log.

Meter configuration
-------------------

.. code-block:: yaml

    growatt_master:
      meters:
        - id: grid_meter
          address: 2
          update_interval: 2s
          model_select:
            name: "Grid Meter Model"
          phase_a:
            voltage:
              name: "Grid L1 Voltage"
            active_power:
              name: "Grid L1 Power"

- **address** (*Optional*, int): Modbus address. ``0`` means not present.
- **update_interval** / **slow_update_interval**: as for inverters.
- **model_select** (*Optional*, select): ``Auto``, ``SDM120``, ``SDM220``,
  ``SDM230`` or ``SDM630``. Every Eastron meter shares the same register map,
  which is why they are drop in replacements for each other, so the model
  cannot be read from the map. What actually differs is the phase count, and
  that is detected from live voltages. The select is therefore an override.

Per phase groups ``phase_a`` to ``phase_c`` accept ``voltage``, ``current``,
``active_power``, ``apparent_power``, ``reactive_power`` and ``power_factor``.

Other sensors: ``total_active_power``, ``total_apparent_power``,
``total_reactive_power``, ``total_power_factor``, ``average_voltage``,
``average_current``, ``sum_current``, ``frequency``, ``import_active_energy``,
``export_active_energy``, ``import_reactive_energy``,
``export_reactive_energy``, ``line_voltage_ab`` and siblings,
``average_line_voltage``, ``neutral_current``, ``total_active_energy``,
``total_reactive_energy``.

Line voltages and total energies are only requested when at least one of their
sensors is declared, so they cost nothing otherwise.

How the controller works
------------------------

Each cycle, in order:

1. **No mains**, according to the grid power sensor: every inverter goes to
   ``offgrid_power_rate``.
2. **Meter offline** while connected to the mains: production is stopped,
   because there is no longer any way to tell whether the property is
   exporting.
3. **Meter stalled**: nothing moves. A few missed frames are not a reason to
   act.
4. **Otherwise**, export is cleared first and only then is production raised.

Stopping export is the priority. Phases are handled worst first, and the
inverter list is walked backwards so the last declared unit gives way first.
Single phase units on the offending phase are preferred, because trimming a
three phase unit would also cut phases that may still be importing. Inverters
delivering nothing to the grid are skipped: a storage unit charging its battery
at full PV power cannot make the export any smaller.

For increases, the inverter that can absorb the most is chosen. A three phase
unit spreads evenly and is bound by its weakest phase, so it covers three times
that phase; a single phase unit covers only its own. Comparing those two
figures settles the choice on its own, without a fixed preference.

Steps are proportional to the measured deviation rather than fixed, so a large
deficit closes in a few cycles instead of dozens.

Bus timing
----------

At 9600 baud a byte takes about 1.04 ms, so a response costs roughly
``(3 + 2 × registers + 2) × 1.04`` milliseconds. A fast cycle for one inverter
is about 156 ms, or 227 ms with battery and UPS blocks; a meter is about
126 ms.

The dominant cost is usually ``turnaround_time``, which applies to every
transaction regardless of size. With four devices at 450 ms the bus saturates;
50 ms leaves ample headroom while still being an order of magnitude above the
3.5 character silence the standard requires.

.. code-block:: yaml

    modbus:
      - id: modbus_growatt
        send_wait_time: 400ms
        turnaround_time: 50ms

Occasional ``bus busy, will retry`` messages at verbose level are normal with
several devices sharing a bus. Values going stale, or ``no answer on poll
block``, are not.

See Also
--------

- :doc:`/components/modbus`
- :doc:`/components/uart`
- :doc:`/components/sensor/index`
- :ghedit_ ``growatt_master``
