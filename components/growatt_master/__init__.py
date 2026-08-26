import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (
    binary_sensor,
    button,
    modbus,
    number,
    select,
    sensor,
    switch,
    text_sensor,
)
from esphome.const import (
    CONF_ACTIVE_POWER,
    CONF_ADDRESS,
    CONF_CURRENT,
    CONF_FREQUENCY,
    CONF_ID,
    CONF_MODE,
    CONF_PHASE_A,
    CONF_PHASE_B,
    CONF_PHASE_C,
    CONF_TEMPERATURE,
    CONF_VOLTAGE,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    UNIT_HOUR,
    UNIT_PERCENT,
    UNIT_SECOND,
    UNIT_VOLT,
    UNIT_WATT,
)

CODEOWNERS = ["@mikesnet"]
DEPENDENCIES = ["modbus"]
AUTO_LOAD = [
    "sensor",
    "text_sensor",
    "binary_sensor",
    "number",
    "switch",
    "button",
    "select",
]
MULTI_CONF = False

UNIT_KWH = "kWh"
UNIT_VOLT_AMPS = "VA"

CONF_MAX_INVERTERS = "max_inverters"
CONF_INVERTERS = "inverters"
CONF_PHASES = "phases"
CONF_STRINGS = "strings"
CONF_UPS = "ups"
CONF_BATTERY = "battery"
CONF_INFO = "info"
CONF_MODULE_VOLTAGE = "battery_module_voltage"
CONF_MODULE_CAPACITY = "battery_module_capacity"
CONF_DISCHARGE_HOURS = "battery_discharge_hours"
CONF_UPS_AVG_WINDOW = "ups_load_average_samples"
# Optional split of the bus. 'modbus_id' stays valid on its own and keeps
# everything on one bus; naming either of the specific ones routes that device
# type elsewhere. A device may still override its own bus.
# Must match OFFLINE_ACTIONS in growatt_master.cpp.
OFFLINE_ACTIONS = ["Stop", "Hold", "Hold then reduce"]
CONF_OFFLINE_ACTION = "meter_offline_action"
CONF_OFFLINE_HOLD = "meter_offline_hold"
CONF_SAFE_RATE = "safe_power_rate"
# Must match CONV_NAMES in growatt_inverter.cpp.
CONVENTIONS = ["Auto", "Phase", "Line"]
# kind indices must match RATE_* in growatt_inverter.cpp
INVERTER_RATES = {
    "min_power_rate": (0, 0, 100, 1, UNIT_PERCENT, "mdi:arrow-collapse-down"),
    "max_power_rate": (1, 0, 100, 1, UNIT_PERCENT, "mdi:arrow-collapse-up"),
    "update_interval": (2, 1, 600, 1, UNIT_SECOND, "mdi:timer-outline"),
    "slow_update_interval": (3, 5, 3600, 5, UNIT_SECOND,
                                    "mdi:timer-sand"),
}

CONF_INVERTERS_MODBUS_ID = "inverters_modbus_id"
CONF_METERS_MODBUS_ID = "meters_modbus_id"

# Bus level address change tools, unrelated to the devices declared below.
CONF_ADDR_CHANGE = "change_inverter_address"
CONF_ADDR_FROM = "change_inverter_address_from"
CONF_ADDR_TO = "change_inverter_address_to"
CONF_ADDR_STATUS = "change_inverter_address_status"
CONF_MADDR_CHANGE = "change_meter_address"
CONF_MADDR_FROM = "change_meter_address_from"
CONF_MADDR_TO = "change_meter_address_to"
CONF_MADDR_STATUS = "change_meter_address_status"
CONF_ADDR_TOOL_ID = "address_tool_id"
CONF_MADDR_TOOL_ID = "meter_address_tool_id"
# Output below which the per phase power registers cannot be judged.
CONF_PHASE_DETECT_MIN = "phase_power_detect_threshold"
# Counters and diagnostics use this cadence; update_interval stays fast enough
# for control decisions.
# Same string as ESPHome's own CONF_UPDATE_INTERVAL; ours is the entity.
CONF_UPDATE_INTERVAL_NUM = "update_interval"
CONF_SLOW_UPDATE_INTERVAL = "slow_update_interval"

def _component_config(conf):
    """The config as register_component wants it, minus 'update_interval'.

    Taking that name for an entity is not enough on its own: register_component
    reads the key straight out of the config and hands it to
    set_update_interval, which then receives the entity's configuration
    dictionary instead of a time. Stripping it here leaves setup_priority and
    anything else intact.
    """
    return {k: v for k, v in conf.items() if k != CONF_UPDATE_INTERVAL_NUM}


# Poll intervals are entities, not YAML keys: two keys for one setting confused
# more than it helped. These are the values a device starts at before flash has
# anything to say.
DEFAULT_HUB_INTERVAL = 2000
DEFAULT_INVERTER_INTERVAL = 10000
DEFAULT_METER_INTERVAL = 2000
DEFAULT_SLOW_INTERVAL = 30000

ns = cg.esphome_ns.namespace("growatt_master")
GrowattHub = ns.class_("GrowattHub", cg.Component)
GrowattInverter = ns.class_(
    "GrowattInverter", cg.PollingComponent, modbus.ModbusClientDevice
)
GrowattSettingNumber = ns.class_("GrowattSettingNumber", number.Number)
GrowattWindowNumber = ns.class_("GrowattWindowNumber", number.Number)
GrowattWindowSwitch = ns.class_("GrowattWindowSwitch", switch.Switch)
GrowattAcChargeSwitch = ns.class_("GrowattAcChargeSwitch", switch.Switch)
GrowattApplyButton = ns.class_("GrowattApplyButton", button.Button)
GrowattRefreshButton = ns.class_("GrowattRefreshButton", button.Button)
GrowattDumpButton = ns.class_("GrowattDumpButton", button.Button)
GrowattInverterAddressNumber = ns.class_(
    "GrowattInverterAddressNumber", number.Number
)
GrowattMeterAddressNumber = ns.class_("GrowattMeterAddressNumber", number.Number)
GrowattOfflineActionSelect = ns.class_(
    "GrowattOfflineActionSelect", select.Select
)
GrowattSafeRateNumber = ns.class_("GrowattSafeRateNumber", number.Number)
GrowattRateNumber = ns.class_("GrowattRateNumber", number.Number)
GrowattConventionSelect = ns.class_("GrowattConventionSelect", select.Select)
GrowattInverterOptionSwitch = ns.class_(
    "GrowattInverterOptionSwitch", switch.Switch
)
GrowattMeterIntervalNumber = ns.class_(
    "GrowattMeterIntervalNumber", number.Number
)
GrowattPhaseCountSelect = ns.class_("GrowattPhaseCountSelect", select.Select)
GrowattStringsSelect = ns.class_("GrowattStringsSelect", select.Select)
GrowattAddressTool = ns.class_(
    "GrowattAddressTool", cg.Component, modbus.ModbusClientDevice
)
GrowattAddressNumber = ns.class_("GrowattAddressNumber", number.Number)
GrowattAddressButton = ns.class_("GrowattAddressButton", button.Button)
GrowattPhaseSelect = ns.class_("GrowattPhaseSelect", select.Select)
GrowattRegisterSelect = ns.class_("GrowattRegisterSelect", select.Select)
GrowattRegisterSwitch = ns.class_("GrowattRegisterSwitch", switch.Switch)
GrowattMeter = ns.class_(
    "GrowattMeter", cg.PollingComponent, modbus.ModbusClientDevice
)
GrowattMeterModelSelect = ns.class_("GrowattMeterModelSelect", select.Select)
GrowattHubNumber = ns.class_("GrowattHubNumber", number.Number)

# ------------------------------ hub thresholds ------------------------------
# Written down once here and consumed through the derived binary sensors, so
# the YAML side never has to repeat a limit.
CONF_GRID_POWER_SENSOR_ID = "grid_power_sensor_id"
# Health timeouts, applied to inverters and meters alike: the same question,
# so there is no reason for them to disagree about what stalled means.

# ------------------------------ power controller ------------------------------

(
    HUB_PHASE_V_LOW,
    HUB_PHASE_V_HIGH,
    HUB_LINE_V_LOW,
    HUB_LINE_V_HIGH,
    HUB_UPS_MAX_LOAD,
    HUB_UPS_MAX_LOAD_AVG,
    HUB_BATTERY_SOC_MIN,
    HUB_BATTERY_SOC_MAX,
    HUB_GRID_EXPORT_LIMIT,
    HUB_UPDATE_INTERVAL,
    HUB_STEP_INTERVAL,
    HUB_REFRESH_INTERVAL,
    HUB_AVERAGE_SAMPLES,
    HUB_STALLED_TIMEOUT,
    HUB_OFFLINE_TIMEOUT,
    HUB_OFFLINE_PROBE,
    HUB_IMPORT_THRESHOLD,
    HUB_EXPORT_THRESHOLD,
    HUB_INCREASE_GAIN,
    HUB_DECREASE_GAIN,
    HUB_MIN_STEP,
    HUB_MAX_STEP,
    HUB_OFFGRID_RATE,
    HUB_PROTECTION_MARGIN,
    HUB_RESTART_DELAY,
    HUB_VOLTAGE_SOFT_MARGIN,
    HUB_CAPABILITY_RATIO,
    HUB_CAPABILITY_WINDOW,
) = range(28)

CONF_REBALANCING = "phase_rebalancing"
CONF_REBALANCE_THRESHOLD = "rebalance_threshold"

# Must match VoltageConvention in growatt_inverter.h
VOLTAGE_CONVENTIONS = {"auto": 0, "phase": 1, "line": 2}

# key -> (field, default, min, max, step, unit, icon)
# Defaults are the EU 230 V / 400 V nominal plus or minus 10 %. The ranges are
# wide enough to also cover 120 V and 208 V systems.
HUB_SETTINGS = {
    "grid_phase_voltage_low": (HUB_PHASE_V_LOW, 207, 70, 250, 1, UNIT_VOLT,
                               "mdi:transmission-tower-off"),
    "grid_phase_voltage_high": (HUB_PHASE_V_HIGH, 253, 100, 300, 1, UNIT_VOLT,
                                "mdi:transmission-tower"),
    "grid_line_voltage_low": (HUB_LINE_V_LOW, 360, 130, 420, 1, UNIT_VOLT,
                              "mdi:transmission-tower-off"),
    "grid_line_voltage_high": (HUB_LINE_V_HIGH, 440, 170, 480, 1, UNIT_VOLT,
                               "mdi:transmission-tower"),
    "ups_max_load": (HUB_UPS_MAX_LOAD, 95, 0, 100, 1, UNIT_PERCENT,
                     "mdi:gauge-full"),
    "ups_max_load_avg": (HUB_UPS_MAX_LOAD_AVG, 90, 0, 100, 1, UNIT_PERCENT,
                         "mdi:gauge"),
    "battery_soc_min": (HUB_BATTERY_SOC_MIN, 10, 0, 100, 1, UNIT_PERCENT,
                        "mdi:battery-low"),
    "battery_soc_max": (HUB_BATTERY_SOC_MAX, 100, 0, 100, 1, UNIT_PERCENT,
                        "mdi:battery-high"),
    # Hard cap the controller enforces itself; it is never written to the
    # inverters' own export limit registers. 0 disables the cap.
    "grid_export_limit": (HUB_GRID_EXPORT_LIMIT, 0, 0, 100000, 100, UNIT_WATT,
                          "mdi:transmission-tower-export"),
    # Controller tuning. All of these were compile time options; the value here
    # is the starting point and whatever is set at runtime wins after that.
    # Intervals are in seconds because that is what a person tuning them thinks
    # in, and because a number entity has no notion of "5min".
    "update_interval": (HUB_UPDATE_INTERVAL, 2, 0.5, 60, 0.5,
                               UNIT_SECOND, "mdi:timer-outline"),
    "step_interval": (HUB_STEP_INTERVAL, 6, 1, 300, 1, UNIT_SECOND,
                      "mdi:timer-cog-outline"),
    "refresh_interval": (HUB_REFRESH_INTERVAL, 60, 5, 3600, 5, UNIT_SECOND,
                         "mdi:refresh"),
    "average_samples": (HUB_AVERAGE_SAMPLES, 60, 1, 60, 1, None,
                        "mdi:chart-bell-curve"),
    "device_stalled_timeout": (HUB_STALLED_TIMEOUT, 10, 1, 600, 1, UNIT_SECOND,
                               "mdi:timer-sand"),
    "device_offline_timeout": (HUB_OFFLINE_TIMEOUT, 20, 2, 3600, 1, UNIT_SECOND,
                               "mdi:timer-alert-outline"),
    "device_offline_probe_interval": (HUB_OFFLINE_PROBE, 60, 5, 3600, 5,
                                      UNIT_SECOND, "mdi:radar"),
    "import_threshold": (HUB_IMPORT_THRESHOLD, 100, 0, 10000, 10, UNIT_WATT,
                         "mdi:transmission-tower-import"),
    "export_threshold": (HUB_EXPORT_THRESHOLD, 0, 0, 10000, 10, UNIT_WATT,
                         "mdi:transmission-tower-export"),
    "increase_gain": (HUB_INCREASE_GAIN, 0.5, 0.05, 2, 0.05, None,
                      "mdi:trending-up"),
    "decrease_gain": (HUB_DECREASE_GAIN, 0.8, 0.05, 2, 0.05, None,
                      "mdi:trending-down"),
    "min_step": (HUB_MIN_STEP, 1, 1, 50, 1, UNIT_PERCENT, "mdi:step-forward"),
    "max_step": (HUB_MAX_STEP, 20, 1, 100, 1, UNIT_PERCENT,
                 "mdi:step-forward-2"),
    "offgrid_power_rate": (HUB_OFFGRID_RATE, 100, 0, 100, 1, UNIT_PERCENT,
                           "mdi:transmission-tower-off"),
    "inverter_protection_margin": (HUB_PROTECTION_MARGIN, 10, 0, 50, 1,
                                   UNIT_PERCENT, "mdi:shield-outline"),
    "inverter_restart_delay": (HUB_RESTART_DELAY, 30, 0, 600, 1, UNIT_SECOND,
                               "mdi:timer-refresh-outline"),
    "grid_voltage_soft_margin": (HUB_VOLTAGE_SOFT_MARGIN, 8, 0, 50, 1,
                                 UNIT_VOLT, "mdi:speedometer-slow"),
    # An inverter is only telling us something about its capability while our
    # own limit is what it is actually hitting. Output at or above this
    # fraction of the limit our setpoint implies counts as binding.
    "capability_binding_ratio": (HUB_CAPABILITY_RATIO, 0.9, 0.5, 1.0, 0.01,
                                 None, "mdi:gauge"),
    "capability_window": (HUB_CAPABILITY_WINDOW, 3600, 60, 86400, 60,
                          UNIT_SECOND, "mdi:history"),
}

# key -> (setter, icon, device_class)
HUB_BINARY_SENSORS = {
    "grid_power": ("set_grid_power_sensor", "mdi:transmission-tower", None),
    "grid_over_voltage": ("set_grid_over_voltage", "mdi:flash-alert", "problem"),
    "grid_under_voltage": ("set_grid_under_voltage", "mdi:flash-off", "problem"),
    "ups_overloaded": ("set_ups_overloaded", "mdi:alert", "problem"),
    "ups_overloaded_average": ("set_ups_overloaded_avg", "mdi:alert-outline",
                               "problem"),
    "battery_below_min": ("set_battery_below_min", "mdi:battery-alert", "problem"),
    "battery_above_max": ("set_battery_above_max", "mdi:battery-check", None),
}

# ------------------------------- smart meters -------------------------------
CONF_METERS = "meters"
CONF_MODEL_SELECT = "model_select"

# All Eastron meters share the same register map, so the model is an override
# rather than something the map can reveal. Order must match MeterModel in
# growatt_meter.h.
METER_MODELS = ["Auto", "SDM120", "SDM220", "SDM230", "SDM630"]

# kind indices must match MK_* in growatt_meter.cpp
METER_PHASE_KINDS = [
    (0, CONF_VOLTAGE),
    (1, CONF_CURRENT),
    (2, CONF_ACTIVE_POWER),
    (3, "apparent_power"),
    (4, "reactive_power"),
    (5, "power_factor"),
]

AUTO = -1
KIND_VOLTAGE, KIND_CURRENT, KIND_POWER = 0, 1, 2

# Must match the enums in growatt_inverter.h
MODE_GRID_FIRST, MODE_BATTERY_FIRST = 0, 1
(
    SET_ACTIVE_POWER_RATE,
    SET_GF_DISCHARGE_RATE,
    SET_GF_STOP_SOC,
    SET_BF_CHARGE_RATE,
    SET_BF_STOP_SOC,
    SET_PV_START_VOLTAGE,
    SET_START_TIME,
    SET_RESTART_DELAY,
    SET_GRID_V_LOW,
    SET_GRID_V_HIGH,
    SET_GRID_F_LOW,
    SET_GRID_F_HIGH,
    SET_EXPORT_LIMIT_RATE,
) = range(13)

# Holding addresses written directly by the register backed selects
ADDR_BATTERY_TYPE = 1048
ADDR_EXPORT_LIMIT = 122

CONF_GRID_FIRST = "grid_first"
CONF_BATTERY_FIRST = "battery_first"
CONF_AC_CHARGE = "ac_charge"
CONF_APPLY = "apply"
CONF_ENABLED = "enabled"
CONF_REFRESH = "refresh"
CONF_DUMP = "dump_registers"
CONF_PHASE_SELECT = "phase"
CONF_BATTERY_TYPE_SELECT = "battery_type_select"
CONF_EXPORT_LIMIT_SELECT = "export_limit_mode"

# key -> (holding address, on value, off value, icon)
# Holding 2 decides whether registers 3, 4, 5 and 99 survive a power cycle.
# Turning it off keeps the frequent power rate writes out of the EEPROM and
# makes the inverter come back unrestricted if the controller stops running.
# Holding 0 is a command register: 1/0 drive the inverter, 3/2 the BDC.
REGISTER_SWITCHES = {
    "setting_memory": (2, 1, 0, "mdi:content-save-cog"),
    "inverter_power": (0, 1, 0, "mdi:power"),
    "bdc_power": (0, 3, 2, "mdi:battery-charging"),
    "ups_enable": (1060, 1, 0, "mdi:home-lightning-bolt"),
}

PHASE_OPTIONS = ["L1", "L2", "L3"]
# Capability overrides. These strings are matched in growatt_inverter.cpp and
# the string count must stay in step with MAX_STRINGS in the header.
CONF_PHASE_COUNT_SELECT = "phase_count"
CONF_STRINGS_SELECT = "pv_strings"
PHASE_COUNT_OPTIONS = ["Auto", "Single phase", "Three phase"]
STRING_OPTIONS = ["Auto"] + [str(i) for i in range(1, 9)]
# Order is the value written to holding 1048. The protocol document contradicts
# itself between this register and the input registers that report battery
# type, so the labels follow the writable register.
BATTERY_TYPE_OPTIONS = ["Lithium", "Lead-acid", "Other"]
# Holding 122: 0 disable, 1 via RS485, 2 via RS232, 3 via CT
EXPORT_LIMIT_OPTIONS = ["Disabled", "RS485 meter", "RS232 meter", "CT clamp"]

PERIOD_KEYS = ["period1", "period2", "period3"]
# (key, max value) pairs, order must match WindowPart in the header
PART_KEYS = [
    ("start_hour", 23),
    ("start_minute", 59),
    ("stop_hour", 23),
    ("stop_minute", 59),
]

# key -> (field, min, max, step, unit, icon)
SETTING_NUMBERS = {
    "active_power_rate": (SET_ACTIVE_POWER_RATE, 0, 100, 1, UNIT_PERCENT,
                          "mdi:speedometer"),
    "grid_first_discharge_rate": (SET_GF_DISCHARGE_RATE, 0, 100, 1,
                                  UNIT_PERCENT, "mdi:battery-arrow-down"),
    "grid_first_stop_soc": (SET_GF_STOP_SOC, 0, 100, 1, UNIT_PERCENT,
                            "mdi:battery-low"),
    "battery_first_charge_rate": (SET_BF_CHARGE_RATE, 0, 100, 1, UNIT_PERCENT,
                                  "mdi:battery-arrow-up"),
    "battery_first_stop_soc": (SET_BF_STOP_SOC, 0, 100, 1, UNIT_PERCENT,
                               "mdi:battery-high"),
    "pv_start_voltage": (SET_PV_START_VOLTAGE, 0, 1000, 0.1, UNIT_VOLT,
                         "mdi:flash-outline"),
    "start_time": (SET_START_TIME, 0, 600, 1, UNIT_SECOND, "mdi:timer-play"),
    "restart_delay": (SET_RESTART_DELAY, 0, 600, 1, UNIT_SECOND,
                      "mdi:timer-refresh"),
    # Range covers both conventions: a unit reporting line voltage needs
    # limits around 400 V, not 230 V.
    "grid_voltage_low": (SET_GRID_V_LOW, 0, 600, 0.1, UNIT_VOLT,
                         "mdi:transmission-tower"),
    "grid_voltage_high": (SET_GRID_V_HIGH, 0, 600, 0.1, UNIT_VOLT,
                          "mdi:transmission-tower"),
    "grid_frequency_low": (SET_GRID_F_LOW, 45, 55, 0.01, UNIT_HERTZ,
                           "mdi:sine-wave"),
    "grid_frequency_high": (SET_GRID_F_HIGH, 45, 65, 0.01, UNIT_HERTZ,
                            "mdi:sine-wave"),
    "export_limit_rate": (SET_EXPORT_LIMIT_RATE, 0, 100, 0.1, UNIT_PERCENT,
                          "mdi:export"),
}

# ---------------------------------------------------------------------------
# Sensor table: key -> (setter, unit, decimals, device_class, state_class)
# Everything the component reads is exposed here; declaring a sensor in YAML
# is what makes it appear as an entity.
# ---------------------------------------------------------------------------
_M = STATE_CLASS_MEASUREMENT
_T = STATE_CLASS_TOTAL_INCREASING
_V, _A, _W, _C = UNIT_VOLT, UNIT_AMPERE, UNIT_WATT, UNIT_CELSIUS
_DV, _DA, _DW = DEVICE_CLASS_VOLTAGE, DEVICE_CLASS_CURRENT, DEVICE_CLASS_POWER
_DE, _DT = DEVICE_CLASS_ENERGY, DEVICE_CLASS_TEMPERATURE

SENSORS = {
    # --- input 0..124, polled every update_interval ---
    "status_code": ("set_status_code", None, 0, None, None),
    "pv_active_power": ("set_pv_active_power", _W, 1, _DW, _M),
    "capability": (
        "set_capability", UNIT_WATT, 0, DEVICE_CLASS_POWER, _M),
    "grid_active_power": ("set_grid_active_power", _W, 1, _DW, _M),
    CONF_FREQUENCY: ("set_frequency", UNIT_HERTZ, 2, DEVICE_CLASS_FREQUENCY, _M),
    "energy_today": ("set_energy_today", UNIT_KWH, 1, _DE, _T),
    "energy_total": ("set_energy_total", UNIT_KWH, 1, _DE, _T),
    "work_time_total": ("set_work_time_total", UNIT_HOUR, 1,
                        DEVICE_CLASS_DURATION, _T),
    "pv_energy_total": ("set_pv_energy_total_all", UNIT_KWH, 1, _DE, _T),
    CONF_TEMPERATURE: ("set_temperature", _C, 1, _DT, _M),
    "ipm_temperature": ("set_ipm_temperature", _C, 1, _DT, _M),
    "boost_temperature": ("set_boost_temperature", _C, 1, _DT, _M),
    "battery_voltage_dsp": ("set_battery_voltage_dsp", _V, 1, _DV, _M),
    "bus_voltage_p": ("set_bus_voltage_p", _V, 1, _DV, _M),
    "bus_voltage_n": ("set_bus_voltage_n", _V, 1, _DV, _M),
    "output_power_factor": ("set_output_power_factor", None, 0, None, _M),
    "real_power_percent": ("set_real_power_percent", UNIT_PERCENT, 0, None, _M),
    "output_max_power": ("set_output_max_power", _W, 1, _DW, _M),
    "derating_mode": ("set_derating_mode", None, 0, None, None),
    "fault_code": ("set_fault_code", None, 0, None, None),
    "fault_subcode": ("set_fault_subcode", None, 0, None, None),
    "warning_bits": ("set_warning_bits", None, 0, None, None),
    "warning_subcode": ("set_warning_subcode", None, 0, None, None),
    "warning_code": ("set_warning_code", None, 0, None, None),
    # --- holding 0..124, read once during identification ---
    "normal_power": ("set_normal_power", UNIT_VOLT_AMPS, 1, None, None),
    "modbus_version": ("set_modbus_version", None, 2, None, None),
    "active_rate": ("set_active_rate", UNIT_PERCENT, 0, None, None),
    "reactive_rate": ("set_reactive_rate", UNIT_PERCENT, 0, None, None),
    "power_factor_set": ("set_power_factor_set", None, 0, None, None),
    "pv_nominal_voltage": ("set_pv_nominal_voltage", _V, 1, _DV, None),
    "com_address": ("set_com_address_sensor", None, 0, None, None),
    "pf_model": ("set_pf_model", None, 0, None, None),
    "tracker_model": ("set_tracker_model", None, 0, None, None),
    # --- storage only: input 1000..1096 ---
    "system_work_mode": ("set_system_work_mode", None, 0, None, None),
    "fault_word": ("set_fault_word", None, 0, None, None),
    "battery_voltage": ("set_battery_voltage", _V, 1, _DV, _M),
    "battery_soc": ("set_battery_soc", UNIT_PERCENT, 0, DEVICE_CLASS_BATTERY, _M),
    "battery_charge_power": ("set_battery_charge_power", _W, 1, _DW, _M),
    "battery_discharge_power": ("set_battery_discharge_power", _W, 1, _DW, _M),
    "battery_capacity": ("set_battery_capacity", UNIT_KWH, 1, None, _M),
    "battery_cycles": ("set_battery_cycles", None, 0, None, _M),
    "battery_health": ("set_battery_health", UNIT_PERCENT, 0, None, _M),
    "battery_temperature": ("set_battery_temperature", _C, 1, _DT, _M),
    "charge_energy_today": ("set_charge_energy_today", UNIT_KWH, 1, _DE, _T),
    "charge_energy_total": ("set_charge_energy_total", UNIT_KWH, 1, _DE, _T),
    "discharge_energy_today": ("set_discharge_energy_today", UNIT_KWH, 1, _DE, _T),
    "discharge_energy_total": ("set_discharge_energy_total", UNIT_KWH, 1, _DE, _T),
    "ac_charge_energy_today": ("set_ac_charge_energy_today", UNIT_KWH, 1, _DE, _T),
    "ac_charge_energy_total": ("set_ac_charge_energy_total", UNIT_KWH, 1, _DE, _T),
    "ac_charge_power": ("set_ac_charge_power", _W, 1, _DW, _M),
    "priority": ("set_priority", None, 0, None, None),
    "battery_type": ("set_battery_type", None, 0, None, None),
    "power_to_user": ("set_power_to_user", _W, 1, _DW, _M),
    "power_to_grid": ("set_power_to_grid", _W, 1, _DW, _M),
    "local_load_power": ("set_local_load_power", _W, 1, _DW, _M),
    "energy_to_user_today": ("set_energy_to_user_today", UNIT_KWH, 1, _DE, _T),
    "energy_to_user_total": ("set_energy_to_user_total", UNIT_KWH, 1, _DE, _T),
    "energy_to_grid_today": ("set_energy_to_grid_today", UNIT_KWH, 1, _DE, _T),
    "energy_to_grid_total": ("set_energy_to_grid_total", UNIT_KWH, 1, _DE, _T),
    "local_load_energy_today": ("set_local_load_energy_today", UNIT_KWH, 1, _DE, _T),
    "local_load_energy_total": ("set_local_load_energy_total", UNIT_KWH, 1, _DE, _T),
    "ups_frequency": ("set_ups_frequency", UNIT_HERTZ, 2, DEVICE_CLASS_FREQUENCY,
                      _M),
    "ups_load": ("set_ups_load", UNIT_PERCENT, 1, None, _M),
    "ups_power_factor": ("set_ups_power_factor", None, 1, None, _M),
    "bms_soc": ("set_bms_soc", UNIT_PERCENT, 0, DEVICE_CLASS_BATTERY, _M),
    "bms_voltage": ("set_bms_voltage", _V, 2, _DV, _M),
    "bms_current": ("set_bms_current", _A, 2, _DA, _M),
    "bms_temperature": ("set_bms_temperature", _C, 1, _DT, _M),
    # --- derived, computed inside the component ---
    "ups_total_power": ("set_ups_total_power", _W, 1, _DW, _M),
    "ups_load_average": ("set_ups_load_avg", UNIT_PERCENT, 1, None, _M),
    "ups_max_power": ("set_ups_max_power", UNIT_PERCENT, 0, None, _M),
    "battery_modules": ("set_battery_modules", None, 0, None, _M),
}

TEXT_SENSORS = {
    CONF_INFO: "set_info_text_sensor",
    "firmware": "set_firmware_text_sensor",
    "firmware_build": "set_fw_build_text_sensor",
    "serial_number": "set_serial_text_sensor",
    "manufacturer": "set_manufacturer_text_sensor",
    "model": "set_model_text_sensor",
    "bootloader": "set_bootloader_text_sensor",
    "system_time": "set_system_time_text_sensor",
    "status": "set_status_text_sensor",
    "fault": "set_fault_text_sensor",
    "derating": "set_derating_text_sensor",
    "state": "set_state_text_sensor",
}

PVS = [f"pv{i}" for i in range(1, 9)]
PV_ENERGY_TODAY = [f"pv{i}_energy_today" for i in range(1, 9)]
PV_ENERGY_TOTAL = [f"pv{i}_energy_total" for i in range(1, 9)]
UPS_PHASES = ["ups_phase_a", "ups_phase_b", "ups_phase_c"]
LINE_VOLTAGES = ["line_voltage_ab", "line_voltage_bc", "line_voltage_ca"]

# Smart meter sensors, same table format as SENSORS above.
METER_SENSORS = {
    "total_active_power": ("set_total_active_power", _W, 1, _DW, _M),
    "total_apparent_power": ("set_total_apparent_power", UNIT_VOLT_AMPS, 1,
                             None, _M),
    "total_reactive_power": ("set_total_reactive_power", "var", 1, None, _M),
    "total_power_factor": ("set_total_power_factor", None, 2, None, _M),
    "average_voltage": ("set_average_voltage", _V, 1, _DV, _M),
    "average_current": ("set_average_current", _A, 2, _DA, _M),
    "sum_current": ("set_sum_current", _A, 2, _DA, _M),
    CONF_FREQUENCY: ("set_frequency", UNIT_HERTZ, 2, DEVICE_CLASS_FREQUENCY, _M),
    "import_active_energy": ("set_import_active_energy", UNIT_KWH, 2, _DE, _T),
    "export_active_energy": ("set_export_active_energy", UNIT_KWH, 2, _DE, _T),
    "import_reactive_energy": ("set_import_reactive_energy", "kvarh", 2, None, _T),
    "export_reactive_energy": ("set_export_reactive_energy", "kvarh", 2, None, _T),
    "average_line_voltage": ("set_average_line_voltage", _V, 1, _DV, _M),
    "neutral_current": ("set_neutral_current", _A, 2, _DA, _M),
    "total_active_energy": ("set_total_active_energy", UNIT_KWH, 2, _DE, _T),
    "total_reactive_energy": ("set_total_reactive_energy", "kvarh", 2, None, _T),
}


def _schema_from(entry):
    _, unit, decimals, device_class, state_class = entry
    kwargs = {"accuracy_decimals": decimals}
    if unit is not None:
        kwargs["unit_of_measurement"] = unit
    if device_class is not None:
        kwargs["device_class"] = device_class
    if state_class is not None:
        kwargs["state_class"] = state_class
    return sensor.sensor_schema(**kwargs)


def _box_number(class_, **kwargs):
    """number_schema with a box input by default.

    Every value here is typed rather than dragged, so a slider is the wrong
    control. Overriding the schema default still lets a user pick another mode
    from YAML.
    """
    # number_schema validates unit_of_measurement as a string, so a dimensionless
    # entity has to omit the argument rather than pass None.
    kwargs = {k: v for k, v in kwargs.items() if v is not None}
    return number.number_schema(class_, **kwargs).extend(
        {cv.Optional(CONF_MODE, default="BOX"): cv.enum(
            number.NUMBER_MODES, upper=True)}
    )


def _volt():
    return sensor.sensor_schema(
        unit_of_measurement=_V, accuracy_decimals=1,
        device_class=_DV, state_class=_M,
    )


def _energy():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_KWH, accuracy_decimals=1,
        device_class=_DE, state_class=_T,
    )


TRIPLE_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_VOLTAGE): _volt(),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=_A, accuracy_decimals=1,
            device_class=_DA, state_class=_M,
        ),
        cv.Optional(CONF_ACTIVE_POWER): sensor.sensor_schema(
            unit_of_measurement=_W, accuracy_decimals=1,
            device_class=_DW, state_class=_M,
        ),
    }
)


def _period_schema():
    schema = {
        cv.Optional(CONF_ENABLED): switch.switch_schema(
            GrowattWindowSwitch, icon="mdi:calendar-check",
            default_restore_mode="DISABLED",
        ),
    }
    for key, _ in PART_KEYS:
        schema[cv.Optional(key)] = _box_number(
            GrowattWindowNumber, icon="mdi:clock-outline"
        )
    return cv.Schema(schema)


def _window_schema():
    schema = {
        cv.Optional(CONF_APPLY): button.button_schema(
            GrowattApplyButton, icon="mdi:content-save-check"
        ),
    }
    for key in PERIOD_KEYS:
        schema[cv.Optional(key)] = _period_schema()
    return cv.Schema(schema)


def _inverter_schema():
    schema = {
        cv.GenerateID(): cv.declare_id(GrowattInverter),
        # Overrides the bus this device type routes to, for the odd unit that
        # sits on the other line.
        cv.Optional(modbus.CONF_MODBUS_ID): cv.use_id(modbus.ModbusClient),
        # 0 or omitted = slot not present. The value stored in flash takes
        # precedence at boot, so this is only the first-boot default.
        # The address is an entity rather than a fixed value: it lives in
        # flash and is the one thing that has to be fixable from the UI, since
        # a slot pointed at the wrong address cannot be reached at all. New
        # devices start at 0, which every part of the component reads as
        # "this slot is empty" and never puts on the bus.
        cv.Required(CONF_ADDRESS): _box_number(
            GrowattInverterAddressNumber, icon="mdi:identifier"
        ),
        # Where this unit parks when the meter has been gone too long to keep
        # holding. Editable at runtime; raising it above the current output
        # applies immediately.
        cv.Optional(CONF_SAFE_RATE): _box_number(
            GrowattSafeRateNumber, icon="mdi:shield-half-full",
            unit_of_measurement=UNIT_PERCENT,
        ),
        **{
            cv.Optional(k): _box_number(
                GrowattRateNumber, unit_of_measurement=u, icon=i
            )
            for k, (_kind, _lo, _hi, _s, u, i) in INVERTER_RATES.items()
        },
        cv.Optional("voltage_convention_select"): select.select_schema(
            GrowattConventionSelect, icon="mdi:sine-wave"
        ),
        cv.Optional("auto_protection_limits_switch"): switch.switch_schema(
            GrowattInverterOptionSwitch, icon="mdi:shield-outline",
            default_restore_mode="DISABLED",
        ),
        cv.Optional("protect_eeprom_switch"): switch.switch_schema(
            GrowattInverterOptionSwitch, icon="mdi:memory",
            default_restore_mode="DISABLED",
        ),
        cv.Optional(CONF_PHASES, default=0): cv.int_range(min=0, max=3),
        cv.Optional(CONF_STRINGS, default=0): cv.int_range(min=0, max=8),
        # The same two overrides as editable entities. They write the same
        # persisted fields, so whichever was touched last is what applies.
        cv.Optional(CONF_PHASE_COUNT_SELECT): select.select_schema(
            GrowattPhaseCountSelect, icon="mdi:sine-wave"
        ),
        cv.Optional(CONF_STRINGS_SELECT): select.select_schema(
            GrowattStringsSelect, icon="mdi:solar-panel"
        ),
        cv.Optional(CONF_UPS): cv.boolean,
        cv.Optional(CONF_BATTERY): cv.boolean,
        # Battery pack geometry, used only by the derived calculations.
        # Defaults match Growatt ARK 2.5 modules.
        cv.Optional(CONF_MODULE_VOLTAGE, default=53.75): cv.positive_float,
        cv.Optional(CONF_MODULE_CAPACITY, default=2.5): cv.positive_float,
        cv.Optional(CONF_DISCHARGE_HOURS, default=2.5): cv.positive_float,
        cv.Optional(CONF_UPS_AVG_WINDOW, default=60): cv.int_range(min=1, max=60),
        # Some units report their whole AC output in Pac1 and leave Pac2 and
        # Pac3 at zero. That is detected from live data, but only above this
        # output: below it a phase carrying almost no active power reads the
        # same as one that is not reported at all, and an inverter idling with
        # reactive current on every phase looks exactly like the case being
        # searched for. Lower it to catch the condition on a small inverter,
        # raise it if the warning ever appears on a unit that is fine. Zero
        # disables the floor and leans entirely on the repeat confirmations.
        cv.Optional(CONF_PHASE_DETECT_MIN, default=100): cv.float_range(min=0),
        cv.Optional(CONF_AC_CHARGE): switch.switch_schema(
            GrowattAcChargeSwitch, icon="mdi:transmission-tower-import",
            default_restore_mode="DISABLED",
        ),
        cv.Optional(CONF_GRID_FIRST): _window_schema(),
        cv.Optional(CONF_BATTERY_FIRST): _window_schema(),
        cv.Optional(CONF_REFRESH): button.button_schema(
            GrowattRefreshButton, icon="mdi:refresh"
        ),
        cv.Optional(CONF_DUMP): button.button_schema(
            GrowattDumpButton, icon="mdi:file-search"
        ),
        # Which mains phase this inverter feeds. Manual: the inverter has no
        # way of knowing, it only sees a voltage on Vac1.
        cv.Optional(CONF_PHASE_SELECT): select.select_schema(
            GrowattPhaseSelect, icon="mdi:transmission-tower"
        ),
        cv.Optional(CONF_BATTERY_TYPE_SELECT): select.select_schema(
            GrowattRegisterSelect, icon="mdi:battery-sync"
        ),
        cv.Optional(CONF_EXPORT_LIMIT_SELECT): select.select_schema(
            GrowattRegisterSelect, icon="mdi:export"
        ),
    }
    for key, (_addr, _on, _off, icon) in REGISTER_SWITCHES.items():
        # DISABLED, emphatically: the default restore mode calls write_state()
        # at boot, which for these would write the restored value straight into
        # a live inverter register - 'inverter_power' would switch the unit off
        # on every restart. The real state is read back from the inverter.
        schema[cv.Optional(key)] = switch.switch_schema(
            GrowattRegisterSwitch, icon=icon, default_restore_mode="DISABLED"
        )
    for key, (_, lo, hi, step, unit, icon) in SETTING_NUMBERS.items():
        schema[cv.Optional(key)] = _box_number(
            GrowattSettingNumber, unit_of_measurement=unit, icon=icon
        )
    for key in (CONF_PHASE_A, CONF_PHASE_B, CONF_PHASE_C, *PVS, *UPS_PHASES):
        schema[cv.Optional(key)] = TRIPLE_SCHEMA
    for key in LINE_VOLTAGES:
        schema[cv.Optional(key)] = _volt()
    for key in (*PV_ENERGY_TODAY, *PV_ENERGY_TOTAL):
        schema[cv.Optional(key)] = _energy()
    for key, entry in SENSORS.items():
        schema[cv.Optional(key)] = _schema_from(entry)
    for key in TEXT_SENSORS:
        schema[cv.Optional(key)] = text_sensor.text_sensor_schema()
    return cv.Schema(schema).extend(cv.COMPONENT_SCHEMA)


def _meter_phase_schema():
    return cv.Schema(
        {
            cv.Optional(CONF_VOLTAGE): _volt(),
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                unit_of_measurement=_A, accuracy_decimals=2,
                device_class=_DA, state_class=_M,
            ),
            cv.Optional(CONF_ACTIVE_POWER): sensor.sensor_schema(
                unit_of_measurement=_W, accuracy_decimals=1,
                device_class=_DW, state_class=_M,
            ),
            cv.Optional("apparent_power"): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT_AMPS, accuracy_decimals=1,
                state_class=_M,
            ),
            cv.Optional("reactive_power"): sensor.sensor_schema(
                unit_of_measurement="var", accuracy_decimals=1, state_class=_M,
            ),
            cv.Optional("power_factor"): sensor.sensor_schema(
                accuracy_decimals=2, state_class=_M,
            ),
        }
    )


def _meter_schema():
    schema = {
        cv.GenerateID(): cv.declare_id(GrowattMeter),
        # Overrides the bus this device type routes to, for the odd unit that
        # sits on the other line.
        cv.Optional(modbus.CONF_MODBUS_ID): cv.use_id(modbus.ModbusClient),
        # 0 or omitted = meter not present. Flash wins at boot.
        # Eastron specifies 1..247 for the whole SDM family, so there is
        # nothing above that to reach. 0 marks the slot empty.
        cv.Required(CONF_ADDRESS): _box_number(
            GrowattMeterAddressNumber, icon="mdi:identifier"
        ),
        # Override for phase count; "Auto" leaves detection in charge.
        cv.Optional(CONF_UPDATE_INTERVAL_NUM): _box_number(
            GrowattMeterIntervalNumber, unit_of_measurement=UNIT_SECOND,
            icon="mdi:timer-outline"
        ),
        cv.Optional(CONF_SLOW_UPDATE_INTERVAL): _box_number(
            GrowattMeterIntervalNumber, unit_of_measurement=UNIT_SECOND,
            icon="mdi:timer-sand"
        ),
        cv.Optional(CONF_MODEL_SELECT): select.select_schema(
            GrowattMeterModelSelect, icon="mdi:meter-electric"
        ),
        cv.Optional(CONF_INFO): text_sensor.text_sensor_schema(),
    }
    for key in (CONF_PHASE_A, CONF_PHASE_B, CONF_PHASE_C):
        schema[cv.Optional(key)] = _meter_phase_schema()
    for key in LINE_VOLTAGES:
        schema[cv.Optional(key)] = _volt()
    for key, entry in METER_SENSORS.items():
        schema[cv.Optional(key)] = _schema_from(entry)
    return cv.Schema(schema).extend(cv.COMPONENT_SCHEMA)


def _bus_for(config, key):
    """Which bus a device type routes to: its own, else the shared fallback."""
    return config.get(key) or config.get(modbus.CONF_MODBUS_ID)


def _validate(config):
    for key, what in (
        (CONF_INVERTERS_MODBUS_ID, "inverters"),
        (CONF_METERS_MODBUS_ID, "meters"),
    ):
        if _bus_for(config, key) is None:
            raise cv.Invalid(
                f"no bus for the {what}: set '{key}', or 'modbus_id' to put "
                f"everything on one bus"
            )
    n = len(config[CONF_INVERTERS])
    if n > config[CONF_MAX_INVERTERS]:
        raise cv.Invalid(
            f"{n} inverters defined but max_inverters is "
            f"{config[CONF_MAX_INVERTERS]}"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(GrowattHub),
            # One bus for everything, or one per device type. Keeping the
            # meter on its own bus means a mute inverter can no longer stall
            # the reading the controller depends on, and vice versa.
            cv.Optional(modbus.CONF_MODBUS_ID): cv.use_id(modbus.ModbusClient),
            # What happens once the meter is definitively gone. Stopping is
            # the safe default for an export limited site; the other two trade
            # some of that safety for not throwing away production during a
            # comms fault.
            cv.Optional(CONF_OFFLINE_ACTION): select.select_schema(
                GrowattOfflineActionSelect, icon="mdi:transmission-tower-off"
            ),
            cv.Optional(
                CONF_OFFLINE_HOLD, default="5min"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_INVERTERS_MODBUS_ID): cv.use_id(modbus.ModbusClient),
            cv.Optional(CONF_METERS_MODBUS_ID): cv.use_id(modbus.ModbusClient),
            cv.GenerateID(CONF_ADDR_TOOL_ID): cv.declare_id(GrowattAddressTool),
            cv.GenerateID(CONF_MADDR_TOOL_ID): cv.declare_id(GrowattAddressTool),
            cv.Optional(CONF_MADDR_FROM): _box_number(
                GrowattAddressNumber, icon="mdi:import"
            ),
            cv.Optional(CONF_MADDR_TO): _box_number(
                GrowattAddressNumber, icon="mdi:export"
            ),
            cv.Optional(CONF_MADDR_CHANGE): button.button_schema(
                GrowattAddressButton, icon="mdi:rename-box"
            ),
            cv.Optional(CONF_MADDR_STATUS): text_sensor.text_sensor_schema(
                icon="mdi:information-outline"
            ),
            cv.Required(CONF_MAX_INVERTERS): cv.int_range(min=1, max=32),
            # Commissioning tool: writes holding 30 at whatever address is
            # entered, whether or not that unit is declared under 'inverters'.
            # Both fields default to 0, the Modbus broadcast address, so the
            # button does nothing until they are filled in.
            cv.Optional(CONF_ADDR_FROM): _box_number(
                GrowattAddressNumber, icon="mdi:import"
            ),
            cv.Optional(CONF_ADDR_TO): _box_number(
                GrowattAddressNumber, icon="mdi:export"
            ),
            cv.Optional(CONF_ADDR_CHANGE): button.button_schema(
                GrowattAddressButton, icon="mdi:rename-box"
            ),
            cv.Optional(CONF_ADDR_STATUS): text_sensor.text_sensor_schema(
                icon="mdi:information-outline"
            ),
            cv.Required(CONF_INVERTERS): cv.ensure_list(_inverter_schema()),
            cv.Optional(CONF_METERS): cv.ensure_list(_meter_schema()),
            # Contactor state sensed on a GPIO, telling us whether the mains is
            # connected. Without it the grid is assumed to be always available.
            cv.Optional(CONF_GRID_POWER_SENSOR_ID): cv.use_id(
                binary_sensor.BinarySensor
            ),
            # A single phase inverter on the tightest phase caps every three
            # phase unit. Trading some of its output releases three times as
            # much elsewhere, so the net gain is twice the gap between the
            # tightest phase and the next one.
            cv.Optional(CONF_REBALANCING, default=True): cv.boolean,
            cv.Optional(CONF_REBALANCE_THRESHOLD, default=300): cv.float_range(
                min=0
            ),
            cv.Optional("controller_state"): text_sensor.text_sensor_schema(
                icon="mdi:tune"
            ),
            cv.Optional("meter_import"): sensor.sensor_schema(
                unit_of_measurement=_W, accuracy_decimals=1,
                device_class=_DW, state_class=_M,
            ),
            cv.Optional("meter_export"): sensor.sensor_schema(
                unit_of_measurement=_W, accuracy_decimals=1,
                device_class=_DW, state_class=_M,
            ),
            cv.Optional("meter_import_average"): sensor.sensor_schema(
                unit_of_measurement=_W, accuracy_decimals=1,
                device_class=_DW, state_class=_M,
            ),
            cv.Optional("meter_export_average"): sensor.sensor_schema(
                unit_of_measurement=_W, accuracy_decimals=1,
                device_class=_DW, state_class=_M,
            ),
            cv.Optional("meter_state"): text_sensor.text_sensor_schema(
                icon="mdi:meter-electric"
            ),
            **{
                cv.Optional(k): _box_number(
                    GrowattHubNumber, unit_of_measurement=u, icon=i
                )
                for k, (_f, _d, _lo, _hi, _s, u, i) in HUB_SETTINGS.items()
            },
            **{
                cv.Optional(k): binary_sensor.binary_sensor_schema(
                    icon=i, device_class=d
                ) if d else binary_sensor.binary_sensor_schema(icon=i)
                for k, (_setter, i, d) in HUB_BINARY_SENSORS.items()
            },
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate,
)


async def _setup_triples(inv, conf, keys, setter):
    """Wire voltage/current/power groups to an indexed setter."""
    for i, key in enumerate(keys):
        if key not in conf:
            continue
        group = conf[key]
        for kind, ckey in (
            (KIND_VOLTAGE, CONF_VOLTAGE),
            (KIND_CURRENT, CONF_CURRENT),
            (KIND_POWER, CONF_ACTIVE_POWER),
        ):
            if ckey in group:
                sens = await sensor.new_sensor(group[ckey])
                cg.add(getattr(inv, setter)(i, kind, sens))


async def _setup_indexed(inv, conf, keys, setter):
    for i, key in enumerate(keys):
        if key in conf:
            sens = await sensor.new_sensor(conf[key])
            cg.add(getattr(inv, setter)(i, sens))


async def _setup_windows(inv, conf, key, mode):
    """Create the entities for one time window block."""
    if key not in conf:
        return
    block = conf[key]
    if CONF_APPLY in block:
        btn = await button.new_button(block[CONF_APPLY])
        cg.add(btn.set_parent(inv))
        cg.add(btn.set_mode(mode))
    for period, pkey in enumerate(PERIOD_KEYS):
        if pkey not in block:
            continue
        pconf = block[pkey]
        for part, (part_key, max_value) in enumerate(PART_KEYS):
            if part_key not in pconf:
                continue
            num = await number.new_number(
                pconf[part_key], min_value=0, max_value=max_value, step=1
            )
            cg.add(num.set_parent(inv))
            cg.add(num.set_target(mode, period, part))
            cg.add(inv.set_window_number(mode, period, part, num))
        if CONF_ENABLED in pconf:
            sw = await switch.new_switch(pconf[CONF_ENABLED])
            cg.add(sw.set_parent(inv))
            cg.add(sw.set_target(mode, period))
            cg.add(inv.set_window_switch(mode, period, sw))


async def to_code(config):
    hub = cg.new_Pvariable(config[CONF_ID])
    # No longer taken from YAML: the tick is an entity, and this is only the
    # value it starts at before flash is read.
    cg.add(hub.set_update_interval(DEFAULT_HUB_INTERVAL))
    await cg.register_component(hub, _component_config(config))
    cg.add(hub.set_max_inverters(config[CONF_MAX_INVERTERS]))
    cg.add(hub.set_offline_hold(config[CONF_OFFLINE_HOLD]))
    if CONF_OFFLINE_ACTION in config:
        sel = await select.new_select(
            config[CONF_OFFLINE_ACTION], options=OFFLINE_ACTIONS
        )
        cg.add(sel.set_parent(hub))
        cg.add(hub.set_offline_action_select(sel))

    # One address tool per bus. A ModbusClientDevice belongs to exactly one bus,
    # so a hub spanning two of them cannot be the tool itself. Each sits at 0,
    # the broadcast address, and so never matches an incoming frame except while
    # it is running and has pointed itself somewhere.
    for tool_key, bus_key, entities, label, reg, is_float in (
        (
            CONF_ADDR_TOOL_ID,
            CONF_INVERTERS_MODBUS_ID,
            (CONF_ADDR_FROM, CONF_ADDR_TO, CONF_ADDR_CHANGE, CONF_ADDR_STATUS),
            "inverter",
            0x001E,  # Growatt: holding 30, plain integer, function 6
            False,
        ),
        (
            CONF_MADDR_TOOL_ID,
            CONF_METERS_MODBUS_ID,
            (CONF_MADDR_FROM, CONF_MADDR_TO, CONF_MADDR_CHANGE, CONF_MADDR_STATUS),
            "meter",
            0x0014,  # Eastron: holding 20-21, float32, function 16
            True,
        ),
    ):
        from_key, to_key, btn_key, status_key = entities
        if not any(k in config for k in entities):
            continue
        tool = cg.new_Pvariable(config[tool_key])
        await cg.register_component(tool, {})
        await modbus.register_modbus_client_device(
            tool,
            {
                modbus.CONF_MODBUS_ID: _bus_for(config, bus_key),
                CONF_ADDRESS: 0,
            },
        )
        cg.add(tool.set_label(label))
        # A literal, not the C++ constant: generated code runs in the global
        # setup(), where an unqualified name from the component namespace is
        # not visible.
        cg.add(tool.set_address_register(reg))
        cg.add(tool.set_float_format(is_float))
        for key, is_target in ((from_key, False), (to_key, True)):
            if key in config:
                num = await number.new_number(
                    config[key], min_value=0, max_value=254, step=1
                )
                cg.add(num.set_parent(tool))
                cg.add(num.set_is_target(is_target))
        if btn_key in config:
            btn = await button.new_button(config[btn_key])
            cg.add(btn.set_parent(tool))
        if status_key in config:
            ts = await text_sensor.new_text_sensor(config[status_key])
            cg.add(tool.set_status(ts))

    cg.add(hub.set_rebalancing(config[CONF_REBALANCING]))
    cg.add(hub.set_rebalance_threshold(config[CONF_REBALANCE_THRESHOLD]))

    if "controller_state" in config:
        ts = await text_sensor.new_text_sensor(config["controller_state"])
        cg.add(hub.set_controller_state(ts))

    if CONF_GRID_POWER_SENSOR_ID in config:
        bs = await cg.get_variable(config[CONF_GRID_POWER_SENSOR_ID])
        cg.add(hub.set_grid_power_source(bs))

    # Defaults are applied first; setup() then overrides them from flash if the
    # user has changed anything from the UI.
    for key, (field, default, lo, hi, step, _u, _i) in HUB_SETTINGS.items():
        cg.add(hub.set_default(field, default))
        if key in config:
            num = await number.new_number(
                config[key], min_value=lo, max_value=hi, step=step
            )
            cg.add(num.set_parent(hub))
            cg.add(num.set_field(field))
            cg.add(hub.set_setting_number(field, num))

    for key, setter in (
        ("meter_import", "set_meter_import"),
        ("meter_export", "set_meter_export"),
        ("meter_import_average", "set_meter_import_avg"),
        ("meter_export_average", "set_meter_export_avg"),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(hub, setter)(sens))

    if "meter_state" in config:
        ts = await text_sensor.new_text_sensor(config["meter_state"])
        cg.add(hub.set_meter_state(ts))

    for key, (setter, _icon, _dc) in HUB_BINARY_SENSORS.items():
        if key in config:
            bs = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(hub, setter)(bs))

    for i, conf in enumerate(config[CONF_INVERTERS]):
        inv = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(inv, _component_config(conf))
        # Every slot registers on the bus at 0 and stays silent there until
        # setup() restores a real address from flash, or the user types one.
        await modbus.register_modbus_client_device(
            inv,
            {
                modbus.CONF_MODBUS_ID: conf.get(modbus.CONF_MODBUS_ID)
                or _bus_for(config, CONF_INVERTERS_MODBUS_ID),
                CONF_ADDRESS: 0,
            },
        )

        cg.add(inv.set_update_interval(DEFAULT_INVERTER_INTERVAL))
        cg.add(inv.set_slow_interval(DEFAULT_SLOW_INTERVAL))
        cg.add(inv.set_slot_index(i))
        cg.add(inv.set_cfg_phases(conf[CONF_PHASES]))
        cg.add(inv.set_cfg_strings(conf[CONF_STRINGS]))
        if CONF_UPS in conf:
            cg.add(inv.set_cfg_ups(1 if conf[CONF_UPS] else 0))
        else:
            cg.add(inv.set_cfg_ups(AUTO))
        if CONF_BATTERY in conf:
            cg.add(inv.set_cfg_battery(1 if conf[CONF_BATTERY] else 0))
        else:
            cg.add(inv.set_cfg_battery(AUTO))

        cg.add(inv.set_module_voltage(conf[CONF_MODULE_VOLTAGE]))
        cg.add(inv.set_module_capacity(conf[CONF_MODULE_CAPACITY]))
        cg.add(inv.set_discharge_hours(conf[CONF_DISCHARGE_HOURS]))
        cg.add(inv.set_ups_avg_window(conf[CONF_UPS_AVG_WINDOW]))
        cg.add(inv.set_phase_detect_min_power(conf[CONF_PHASE_DETECT_MIN]))

        await _setup_triples(
            inv, conf, [CONF_PHASE_A, CONF_PHASE_B, CONF_PHASE_C],
            "set_phase_sensor",
        )
        await _setup_triples(inv, conf, PVS, "set_pv_sensor")
        await _setup_triples(inv, conf, UPS_PHASES, "set_ups_sensor")
        await _setup_indexed(inv, conf, LINE_VOLTAGES, "set_line_voltage")
        await _setup_indexed(inv, conf, PV_ENERGY_TODAY, "set_pv_energy_today")
        await _setup_indexed(inv, conf, PV_ENERGY_TOTAL, "set_pv_energy_total")

        for key, entry in SENSORS.items():
            if key in conf:
                sens = await sensor.new_sensor(conf[key])
                cg.add(getattr(inv, entry[0])(sens))

        for key, setter in TEXT_SENSORS.items():
            if key in conf:
                ts = await text_sensor.new_text_sensor(conf[key])
                cg.add(getattr(inv, setter)(ts))

        # ---- control surface ----
        for key, (field, lo, hi, step, _unit, _icon) in SETTING_NUMBERS.items():
            if key in conf:
                num = await number.new_number(
                    conf[key], min_value=lo, max_value=hi, step=step
                )
                cg.add(num.set_parent(inv))
                cg.add(num.set_field(field))
                cg.add(inv.set_setting_number(field, num))

        for key, (kind, lo, hi, step, _u, _i) in INVERTER_RATES.items():
            if key in conf:
                rnum = await number.new_number(
                    conf[key], min_value=lo, max_value=hi, step=step
                )
                cg.add(rnum.set_parent(inv))
                cg.add(rnum.set_kind(kind))
                cg.add(
                    getattr(
                        inv,
                        {
                            0: "set_min_rate_number",
                            1: "set_max_rate_number",
                            2: "set_update_number",
                            3: "set_slow_number",
                        }[kind],
                    )(rnum)
                )
        if "voltage_convention_select" in conf:
            csel = await select.new_select(
                conf["voltage_convention_select"], options=CONVENTIONS
            )
            cg.add(csel.set_parent(inv))
            cg.add(inv.set_convention_select(csel))
        for key, is_eeprom, setter in (
            ("auto_protection_limits_switch", False, "set_auto_protection_switch"),
            ("protect_eeprom_switch", True, "set_protect_eeprom_switch"),
        ):
            if key in conf:
                sw = await switch.new_switch(conf[key])
                cg.add(sw.set_parent(inv))
                cg.add(sw.set_is_eeprom(is_eeprom))
                cg.add(getattr(inv, setter)(sw))

        if CONF_SAFE_RATE in conf:
            snum = await number.new_number(
                conf[CONF_SAFE_RATE], min_value=0, max_value=100, step=1
            )
            cg.add(snum.set_parent(inv))
            cg.add(inv.set_safe_rate_number(snum))

        num = await number.new_number(
            conf[CONF_ADDRESS], min_value=0, max_value=254, step=1
        )
        cg.add(num.set_parent(inv))
        cg.add(inv.set_address_number(num))

        for key, options, setter in (
            (CONF_PHASE_COUNT_SELECT, PHASE_COUNT_OPTIONS, "set_phase_count_select"),
            (CONF_STRINGS_SELECT, STRING_OPTIONS, "set_strings_select"),
        ):
            if key in conf:
                sel = await select.new_select(conf[key], options=options)
                cg.add(sel.set_parent(inv))
                cg.add(getattr(inv, setter)(sel))

        if CONF_PHASE_SELECT in conf:
            sel = await select.new_select(conf[CONF_PHASE_SELECT],
                                          options=PHASE_OPTIONS)
            cg.add(sel.set_parent(inv))
            cg.add(inv.set_phase_select(sel))

        for key, addr, options in (
            (CONF_BATTERY_TYPE_SELECT, ADDR_BATTERY_TYPE, BATTERY_TYPE_OPTIONS),
            (CONF_EXPORT_LIMIT_SELECT, ADDR_EXPORT_LIMIT, EXPORT_LIMIT_OPTIONS),
        ):
            if key in conf:
                sel = await select.new_select(conf[key], options=options)
                cg.add(sel.set_parent(inv))
                cg.add(sel.set_address(addr))
                cg.add(inv.set_register_select(addr, sel))

        if CONF_AC_CHARGE in conf:
            sw = await switch.new_switch(conf[CONF_AC_CHARGE])
            cg.add(sw.set_parent(inv))
            cg.add(inv.set_ac_charge_switch(sw))

        for key, (addr, on_val, off_val, _icon) in REGISTER_SWITCHES.items():
            if key in conf:
                sw = await switch.new_switch(conf[key])
                cg.add(sw.set_parent(inv))
                cg.add(sw.set_registers(addr, on_val, off_val))
                cg.add(inv.set_register_switch(addr, on_val, sw))

        await _setup_windows(inv, conf, CONF_GRID_FIRST, MODE_GRID_FIRST)
        await _setup_windows(inv, conf, CONF_BATTERY_FIRST, MODE_BATTERY_FIRST)

        if CONF_REFRESH in conf:
            btn = await button.new_button(conf[CONF_REFRESH])
            cg.add(btn.set_parent(inv))
        if CONF_DUMP in conf:
            btn = await button.new_button(conf[CONF_DUMP])
            cg.add(btn.set_parent(inv))

        cg.add(hub.add_inverter(inv))

    # ------------------------------- meters -------------------------------
    for i, mconf in enumerate(config.get(CONF_METERS, [])):
        meter = cg.new_Pvariable(mconf[CONF_ID])
        await cg.register_component(meter, _component_config(mconf))
        await modbus.register_modbus_client_device(
            meter,
            {
                modbus.CONF_MODBUS_ID: mconf.get(modbus.CONF_MODBUS_ID)
                or _bus_for(config, CONF_METERS_MODBUS_ID),
                CONF_ADDRESS: 0,
            },
        )
        num = await number.new_number(
            mconf[CONF_ADDRESS], min_value=0, max_value=247, step=1
        )
        cg.add(num.set_parent(meter))
        cg.add(meter.set_address_number(num))
        cg.add(meter.set_update_interval(DEFAULT_METER_INTERVAL))
        cg.add(meter.set_slow_interval(DEFAULT_SLOW_INTERVAL))
        cg.add(meter.set_slot_index(i))

        for key, is_slow, setter in (
            (CONF_UPDATE_INTERVAL_NUM, False, "set_update_number"),
            (CONF_SLOW_UPDATE_INTERVAL, True, "set_slow_number"),
        ):
            if key in mconf:
                inum = await number.new_number(
                    mconf[key], min_value=1, max_value=3600, step=1
                )
                cg.add(inum.set_parent(meter))
                cg.add(inum.set_is_slow(is_slow))
                cg.add(getattr(meter, setter)(inum))

        if CONF_MODEL_SELECT in mconf:
            sel = await select.new_select(mconf[CONF_MODEL_SELECT],
                                          options=METER_MODELS)
            cg.add(sel.set_parent(meter))
            cg.add(meter.set_model_select(sel))

        if CONF_INFO in mconf:
            ts = await text_sensor.new_text_sensor(mconf[CONF_INFO])
            cg.add(meter.set_info_text_sensor(ts))

        for p, key in enumerate((CONF_PHASE_A, CONF_PHASE_B, CONF_PHASE_C)):
            if key not in mconf:
                continue
            group = mconf[key]
            for kind, ckey in METER_PHASE_KINDS:
                if ckey in group:
                    sens = await sensor.new_sensor(group[ckey])
                    cg.add(meter.set_phase_sensor(p, kind, sens))

        for idx, key in enumerate(LINE_VOLTAGES):
            if key in mconf:
                sens = await sensor.new_sensor(mconf[key])
                cg.add(meter.set_line_voltage(idx, sens))

        for key, entry in METER_SENSORS.items():
            if key in mconf:
                sens = await sensor.new_sensor(mconf[key])
                cg.add(getattr(meter, entry[0])(sens))

        cg.add(hub.add_meter(meter))
