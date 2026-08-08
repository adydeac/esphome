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

CODEOWNERS = ["@adydeac"]
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
# Bus level address change tool, unrelated to the inverters declared below.
CONF_ADDR_CHANGE = "change_inverter_address"
CONF_ADDR_FROM = "change_inverter_address_from"
CONF_ADDR_TO = "change_inverter_address_to"
CONF_ADDR_STATUS = "change_inverter_address_status"
# Output below which the per phase power registers cannot be judged.
CONF_PHASE_DETECT_MIN = "phase_power_detect_threshold"
# Counters and diagnostics use this cadence; update_interval stays fast enough
# for control decisions.
CONF_SLOW_UPDATE_INTERVAL = "slow_update_interval"

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
GrowattPhaseCountSelect = ns.class_("GrowattPhaseCountSelect", select.Select)
GrowattStringsSelect = ns.class_("GrowattStringsSelect", select.Select)
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
# Health timeouts, applied to inverters and meters alike: they share one bus,
# so there is no reason for them to disagree about what stalled means.
CONF_DEVICE_STALLED = "device_stalled_timeout"
CONF_DEVICE_OFFLINE = "device_offline_timeout"
CONF_OFFLINE_PROBE = "device_offline_probe_interval"
CONF_AVG_SAMPLES = "average_samples"

# ------------------------------ power controller ------------------------------
CONF_IMPORT_THRESHOLD = "import_threshold"
CONF_EXPORT_THRESHOLD = "export_threshold"
CONF_GAIN_UP = "increase_gain"
CONF_GAIN_DOWN = "decrease_gain"
CONF_MIN_STEP = "min_step"
CONF_MAX_STEP = "max_step"
CONF_STEP_INTERVAL = "step_interval"
CONF_REFRESH_INTERVAL = "refresh_interval"
CONF_STARTUP_RATE = "startup_power_rate"
CONF_OFFGRID_RATE = "offgrid_power_rate"
CONF_MIN_POWER_RATE = "min_power_rate"
CONF_MAX_POWER_RATE = "max_power_rate"

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
) = range(9)

CONF_VOLTAGE_SOFT_MARGIN = "grid_voltage_soft_margin"
CONF_PROTECTION_MARGIN = "inverter_protection_margin"
CONF_RESTART_DELAY = "inverter_restart_delay"
CONF_AUTO_PROTECTION = "auto_protection_limits"
CONF_REBALANCING = "phase_rebalancing"
CONF_REBALANCE_THRESHOLD = "rebalance_threshold"
CONF_VOLTAGE_CONVENTION = "voltage_convention"

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
CONF_PROTECT_EEPROM = "protect_eeprom"

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
            GrowattWindowSwitch, icon="mdi:calendar-check"
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
        # 0 or omitted = slot not present. The value stored in flash takes
        # precedence at boot, so this is only the first-boot default.
        # The address is an entity rather than a fixed value: it lives in
        # flash and is the one thing that has to be fixable from the UI, since
        # a slot pointed at the wrong address cannot be reached at all. New
        # devices start at 0, which every part of the component reads as
        # "this slot is empty" and never puts on the bus.
        cv.Required(CONF_ADDRESS): number.number_schema(
            GrowattInverterAddressNumber, icon="mdi:identifier"
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
        # Bounds for the power controller. Setting both to the same value takes
        # this inverter out of automatic control.
        cv.Optional(CONF_MIN_POWER_RATE, default=0): cv.int_range(min=0, max=100),
        cv.Optional(CONF_MAX_POWER_RATE, default=100): cv.int_range(min=0, max=100),
        # These are grid code protection registers. Writing them wrong can stop
        # the inverter connecting, or stop it disconnecting when it should, so
        # the automatic adjustment is opt in.
        cv.Optional(CONF_AUTO_PROTECTION, default=False): cv.boolean,
        # Which limits the inverter expects in its protection registers. "auto"
        # picks line whenever the unit populates its line to line registers,
        # which some models do while still reporting 230 V phase voltages.
        cv.Optional(CONF_VOLTAGE_CONVENTION, default="auto"): cv.enum(
            VOLTAGE_CONVENTIONS, lower=True
        ),
        cv.Optional(
            CONF_SLOW_UPDATE_INTERVAL, default="30s"
        ): cv.positive_time_period_milliseconds,
        # control surface
        cv.Optional(CONF_AC_CHARGE): switch.switch_schema(
            GrowattAcChargeSwitch, icon="mdi:transmission-tower-import"
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
        # Clears holding 2 at boot so the frequent power rate writes never
        # reach the EEPROM.
        cv.Optional(CONF_PROTECT_EEPROM, default=False): cv.boolean,
    }
    for key, (_addr, _on, _off, icon) in REGISTER_SWITCHES.items():
        schema[cv.Optional(key)] = switch.switch_schema(
            GrowattRegisterSwitch, icon=icon
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
    return cv.Schema(schema).extend(cv.polling_component_schema("10s"))


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
        # 0 or omitted = meter not present. Flash wins at boot.
        # Eastron specifies 1..247 for the whole SDM family, so there is
        # nothing above that to reach. 0 marks the slot empty.
        cv.Required(CONF_ADDRESS): number.number_schema(
            GrowattMeterAddressNumber, icon="mdi:identifier"
        ),
        # Override for phase count; "Auto" leaves detection in charge.
        cv.Optional(CONF_MODEL_SELECT): select.select_schema(
            GrowattMeterModelSelect, icon="mdi:meter-electric"
        ),
        cv.Optional(CONF_INFO): text_sensor.text_sensor_schema(),
        cv.Optional(
            CONF_SLOW_UPDATE_INTERVAL, default="30s"
        ): cv.positive_time_period_milliseconds,
    }
    for key in (CONF_PHASE_A, CONF_PHASE_B, CONF_PHASE_C):
        schema[cv.Optional(key)] = _meter_phase_schema()
    for key in LINE_VOLTAGES:
        schema[cv.Optional(key)] = _volt()
    for key, entry in METER_SENSORS.items():
        schema[cv.Optional(key)] = _schema_from(entry)
    return cv.Schema(schema).extend(cv.polling_component_schema("10s"))


def _validate(config):
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
            cv.GenerateID(modbus.CONF_MODBUS_ID): cv.use_id(modbus.Modbus),
            cv.Required(CONF_MAX_INVERTERS): cv.int_range(min=1, max=32),
            # Commissioning tool: writes holding 30 at whatever address is
            # entered, whether or not that unit is declared under 'inverters'.
            # Both fields default to 0, the Modbus broadcast address, so the
            # button does nothing until they are filled in.
            cv.Optional(CONF_ADDR_FROM): number.number_schema(
                GrowattAddressNumber, icon="mdi:import"
            ),
            cv.Optional(CONF_ADDR_TO): number.number_schema(
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
            cv.Optional(
                CONF_DEVICE_STALLED, default="10s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_DEVICE_OFFLINE, default="20s"
            ): cv.positive_time_period_milliseconds,
            # An inverter without storage shuts down when the panels go dark.
            # Querying it then costs more bus time in timeouts than a whole
            # valid cycle, so it is only checked this often until it answers.
            cv.Optional(
                CONF_OFFLINE_PROBE, default="60s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_AVG_SAMPLES, default=60): cv.int_range(min=1, max=60),
            # controller tuning
            cv.Optional(CONF_IMPORT_THRESHOLD, default=100): cv.float_range(min=0),
            cv.Optional(CONF_EXPORT_THRESHOLD, default=0): cv.float_range(min=0),
            # Fraction of the measured deviation each step tries to close.
            # Lower when raising: overshooting there means exporting, which
            # costs money, while overshooting downwards only means importing a
            # little longer.
            cv.Optional(CONF_GAIN_UP, default=0.5): cv.float_range(
                min=0.05, max=1.0
            ),
            cv.Optional(CONF_GAIN_DOWN, default=0.8): cv.float_range(
                min=0.05, max=1.0
            ),
            # The active power register takes whole percent, so anything below
            # one would round away and waste the cycle.
            cv.Optional(CONF_MIN_STEP, default=1): cv.float_range(min=1, max=50),
            cv.Optional(CONF_MAX_STEP, default=20): cv.float_range(min=1, max=100),
            cv.Optional(
                CONF_STEP_INTERVAL, default="6s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_REFRESH_INTERVAL, default="60s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_STARTUP_RATE, default=0): cv.int_range(min=0, max=100),
            cv.Optional(CONF_OFFGRID_RATE, default=100): cv.int_range(min=0, max=100),
            # How far outside the controller's own window each inverter's trip
            # thresholds are pushed, so there is room to react before the
            # hardware disconnects and locks us out for minutes.
            # Volts below the limit at which increases stop being
            # proportional and start creeping by min_step instead.
            cv.Optional(CONF_VOLTAGE_SOFT_MARGIN, default=8): cv.float_range(
                min=0, max=50
            ),
            cv.Optional(CONF_PROTECTION_MARGIN, default=10): cv.float_range(
                min=0, max=50
            ),
            cv.Optional(CONF_RESTART_DELAY, default=30): cv.int_range(
                min=0, max=600
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
    ).extend(cv.polling_component_schema("2s")),
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
    await cg.register_component(hub, config)
    # The hub only takes a place on the bus when the address tool is actually
    # configured. It sits at 0, the broadcast address, so it never matches an
    # incoming frame except while the tool is running and has pointed it
    # somewhere - but a device that is not registered cannot interfere at all,
    # which is worth more than the convenience of registering unconditionally.
    addr_tool = any(
        k in config
        for k in (CONF_ADDR_CHANGE, CONF_ADDR_FROM, CONF_ADDR_TO, CONF_ADDR_STATUS)
    )
    if addr_tool:
        await modbus.register_modbus_client_device(
            hub,
            {
                modbus.CONF_MODBUS_ID: config[modbus.CONF_MODBUS_ID],
                CONF_ADDRESS: 0,
            },
        )
    cg.add(hub.set_max_inverters(config[CONF_MAX_INVERTERS]))

    for key, is_target in ((CONF_ADDR_FROM, False), (CONF_ADDR_TO, True)):
        if key in config:
            num = await number.new_number(
                config[key], min_value=0, max_value=254, step=1
            )
            cg.add(num.set_parent(hub))
            cg.add(num.set_is_target(is_target))
    if CONF_ADDR_CHANGE in config:
        btn = await button.new_button(config[CONF_ADDR_CHANGE])
        cg.add(btn.set_parent(hub))
    if CONF_ADDR_STATUS in config:
        ts = await text_sensor.new_text_sensor(config[CONF_ADDR_STATUS])
        cg.add(hub.set_addr_status(ts))
    cg.add(hub.set_stalled_timeout(config[CONF_DEVICE_STALLED]))
    cg.add(hub.set_offline_timeout(config[CONF_DEVICE_OFFLINE]))
    cg.add(hub.set_offline_probe_interval(config[CONF_OFFLINE_PROBE]))
    cg.add(hub.set_avg_window(config[CONF_AVG_SAMPLES]))
    cg.add(hub.set_import_threshold(config[CONF_IMPORT_THRESHOLD]))
    cg.add(hub.set_export_threshold(config[CONF_EXPORT_THRESHOLD]))
    cg.add(hub.set_increase_gain(config[CONF_GAIN_UP]))
    cg.add(hub.set_decrease_gain(config[CONF_GAIN_DOWN]))
    cg.add(hub.set_min_step(config[CONF_MIN_STEP]))
    cg.add(hub.set_max_step(config[CONF_MAX_STEP]))
    cg.add(hub.set_step_interval(config[CONF_STEP_INTERVAL]))
    cg.add(hub.set_refresh_interval(config[CONF_REFRESH_INTERVAL]))
    cg.add(hub.set_startup_rate(config[CONF_STARTUP_RATE]))
    cg.add(hub.set_offgrid_rate(config[CONF_OFFGRID_RATE]))
    cg.add(hub.set_voltage_soft_margin(config[CONF_VOLTAGE_SOFT_MARGIN]))
    cg.add(hub.set_protection_margin(config[CONF_PROTECTION_MARGIN]))
    cg.add(hub.set_restart_delay(config[CONF_RESTART_DELAY]))
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
        await cg.register_component(inv, conf)
        # Every slot registers on the bus at 0 and stays silent there until
        # setup() restores a real address from flash, or the user types one.
        await modbus.register_modbus_client_device(
            inv,
            {
                modbus.CONF_MODBUS_ID: config[modbus.CONF_MODBUS_ID],
                CONF_ADDRESS: 0,
            },
        )

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
        cg.add(inv.set_slow_interval(conf[CONF_SLOW_UPDATE_INTERVAL]))
        cg.add(inv.set_min_power_rate(conf[CONF_MIN_POWER_RATE]))
        cg.add(inv.set_max_power_rate(conf[CONF_MAX_POWER_RATE]))
        cg.add(inv.set_auto_protection(conf[CONF_AUTO_PROTECTION]))
        cg.add(inv.set_voltage_convention(conf[CONF_VOLTAGE_CONVENTION]))

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

        cg.add(inv.set_protect_eeprom(conf[CONF_PROTECT_EEPROM]))
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
        await cg.register_component(meter, mconf)
        await modbus.register_modbus_client_device(
            meter,
            {
                modbus.CONF_MODBUS_ID: config[modbus.CONF_MODBUS_ID],
                CONF_ADDRESS: 0,
            },
        )
        num = await number.new_number(
            mconf[CONF_ADDRESS], min_value=0, max_value=247, step=1
        )
        cg.add(num.set_parent(meter))
        cg.add(meter.set_address_number(num))
        cg.add(meter.set_slot_index(i))
        cg.add(meter.set_slow_interval(mconf[CONF_SLOW_UPDATE_INTERVAL]))

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
