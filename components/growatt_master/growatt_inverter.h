#pragma once

#include "growatt_master.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/button/button.h"
#include "esphome/components/select/select.h"
#include <cmath>

namespace esphome {
namespace growatt_master {

// ---------------------------------------------------------------------------
// Register map notes
//
// Growatt firmware answers non-existent registers with ZEROS instead of a
// Modbus exception, so "did it answer" is never a usable capability test -
// every probe must inspect the returned data.
//
// Holding register 44 (TP) is NOT reliable across families:
//   MIN 6000TL-X    DTC 5100  TP 0x0201  (2 MPPT, 1 phase)   correct
//   SPH 10000TL3    DTC 3601  TP 0x00C9  (garbage)           wrong
// so phases and strings are derived from live measurements instead.
//
// The protocol document lists input range 0..124 only for TL3-X and Storage,
// but MIN units answer it too, so the first group is treated as universal.
// 125 registers is the maximum a single read command may request.
// ---------------------------------------------------------------------------

static const uint8_t MAX_STRINGS = 8;
static const uint16_t VOLTAGE_PRESENT = 100;  // 10.0 V, rejects ADC noise
static const uint8_t FIRST_GROUP_CNT = 125;

// Thresholds for judging whether the per phase power registers really carry a
// per phase figure. The current threshold is what tells a phase that is idle
// apart from one that is working but not reported.
static const float PHASE_POWER_PRESENT_W = 1.0f;
static const float PHASE_CURRENT_PRESENT_A = 0.1f;
// Above this an AC voltage reading can only be line to line: no grid runs
// phase to neutral this high, and no line to line voltage sits this low.
static const float LINE_VOLTAGE_FLOOR = 300.0f;
// How close Pac1 has to sit to the total AC output before it is accepted as
// being the total rather than one phase of it.
static const float PAC_TOTAL_TOLERANCE = 0.1f;
// Consecutive cycles that must agree before the conclusion is latched. A unit
// pushing mostly reactive current at first light can look like the real thing
// for a moment; it cannot keep looking like it.
static const uint8_t PAC_TOTAL_CONFIRMATIONS = 3;

// ------------------------- input registers 0..124 -------------------------
static const uint16_t IN_BASE = 0;
static const uint8_t IN_STATUS = 0;
static const uint8_t IN_PV_POWER = 1;          // DWORD 0.1 W
static const uint8_t IN_VPV_FIRST = 3;         // Vpv, Ipv, Ppv(DWORD) per string
static const uint8_t IN_VPV_STEP = 4;
static const uint8_t IN_AC_POWER = 35;         // DWORD 0.1 W
static const uint8_t IN_FREQUENCY = 37;        // 0.01 Hz
static const uint8_t IN_VAC[3] = {38, 42, 46};
static const uint8_t IN_LINE_VOLT = 50;        // 50..52 RS, ST, TR
static const uint8_t IN_E_TODAY = 53;          // DWORD 0.1 kWh
static const uint8_t IN_E_TOTAL = 55;
static const uint8_t IN_WORK_TIME = 57;        // DWORD 0.5 s
static const uint8_t IN_EPV_TODAY_FIRST = 59;  // per string, step 4
static const uint8_t IN_EPV_TOTAL_FIRST = 61;
static const uint8_t IN_EPV_STEP = 4;
static const uint8_t IN_EPV_TOTAL_ALL = 91;
static const uint8_t IN_TEMP = 93;
static const uint8_t IN_TEMP_IPM = 94;
static const uint8_t IN_TEMP_BOOST = 95;
static const uint8_t IN_BAT_VOLT_DSP = 97;
static const uint8_t IN_BUS_P = 98;
static const uint8_t IN_BUS_N = 99;
static const uint8_t IN_OUTPUT_PF = 100;       // raw 0..20000, 10000 = unity
static const uint8_t IN_REAL_PERCENT = 101;    // doc says 1 %, see note in .cpp
static const uint8_t IN_MAX_POWER = 102;       // DWORD 0.1 W
static const uint8_t IN_DERATING = 104;
static const uint8_t IN_FAULT_MAIN = 105;
static const uint8_t IN_FAULT_SUB = 107;
static const uint8_t IN_WARN_BITS = 110;
static const uint8_t IN_WARN_SUB = 111;
// 112..115 mean different things per family: AC charge energy on Storage,
// warning and fault codes on MAX. Interpreted according to has_storage.
static const uint8_t IN_EACHARGE_TODAY = 112;
static const uint8_t IN_EACHARGE_TOTAL = 114;
static const uint8_t IN_WARN_MAIN = 112;
static const uint8_t IN_AC_CHARGE_POWER = 116;
static const uint8_t IN_PRIORITY = 118;
static const uint8_t IN_BATTERY_TYPE = 119;

// ------------------------ holding registers 0..124 ------------------------
static const uint16_t HOLD_BASE = 0;
static const uint8_t HO_ONOFF = 0;        // 1 inverter on, 0 off, 3 BDC on, 2 off
static const uint8_t HO_PF_CMD_MEMORY = 2;  // 1 = registers 3,4,5,99 persist
static const uint8_t HO_ACTIVE_RATE = 3;
static const uint8_t HO_REACTIVE_RATE = 4;
static const uint8_t HO_PF_SET = 5;
static const uint8_t HO_NORMAL_POWER = 6;      // DWORD 0.1 VA
static const uint8_t HO_VNORMAL = 8;
static const uint8_t HO_FIRMWARE = 9;          // 9..14 ASCII
static const uint8_t HO_FIRMWARE_CNT = 6;
static const uint8_t HO_PV_START_VOLT = 17;
static const uint8_t HO_START_TIME = 18;
static const uint8_t HO_RESTART_DELAY = 19;
static const uint8_t HO_SERIAL = 23;           // 23..27 ASCII
static const uint8_t HO_SERIAL_CNT = 5;
static const uint8_t HO_COM_ADDRESS = 30;
static const uint8_t HO_MANUFACTURER = 34;     // 34..41 ASCII
static const uint8_t HO_MANUFACTURER_CNT = 8;
static const uint8_t HO_DTC = 43;
static const uint8_t HO_TP = 44;
static const uint8_t HO_SYS_TIME = 45;         // 45..50 year..second
static const uint8_t HO_GRID_V_LOW = 52;
static const uint8_t HO_GRID_V_HIGH = 53;
static const uint8_t HO_GRID_F_LOW = 54;
static const uint8_t HO_GRID_F_HIGH = 55;
static const uint8_t HO_FW_BUILD = 82;         // 82..87 ASCII
static const uint8_t HO_FW_BUILD_CNT = 6;
static const uint8_t HO_MODBUS_VER = 88;
static const uint8_t HO_PF_MODEL = 89;
static const uint8_t HO_EXPORT_LIMIT_EN = 122;
static const uint8_t HO_EXPORT_LIMIT_RATE = 123;
static const uint8_t HO_TRACKER_MODEL = 124;

// Model strings, holding 125..136. Present on MIN, all zeros on SPH.
static const uint16_t REG_TYPE_BASE = 125;
static const uint8_t REG_TYPE_CNT = 12;
static const uint8_t OFF_INV_TYPE = 0;         // 125..132
static const uint8_t OFF_INV_TYPE_CNT = 8;
static const uint8_t OFF_BOOTLOADER = 8;       // 133..136
static const uint8_t OFF_BOOTLOADER_CNT = 4;

static const uint16_t REG_PVSTRSCAN = 183;     // 183..185
static const uint8_t REG_CAPS_CNT = 3;

// Storage probe, holding 1000..1060. 1000..1007 hold battery configuration
// with non-zero factory defaults on MIX/SPA/SPH; 1060 is BuckUpsFunEn.
static const uint16_t REG_STORAGE_BASE = 1000;
static const uint8_t REG_STORAGE_CNT = 61;
static const uint8_t REG_STORAGE_CHECK = 8;
static const uint8_t REG_UPS_OFFSET = 60;

// Battery presence, input 1013 (Vbat) and 1014 (SOC). PackNum (holding 185)
// reads 0 even on units with a working battery, so only live values are used.
static const uint16_t REG_BAT_BASE = 1013;
static const uint8_t REG_BAT_CNT = 2;

// ----------------------- storage input 1000..1096 -----------------------
static const uint8_t ST_WORK_MODE = 0;
static const uint8_t ST_FAULT_WORD = 1;
static const uint8_t ST_DISCHARGE_POWER = 9;   // DWORD
static const uint8_t ST_CHARGE_POWER = 11;     // DWORD
static const uint8_t ST_BAT_VOLT = 13;
static const uint8_t ST_BAT_SOC = 14;
static const uint8_t ST_P_TO_USER = 21;        // DWORD total
static const uint8_t ST_P_TO_GRID = 29;        // DWORD total
static const uint8_t ST_P_LOCAL_LOAD = 37;     // DWORD total
static const uint8_t ST_BAT_TEMP = 40;
static const uint8_t ST_E_TO_USER_TODAY = 44;
static const uint8_t ST_E_TO_USER_TOTAL = 46;
static const uint8_t ST_E_TO_GRID_TODAY = 48;
static const uint8_t ST_E_TO_GRID_TOTAL = 50;
static const uint8_t ST_E_DISCHARGE_TODAY = 52;
static const uint8_t ST_E_DISCHARGE_TOTAL = 54;
static const uint8_t ST_E_CHARGE_TODAY = 56;
static const uint8_t ST_E_CHARGE_TOTAL = 58;
static const uint8_t ST_E_LOAD_TODAY = 60;
static const uint8_t ST_E_LOAD_TOTAL = 62;
static const uint8_t ST_UPS_FREQ = 67;
static const uint8_t ST_UPS_VAC_FIRST = 68;    // V, I, P(DWORD) per phase
static const uint8_t ST_UPS_STEP = 4;
static const uint8_t ST_UPS_LOAD = 80;
static const uint8_t ST_UPS_PF = 81;
static const uint8_t ST_BMS_SOC = 86;
static const uint8_t ST_BMS_VOLT = 87;
static const uint8_t ST_BMS_CURR = 88;
static const uint8_t ST_BMS_TEMP = 89;
static const uint8_t ST_BAT_CAPACITY = 90;
static const uint8_t ST_BAT_CYCLES = 95;
static const uint8_t ST_BAT_HEALTH = 96;

// ---------------------------- polling blocks ----------------------------
// Split into a fast cycle that feeds control decisions and a slow cycle for
// counters and diagnostics. At 9600 baud a response costs about
// (3 + 2 * registers + 2) * 1.04 ms, so the fast blocks are kept small enough
// that several devices still fit inside a two second window.
static const uint16_t POLL_FAST_MAIN_BASE = 0;
static const uint8_t POLL_FAST_MAIN_CNT = 57;   // status, PV, phases, energy
// Small but control relevant: real output percent tells us whether the
// inverter is actually following our setpoint, and derating mode tells us why
// it is not. Without these the controller keeps pushing an inverter that is
// already limited by sun, temperature or grid frequency.
static const uint16_t POLL_FAST_STATUS_BASE = 101;
static const uint8_t POLL_FAST_STATUS_CNT = 5;  // 101..105, about 20 ms
static const uint16_t POLL_SLOW_MAIN_BASE = 57;
static const uint8_t POLL_SLOW_MAIN_CNT = 68;   // 57..124
static const uint16_t POLL_FAST_BAT_BASE = 1009;
static const uint8_t POLL_FAST_BAT_CNT = 6;     // charge/discharge, V, SOC
static const uint16_t POLL_FAST_UPS_BASE = 1067;
static const uint8_t POLL_FAST_UPS_CNT = 15;    // UPS phases, load, PF
static const uint16_t POLL_SLOW_STOR_BASE = 1000;
static const uint8_t POLL_SLOW_STOR_CNT = 97;

// ---------------------------- writable registers ----------------------------
static const uint16_t REG_ACTIVE_POWER_RATE = 3;  // holding, 0-100 percent

// Independent settings, written one at a time with function 0x06.
static const uint16_t HO_GF_DISCHARGE_RATE = 1070;
static const uint16_t HO_GF_STOP_SOC = 1071;
static const uint16_t HO_BF_CHARGE_RATE = 1090;
static const uint16_t HO_BF_STOP_SOC = 1091;
static const uint16_t HO_BF_AC_CHARGE = 1092;
//
// Time windows. The firmware only accepts a mode change when the three
// periods and their enable flags arrive together, so these two blocks must be
// written atomically with function 0x10 (Preset Multiple Registers).
// Layout per block: start1, stop1, enable1, start2, stop2, enable2, ...
// Times are encoded as (hour << 8) | minute and may wrap past midnight.
static const uint16_t HO_GF_WINDOW_BASE = 1080;
static const uint16_t HO_BF_WINDOW_BASE = 1100;
static const uint8_t PERIOD_COUNT = 3;
static const uint8_t WINDOW_REGS = 9;  // 3 periods x (start, stop, enable)
// Settings and both window blocks in one read: 1070..1108
static const uint16_t HO_SETTINGS_BASE = 1070;
static const uint8_t HO_SETTINGS_CNT = 39;

enum WindowMode : uint8_t {
  MODE_GRID_FIRST = 0,
  MODE_BATTERY_FIRST = 1,
  MODE_COUNT = 2,
};

// Parts of a time window addressable from the UI
enum WindowPart : uint8_t {
  PART_START_HOUR = 0,
  PART_START_MIN = 1,
  PART_STOP_HOUR = 2,
  PART_STOP_MIN = 3,
  PART_COUNT = 4,
};

// Independent settings addressable from the UI. Each entry has an address and
// a scale in SETTING_ADDR / SETTING_SCALE in the .cpp, so the UI works in
// engineering units while the register keeps its raw encoding.
enum SettingField : uint8_t {
  SET_ACTIVE_POWER_RATE = 0,  // holding 3,    %
  SET_GF_DISCHARGE_RATE,      // holding 1070, %
  SET_GF_STOP_SOC,            // holding 1071, %
  SET_BF_CHARGE_RATE,         // holding 1090, %
  SET_BF_STOP_SOC,            // holding 1091, %
  SET_PV_START_VOLTAGE,       // holding 17,   0.1 V
  SET_START_TIME,             // holding 18,   s
  SET_RESTART_DELAY,          // holding 19,   s
  SET_GRID_V_LOW,             // holding 52,   0.1 V
  SET_GRID_V_HIGH,            // holding 53,   0.1 V
  SET_GRID_F_LOW,             // holding 54,   0.01 Hz
  SET_GRID_F_HIGH,            // holding 55,   0.01 Hz
  SET_EXPORT_LIMIT_RATE,      // holding 123,  0.1 %
  SET_COUNT,
};

// Which mains phase a single phase inverter is wired to. The inverter cannot
// know this - it only sees "a" voltage on Vac1 - so it stays manual config.
enum InvPhase : uint8_t {
  INV_PHASE_L1 = 0,
  INV_PHASE_L2 = 1,
  INV_PHASE_L3 = 2,
};

struct TimeWindow {
  uint8_t start_h{0};
  uint8_t start_m{0};
  uint8_t stop_h{0};
  uint8_t stop_m{0};
  bool enabled{false};
};

static const uint8_t WRITE_QUEUE_SIZE = 4;

struct PendingWrite {
  uint8_t function;
  uint16_t address;
  uint8_t count;
  uint16_t values[WINDOW_REGS];
};

static const int8_t CFG_AUTO = -1;
static const uint8_t UPS_AVG_MAX = 60;

// --------------------------- register dump ---------------------------
struct DumpRange {
  uint8_t function;
  uint16_t start;
  uint16_t count;
};
static const DumpRange DUMP_RANGES[] = {
    {CMD_READ_HOLDING, 0, 125},    {CMD_READ_INPUT, 0, 125},
    {CMD_READ_HOLDING, 1000, 125}, {CMD_READ_INPUT, 1000, 125},
    {CMD_READ_HOLDING, 3000, 125}, {CMD_READ_INPUT, 3000, 125},
    // The VPP protocol is a separate Growatt document from the RTU one and
    // lives in its own address space. Two areas are worth having: 30000-30124
    // carries the reference power (30026) and the control authority flag
    // (30100), and 30400-30524 carries the remote control trio (30407 enable,
    // 30408 duration in minutes, 30409 signed percent) plus the applied value
    // read back at 30474.
    //
    // A range that is not implemented answers with an exception and the whole
    // range is skipped, so probing costs nothing on a unit without VPP.
    {CMD_READ_HOLDING, 30000, 125}, {CMD_READ_INPUT, 30000, 125},
    {CMD_READ_HOLDING, 30400, 125}, {CMD_READ_INPUT, 30400, 125},
};
static const uint8_t DUMP_RANGE_COUNT = sizeof(DUMP_RANGES) / sizeof(DumpRange);
static const uint8_t DUMP_CHUNK = 25;

enum IdentStep : uint8_t {
  IDENT_START = 0,
  IDENT_LIVE,      // input 0..124      phases, strings
  IDENT_INFO,      // holding 0..124    nameplate, firmware, serial, DTC
  IDENT_TYPE,      // holding 125..136  model strings
  IDENT_CAPS,      // holding 183..185
  IDENT_STORAGE,   // holding 1000..1060
  IDENT_BATTERY,   // input 1013..1014
  IDENT_SETTINGS,  // holding 1070..1108, storage only
  IDENT_DONE,
};

enum PollBlock : uint8_t {
  POLL_IDLE = 0,
  POLL_FAST_MAIN,    // input 0..56
  POLL_FAST_STATUS,  // input 101..105
  POLL_FAST_BAT,     // input 1009..1014, storage only
  POLL_FAST_UPS,     // input 1067..1081, storage only
  POLL_SLOW_MAIN,    // input 57..124
  POLL_SLOW_STOR,    // input 1000..1096, storage only
};

// Which convention an inverter uses for its own protection thresholds. Some
// three phase units report both phase and line voltages but expect the trip
// limits in line terms, so magnitude alone is not a safe test.
enum VoltageConvention : uint8_t {
  CONV_AUTO = 0,
  CONV_PHASE,
  CONV_LINE,
  CONV_MODE_COUNT,
};

// Health of one inverter. Some models shut down completely when the panels go
// dark, and querying a unit that is not there wastes more bus time in timeouts
// than every valid read put together.
enum InvHealth : uint8_t {
  INV_ONLINE = 0,
  INV_STALLED,
  INV_OFFLINE,
};

// Cheapest possible read, used to find out whether an offline unit is back.
static const uint16_t PROBE_BASE = 0;
static const uint8_t PROBE_CNT = 1;

struct GrowattCaps {
  uint16_t dtc{0};
  uint8_t phases{0};
  uint8_t strings{0};    // PV inputs actually carrying voltage
  uint8_t trackers{0};   // MPPT inputs the hardware has, from TP
  bool has_storage{false};
  bool has_ups{false};
  bool has_battery{false};
  uint16_t battery_soc{0};
  uint8_t battery_packs{0};
  uint8_t bdc_count{0};
  std::string inv_type;
  std::string serial;
};

// Persisted per slot. The component is the single source of truth for these,
// so UI entities read them back instead of keeping their own copy.
// The YAML values act as defaults for the very first boot only.
// See PREFS_VERSION in growatt_master.h for the versioning and reserved space
// convention these structures follow.
struct GrowattSlotPrefs {
  uint8_t version;
  uint8_t address;
  int8_t cfg_phases;
  int8_t cfg_strings;
  uint8_t phase;  // InvPhase, single phase inverters only
  uint8_t safe_power_rate;
  uint8_t min_power_rate;
  uint8_t max_power_rate;
  uint8_t convention;         // ConvMode
  uint8_t auto_protection;
  uint8_t protect_eeprom;
  uint16_t update_interval;   // seconds
  uint16_t slow_interval;     // seconds
  uint8_t reserved[12];
} __attribute__((packed));

// Voltage / current / power triple, reused for grid phases, PV strings and
// UPS outputs.
struct SensorTriple {
  sensor::Sensor *voltage{nullptr};
  sensor::Sensor *current{nullptr};
  sensor::Sensor *power{nullptr};
};

// One read callback for both register tables. The identification sequence
// alternates between them - input registers for the live block, holding for the
// info block - so a split into on_read_input_registers()/on_read_holding_
// registers() would put two doors on one state machine while the step being
// served already records which table was asked for.
class GrowattInverter : public PollingComponent, public modbus::ModbusClientDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Data and exceptions arrive together, the outcome in status. Handling an
  // exception explicitly avoids waiting out the full timeout on registers the
  // model does not implement.
  void on_read_registers(modbus::EntityType entity_type, uint16_t start_address,
                         std::span<const uint16_t> data,
                         modbus::ResponseStatus status) override;
  // Write acknowledgements have their own callbacks, which is what keeps a
  // rejected write out of the identification state machine structurally rather
  // than by remembering to check writing_ before anything else.
  void on_write_single_register(uint16_t address, uint16_t value,
                                modbus::ResponseStatus status) override;
  void on_write_multiple_registers(uint16_t start_address,
                                   std::span<const uint16_t> registers,
                                   modbus::ResponseStatus status) override;
  // Replies that do not match the request's shape land here instead of being
  // delivered short; see the definition.
  void on_custom_response(std::span<const uint8_t> request_pdu,
                          std::span<const uint8_t> response_pdu,
                          modbus::ResponseStatus status) override;

  // ------------------ public API, callable from YAML lambdas ------------------
  void change_address(uint8_t addr);
  uint8_t get_address() const { return this->address_; }

  void set_address_number(number::Number *n) { this->address_num_ = n; }
  void set_phase_count_select(select::Select *s) { this->phase_count_select_ = s; }
  void set_strings_select(select::Select *s) { this->strings_select_ = s; }
  void set_cfg_phases(int8_t v);
  int8_t get_cfg_phases() const { return this->cfg_phases_; }
  void set_cfg_strings(int8_t v);
  int8_t get_cfg_strings() const { return this->cfg_strings_; }
  void set_cfg_ups(int8_t v) { this->cfg_ups_ = v; }
  void set_cfg_battery(int8_t v) { this->cfg_battery_ = v; }

  void restart_identification();
  void start_dump();
  // ------------------------------ write API ------------------------------
  // Independent settings go out immediately as function 0x06. Values are in
  // engineering units; the scale table converts to the raw register value.
  void set_setting(uint8_t field, float value);
  float get_setting(uint8_t field) const;
  void set_ac_charge(bool on);
  bool get_ac_charge() const { return this->ac_charge_; }

  // Which mains phase this inverter feeds. Only meaningful when single phase.
  void set_phase(uint8_t p);
  uint8_t get_phase() const { return this->phase_; }

  // Writes a single holding register directly, used by the register backed
  // selects (battery type, export limit mode).
  void write_register(uint16_t address, uint16_t value);
  void set_register_select(uint16_t address, select::Select *s);
  void set_register_switch(uint16_t address, uint16_t on_value,
                           switch_::Switch *s);
  // When set, the component clears holding 2 at identification. With setting
  // memory off the power rate writes stay volatile, which both spares the
  // EEPROM and lets the inverter come back at full power if the controller
  // ever stops running.

  // Window edits are staged in memory. Nothing reaches the inverter until
  // apply_windows(), because the firmware needs the whole block at once.
  void set_window_part(uint8_t mode, uint8_t period, uint8_t part, uint8_t v);
  uint8_t get_window_part(uint8_t mode, uint8_t period, uint8_t part) const;
  void set_window_enabled(uint8_t mode, uint8_t period, bool on);
  bool get_window_enabled(uint8_t mode, uint8_t period) const;
  // Returns false and logs the reason when validation fails.
  bool apply_windows(uint8_t mode);
  bool windows_overlap(std::string *reason = nullptr) const;

  // Entity back references so the component can push freshly read values.
  void set_window_number(uint8_t mode, uint8_t period, uint8_t part,
                         number::Number *n);
  void set_window_switch(uint8_t mode, uint8_t period, switch_::Switch *s);
  void set_setting_number(uint8_t field, number::Number *n);
  void set_ac_charge_switch(switch_::Switch *s) { this->ac_charge_sw_ = s; }
  void set_phase_select(select::Select *s) { this->phase_select_ = s; }

  void set_slot_index(uint8_t i) { this->slot_index_ = i; }

  bool is_enabled() const { return this->address_ != 0; }
  bool ident_done() const { return this->step_ == IDENT_DONE; }
  const GrowattCaps &caps() const { return this->caps_; }
  uint8_t get_phases() const { return this->caps_.phases; }
  uint8_t get_strings() const { return this->caps_.strings; }
  uint8_t get_trackers() const { return this->caps_.trackers; }
  bool get_has_storage() const { return this->caps_.has_storage; }
  bool get_has_battery() const { return this->caps_.has_battery; }
  bool get_has_ups() const { return this->caps_.has_ups; }

  // micros() timestamp of the last frame received. Wraps every ~71 min.
  uint32_t get_last_update() const { return this->last_update_; }
  bool is_stale(uint32_t timeout_us) const {
    return this->last_update_ == 0 || (micros() - this->last_update_) > timeout_us;
  }

  // Health, and the timeouts that define it. The hub passes down the same
  // values it uses for the meter: the same question, asked once.
  uint8_t health() const { return this->health_; }
  bool is_online() const { return this->health_ == INV_ONLINE; }
  const char *health_text() const;

  /// The hub's per cycle view of this slot, in the same words it logs. Built by
  /// the hub because it is the hub that knows the headroom figure; published
  /// here so the string lands on the inverter's own device in Home Assistant
  /// rather than on the hub with a slot number to decode.
  void publish_control_summary(const char *s);
  void set_health_timeouts(uint32_t stalled_ms, uint32_t offline_ms) {
    this->stalled_ms_ = stalled_ms;
    this->offline_ms_ = offline_ms;
  }
  void set_offline_probe_interval(uint32_t ms) { this->offline_probe_ms_ = ms; }

  // Live values the hub needs for its threshold conditions. NaN until the
  // matching block has been read at least once.
  float get_ups_load() const { return this->ups_load_pct_; }
  float get_ups_load_avg() const { return this->ups_load_avg_pct_; }
  float get_battery_soc() const { return this->battery_soc_pct_; }
  float get_grid_power() const { return this->grid_power_w_; }

  void set_power_percent(uint8_t p) { this->power_percent_ = p > 100 ? 100 : p; }
  uint8_t get_power_percent() const { return this->power_percent_; }

  // ------------------------------ controller API ------------------------------
  // Bounds the controller must stay inside. Setting both to the same value
  // effectively takes this inverter out of automatic control.
  // Where this unit is parked when the meter has been gone long enough that
  // holding the last setpoint stops being prudent. Editable, and persisted
  // under its own preference key so adding it cannot invalidate the stored
  // addresses.
  void set_safe_power_rate(uint8_t v) { this->safe_power_rate_ = v; }
  uint8_t get_safe_power_rate() const { return this->safe_power_rate_; }
  void apply_safe_power_rate(float v);
  void set_safe_rate_number(number::Number *n) { this->safe_rate_num_ = n; }
  // Runtime edits. Each stores, persists, and where it matters takes effect at
  // once rather than at the next identification.
  void apply_min_power_rate(float v);
  void apply_max_power_rate(float v);
  void apply_update_interval(float seconds);
  void apply_slow_interval(float seconds);
  void apply_convention(uint8_t c);
  void apply_auto_protection(bool on);
  void apply_protect_eeprom(bool on);
  void set_min_rate_number(number::Number *n) { this->min_rate_num_ = n; }
  void set_max_rate_number(number::Number *n) { this->max_rate_num_ = n; }
  void set_update_number(number::Number *n) { this->update_num_ = n; }
  void set_slow_number(number::Number *n) { this->slow_num_ = n; }
  void set_convention_select(select::Select *s) { this->convention_select_ = s; }
  void set_auto_protection_switch(switch_::Switch *s) { this->auto_prot_sw_ = s; }
  void set_protect_eeprom_switch(switch_::Switch *s) { this->eeprom_sw_ = s; }
  uint8_t get_min_power_rate() const { return this->min_power_rate_; }
  // How much this unit could produce at 100 %, inferred only while our own
  // limit is what it is actually hitting. See update_capability_().
  void set_capability_params(float ratio, uint32_t window_ms) {
    this->cap_ratio_ = ratio;
    this->cap_window_ms_ = window_ms;
  }
  float get_capability() const { return this->capability_w_; }
  bool rate_is_binding() const { return this->rate_binding_; }
  // What raising this unit could still deliver. Zero when our limit is not what
  // is holding it back, because then raising it achieves nothing.
  float available_headroom() const;

  // ---- controller bookkeeping -------------------------------------------
  // Held here rather than in a table beside the inverter list because the slot
  // already knows which slot it is, and a parallel vector would be one more
  // thing to keep in step with enable/disable.
  //
  // A unit with no capability estimate is not necessarily PV limited - it may
  // simply never have been seen clipping. The only way to tell the two apart
  // is to raise it and watch, so the controller is allowed to probe; this
  // keeps that from happening every cycle.
  // True unless the last rate change went the other way and has not yet had
  // time to show up in the output. Reversing inside that window is how the
  // rebalance and increase passes ended up undoing each other every cycle:
  // neither could see what the other had just done, because the inverter had
  // not answered yet.
  //
  // Safety paths - export over the cap, grid or terminal over voltage - do not
  // consult this. Waiting is not an option when the meter says watts are going
  // out of the gate, and a reduction is always safe to make.
  bool may_move(int8_t dir, uint32_t now, uint32_t lockout) const {
    if (this->ctrl_dir_ == 0 || dir == this->ctrl_dir_)
      return true;
    return now - this->ctrl_move_ms_ >= lockout;
  }
  uint32_t since_last_move(uint32_t now) const {
    return now - this->ctrl_move_ms_;
  }

  bool probe_due(uint32_t now, uint32_t interval) const {
    return this->ctrl_probe_ms_ == 0 || now - this->ctrl_probe_ms_ >= interval;
  }
  void note_probe(uint32_t now) { this->ctrl_probe_ms_ = now; }
  uint8_t get_max_power_rate() const { return this->max_power_rate_; }

  // Clamps, stores and writes the active power rate, refreshing the UI entity.
  void apply_power_rate(float pct);

  // False when the inverter is already giving everything it can, so asking for
  // more would be pointless: either it reports derating, or its real output has
  // not kept up with the setpoint we last gave it.
  bool can_produce_more() const;

  // Some models report the phase to neutral voltage (around 230 V) on Vac1..3
  // and others the line to line voltage (around 400 V). The protection limits
  // have to be written in whichever convention the unit uses.
  bool reports_line_voltage() const;
  // What this unit measures at its own terminals, which is higher than the
  // meter sees by whatever the AC cabling drops. Highest of the three, since one
  // phase over the limit is enough to trip the whole unit.
  //
  // These are two separate registers blocks and two separate questions. A MOD
  // 40K populates 38/42/46 with phase voltages AND 50-52 with line voltages at
  // the same time, so each has to be compared against its own threshold.
  // reports_line_voltage() answers a third question entirely - which convention
  // the protection registers 52/53 expect - and must not be used to decide how
  // to read these.
  float peak_ac_voltage() const;
  float peak_line_voltage() const;
  // Whether the AC voltage registers hold line voltages rather than phase
  // ones, which decides which threshold they must be compared against. This is
  // NOT the same question as reports_line_voltage(): a MOD 40K wants its
  // protection registers written in line terms while still reporting phase
  // voltages at 38/42/46, and an SPH reports line voltages there.
  bool ac_voltage_is_line() const;
  // One phase of the same reading, for diagnostics. A single phase unit only
  // ever populates index 0, whatever grid phase it is wired to.
  float get_ac_voltage(uint8_t i) const {
    return i < 3 ? this->ac_voltage_[i] : NAN;
  }

  // Protection limits the controller wants written, in volts. They are applied
  // once identification has revealed the voltage convention, and only when the
  // registers hold something different.
  void set_protection_targets(float phase_low, float phase_high, float line_low,
                              float line_high, uint16_t restart_delay_s);

  // What the inverter reports it is actually doing, used by the controller to
  // tell "cannot go higher" apart from "has not been asked to".
  uint16_t get_real_power_percent() const { return this->real_percent_val_; }
  uint8_t get_derating_mode() const { return this->derating_val_; }
  // Human readable reason the inverter is holding itself back.
  const char *get_derating_text() const;
  bool is_derated() const { return this->derating_val_ != 0; }

  // Nameplate power, and whether it looked sane. The controller falls back to
  // fixed steps when it did not.
  float get_normal_power() const { return this->normal_power_va_; }
  bool has_valid_normal_power() const { return this->normal_power_valid_; }

  // ---------------------- fleet aggregate contributions ----------------------
  // What this slot adds to the hub totals. A unit that has gone quiet keeps
  // contributing its last known figures until it is declared offline, which is
  // the same window the health machine uses to tell a comms glitch from a unit
  // that has actually stopped. After that it contributes nothing, so the totals
  // fall as units drop out instead of quoting numbers from hours ago.
  bool contributes() const {
    return this->is_enabled() && this->health_ != INV_OFFLINE;
  }

  /// Nameplate scaled by the configured rate ceiling, counted whatever the
  /// unit's health. This is what is bolted to the roof and allowed to run: it
  /// does not move when a unit stops answering, only when the ceiling is
  /// re-configured or the nameplate is revised. Zero when the nameplate never
  /// validated - a slot with a nonsense nameplate must not be counted at face
  /// value.
  float rated_capacity() const {
    if (!this->is_enabled() || !this->normal_power_valid_)
      return 0.0f;
    return this->normal_power_va_ * this->max_power_rate_ / 100.0f;
  }

  /// The same figure, but only while the unit is still counted as present. Use
  /// this for "what the live fleet is allowed to reach"; rated_capacity() for
  /// "what is installed".
  float installed_capacity() const {
    return this->contributes() ? this->rated_capacity() : 0.0f;
  }

  /// What this unit looks able to deliver right now. The estimate only exists
  /// while our own rate limit is what the unit is pressing against
  /// (update_capability_); at 100 % and unconstrained there is nothing to infer,
  /// so current output stands in - at full rate that is the capability actually
  /// observable. Never NaN, so the sum stays defined with mixed slots.
  float effective_capability() const {
    if (!this->contributes())
      return 0.0f;
    if (!std::isnan(this->capability_w_))
      return this->capability_w_;
    return std::isnan(this->grid_power_w_) ? 0.0f : this->grid_power_w_;
  }

  /// Power being injected right now, zero once the unit is written off.
  float contributed_power() const {
    if (!this->contributes() || std::isnan(this->grid_power_w_))
      return 0.0f;
    return this->grid_power_w_;
  }

  // ---------------------- battery pack geometry (configurable) ----------------
  // Defaults match Growatt ARK 2.5 modules. Community hardware differs, so
  // these must not be hardcoded inside the derived calculations.
  void set_module_voltage(float v) { this->module_voltage_ = v; }
  void set_module_capacity(float kwh) { this->module_capacity_ = kwh; }
  void set_discharge_hours(float h) { this->discharge_hours_ = h; }
  // Output below which the per phase power registers tell us nothing, because
  // a phase carrying almost no active power is indistinguishable from one that
  // is not reported at all. Zero lets the detection run at any output, which
  // is only safe on a unit that never idles with reactive current flowing.
  void set_phase_detect_min_power(float w) { this->phase_detect_min_w_ = w; }
  void set_ups_avg_window(uint8_t n) {
    this->ups_avg_window_ = (n == 0 || n > UPS_AVG_MAX) ? UPS_AVG_MAX : n;
  }
  // Counters and diagnostics are refreshed on this longer cadence so the fast
  // cycle stays short enough for control decisions.
  void set_slow_interval(uint32_t ms) { this->slow_interval_ = ms; }

  // ------------------------------ indexed setters ------------------------------
  void set_phase_sensor(uint8_t i, uint8_t kind, sensor::Sensor *s);
  void set_pv_sensor(uint8_t i, uint8_t kind, sensor::Sensor *s);
  void set_ups_sensor(uint8_t i, uint8_t kind, sensor::Sensor *s);
  void set_line_voltage(uint8_t i, sensor::Sensor *s);
  void set_pv_energy_today(uint8_t i, sensor::Sensor *s);
  void set_pv_energy_total(uint8_t i, sensor::Sensor *s);

#define GI_SETTER(name, member) \
  void set_##name(sensor::Sensor *s) { this->member = s; }

  // input 0..124
  GI_SETTER(status_code, status_code_)
  GI_SETTER(pv_active_power, pv_active_power_)
  GI_SETTER(capability, capability_sens_)
  GI_SETTER(grid_active_power, grid_active_power_)
  GI_SETTER(frequency, frequency_)
  GI_SETTER(energy_today, energy_today_)
  GI_SETTER(energy_total, energy_total_)
  GI_SETTER(work_time_total, work_time_total_)
  GI_SETTER(pv_energy_total_all, pv_energy_total_all_)
  GI_SETTER(temperature, temperature_)
  GI_SETTER(ipm_temperature, ipm_temperature_)
  GI_SETTER(boost_temperature, boost_temperature_)
  GI_SETTER(battery_voltage_dsp, battery_voltage_dsp_)
  GI_SETTER(bus_voltage_p, bus_voltage_p_)
  GI_SETTER(bus_voltage_n, bus_voltage_n_)
  GI_SETTER(output_power_factor, output_power_factor_)
  GI_SETTER(real_power_percent, real_power_percent_)
  GI_SETTER(output_max_power, output_max_power_)
  GI_SETTER(derating_mode, derating_mode_)
  GI_SETTER(fault_code, fault_code_)
  GI_SETTER(fault_subcode, fault_subcode_)
  GI_SETTER(warning_bits, warning_bits_)
  GI_SETTER(warning_subcode, warning_subcode_)
  GI_SETTER(warning_code, warning_code_)
  GI_SETTER(ac_charge_energy_today, ac_charge_e_today_)
  GI_SETTER(ac_charge_energy_total, ac_charge_e_total_)
  GI_SETTER(ac_charge_power, ac_charge_power_)
  GI_SETTER(priority, priority_)
  GI_SETTER(battery_type, battery_type_)

  // holding 0..124, read once
  GI_SETTER(normal_power, normal_power_)
  GI_SETTER(modbus_version, modbus_version_)
  GI_SETTER(active_rate, active_rate_)
  GI_SETTER(reactive_rate, reactive_rate_)
  GI_SETTER(power_factor_set, power_factor_set_)
  GI_SETTER(pv_nominal_voltage, pv_nominal_voltage_)
  GI_SETTER(pv_start_voltage, pv_start_voltage_)
  GI_SETTER(start_time, start_time_)
  GI_SETTER(restart_delay, restart_delay_)
  GI_SETTER(com_address_sensor, com_address_)
  GI_SETTER(grid_voltage_low, grid_voltage_low_)
  GI_SETTER(grid_voltage_high, grid_voltage_high_)
  GI_SETTER(grid_freq_low, grid_freq_low_)
  GI_SETTER(grid_freq_high, grid_freq_high_)
  GI_SETTER(pf_model, pf_model_)
  GI_SETTER(export_limit_enable, export_limit_enable_)
  GI_SETTER(export_limit_rate, export_limit_rate_)
  GI_SETTER(tracker_model, tracker_model_)

  // input 1000..1096, storage only
  GI_SETTER(system_work_mode, system_work_mode_)
  GI_SETTER(fault_word, fault_word_)
  GI_SETTER(battery_voltage, battery_voltage_)
  GI_SETTER(battery_soc, battery_soc_sens_)
  GI_SETTER(battery_charge_power, bat_charge_power_)
  GI_SETTER(battery_discharge_power, bat_discharge_power_)
  GI_SETTER(battery_capacity, battery_capacity_)
  GI_SETTER(battery_cycles, battery_cycles_)
  GI_SETTER(battery_health, battery_health_)
  GI_SETTER(battery_temperature, battery_temperature_)
  GI_SETTER(charge_energy_today, charge_energy_today_)
  GI_SETTER(charge_energy_total, charge_energy_total_)
  GI_SETTER(discharge_energy_today, discharge_energy_today_)
  GI_SETTER(discharge_energy_total, discharge_energy_total_)
  GI_SETTER(power_to_user, power_to_user_)
  GI_SETTER(power_to_grid, power_to_grid_)
  GI_SETTER(local_load_power, local_load_power_)
  GI_SETTER(energy_to_user_today, e_to_user_today_)
  GI_SETTER(energy_to_user_total, e_to_user_total_)
  GI_SETTER(energy_to_grid_today, e_to_grid_today_)
  GI_SETTER(energy_to_grid_total, e_to_grid_total_)
  GI_SETTER(local_load_energy_today, e_load_today_)
  GI_SETTER(local_load_energy_total, e_load_total_)
  GI_SETTER(ups_frequency, ups_frequency_)
  GI_SETTER(ups_load, ups_load_)
  GI_SETTER(ups_power_factor, ups_power_factor_)
  GI_SETTER(bms_soc, bms_soc_)
  GI_SETTER(bms_voltage, bms_voltage_)
  GI_SETTER(bms_current, bms_current_)
  GI_SETTER(bms_temperature, bms_temperature_)

  // derived, computed inside the component
  GI_SETTER(ups_total_power, ups_total_power_)
  GI_SETTER(ups_load_avg, ups_load_avg_)
  GI_SETTER(ups_max_power, ups_max_power_)
  GI_SETTER(battery_modules, battery_modules_)
#undef GI_SETTER

#define GI_TSETTER(name, member) \
  void set_##name(text_sensor::TextSensor *ts) { this->member = ts; }
  GI_TSETTER(control_summary_text_sensor, control_ts_)
  GI_TSETTER(info_text_sensor, info_ts_)
  GI_TSETTER(firmware_text_sensor, firmware_ts_)
  GI_TSETTER(fw_build_text_sensor, fw_build_ts_)
  GI_TSETTER(serial_text_sensor, serial_ts_)
  GI_TSETTER(manufacturer_text_sensor, manufacturer_ts_)
  GI_TSETTER(model_text_sensor, model_ts_)
  GI_TSETTER(bootloader_text_sensor, bootloader_ts_)
  GI_TSETTER(system_time_text_sensor, system_time_ts_)
  GI_TSETTER(status_text_sensor, status_ts_)
  GI_TSETTER(fault_text_sensor, fault_ts_)
  GI_TSETTER(derating_text_sensor, derating_ts_)
  GI_TSETTER(state_text_sensor, state_ts_)
#undef GI_TSETTER

 protected:
  void try_send_();
  // Starts the wait when a request was accepted into the queue, and unwinds the
  // transaction the way a timeout would when it was refused.
  bool queued_(bool ok);
  // Shared preamble of every terminal callback: refreshes liveness and reports
  // whether this is a frame we are actually waiting for.
  bool answered_();
  // Retires the write at the head of the queue, acknowledged or rejected.
  void finish_write_(modbus::ResponseStatus status);
  void send_step_();
  void advance_(bool ok);
  void publish_info_();
  void apply_overrides_();
  void detect_from_live_(std::span<const uint16_t> data);
  void send_dump_chunk_();
  void handle_dump_(std::span<const uint16_t> data);
  void dump_skip_range_();
  void save_prefs_();

  void start_poll_();
  void send_poll_();
  void advance_poll_();
  void parse_fast_main_(std::span<const uint16_t> data);
  void parse_fast_status_(std::span<const uint16_t> data);
  void parse_slow_main_(std::span<const uint16_t> data);
  void parse_fast_bat_(std::span<const uint16_t> data);
  void parse_fast_ups_(std::span<const uint16_t> data);
  void parse_device_info_(std::span<const uint16_t> data);
  void parse_storage_(std::span<const uint16_t> data);
  void parse_settings_(std::span<const uint16_t> data);
  void publish_derived_();
  void publish_settings_();
  void apply_protection_limits_();
  // State of charge below which the inverter stops discharging by itself.
  float discharge_floor_() const;
  // Starts a run without clearing the attempt counter, so an automatic retry
  // after a failed run cannot loop forever.
  void begin_identification_();
  void update_health_();
  void zero_instantaneous_();
  void send_probe_();
  // Publishes any register backed select or switch whose address falls inside
  // the block that was just read, so they pick up values from whichever read
  // covers them rather than only from the first holding group.
  void publish_reg_entities_(std::span<const uint16_t> data, uint16_t base,
                             uint16_t count);

  bool queue_write_(uint8_t function, uint16_t address, const uint16_t *values,
                    uint8_t count);
  void send_write_();

  ESPPreferenceObject pref_;

  uint8_t slot_index_{0};
  IdentStep step_{IDENT_START};
  // A probe that answers with zeros is a definitive "not supported", but a
  // probe that never answers tells us nothing. Treating the two the same would
  // silently downgrade a storage inverter to grid tie after one lost frame, so
  // an incomplete run is retried instead.
  bool ident_incomplete_{false};
  uint32_t ident_retry_at_{0};
  uint8_t ident_runs_{0};
  uint8_t health_{INV_ONLINE};
  // The state entity is written on transitions, and the initial value above is
  // the one a healthy slot settles on, so a unit that comes up and stays up
  // never produces a transition and its entity stays unknown for the life of
  // the node. This makes the first evaluation count as one regardless of what
  // it finds. Set even when no state sensor is configured, so the early return
  // still short-circuits and the transition logging does not repeat.
  bool health_published_{false};
  uint32_t stalled_ms_{10000};
  uint32_t offline_ms_{20000};
  uint32_t offline_probe_ms_{60000};
  uint32_t last_probe_{0};
  bool probing_{false};
  PollBlock poll_{POLL_IDLE};
  uint32_t slow_interval_{30000};
  uint32_t last_slow_{0};
  bool slow_due_{false};
  uint32_t last_send_{0};
  uint32_t last_update_{0};
  bool waiting_{false};
  bool want_send_{false};
  bool busy_logged_{false};
  uint32_t bus_release_{0};
  uint8_t retries_{0};
  uint8_t power_percent_{0};
  // Capability estimate. NAN until the unit has been seen actually clipping at
  // our setpoint; expires after cap_window_ms_ because an hour old figure says
  // nothing about the sun now.
  float capability_w_{NAN};
  uint32_t cap_time_{0};
  float cap_ratio_{0.9f};
  uint32_t cap_window_ms_{3600000};
  bool rate_binding_{false};
  uint32_t ctrl_probe_ms_{0};
  int8_t ctrl_dir_{0};
  uint32_t ctrl_move_ms_{0};
  void update_capability_();
  sensor::Sensor *capability_sens_{nullptr};

  uint8_t safe_power_rate_{0};
  number::Number *safe_rate_num_{nullptr};
  number::Number *min_rate_num_{nullptr};
  number::Number *max_rate_num_{nullptr};
  number::Number *update_num_{nullptr};
  number::Number *slow_num_{nullptr};
  select::Select *convention_select_{nullptr};
  switch_::Switch *auto_prot_sw_{nullptr};
  switch_::Switch *eeprom_sw_{nullptr};
  uint8_t min_power_rate_{0};
  uint8_t max_power_rate_{100};
  uint16_t real_percent_val_{0};
  uint8_t derating_val_{0};
  uint8_t derating_prev_{255};
  float ac_voltage_[3]{NAN, NAN, NAN};
  float ac_line_voltage_[3]{NAN, NAN, NAN};
  // Set when the unit puts its whole AC output in Pac1 and leaves Pac2 and
  // Pac3 at zero, which makes all three registers useless as per phase values.
  bool pac_is_total_{false};
  uint8_t pac_total_hits_{0};
  float phase_detect_min_w_{100.0f};
  uint8_t cfg_convention_{CONV_AUTO};

  // protection limit targets, in volts
  // On by default: the trip windows are derived from the hub thresholds the
  // controller already works within, so leaving the inverter on its factory
  // limits means it can disconnect before the controller ever gets a chance to
  // reduce output.
  bool auto_protection_{true};
  bool protection_applied_{false};
  float tgt_phase_low_{0};
  float tgt_phase_high_{0};
  float tgt_line_low_{0};
  float tgt_line_high_{0};
  uint16_t tgt_restart_delay_{0};

  bool dump_active_{false};
  uint8_t dump_range_{0};
  uint16_t dump_offset_{0};

  // staged window state and independent settings
  TimeWindow windows_[MODE_COUNT][PERIOD_COUNT];
  uint16_t settings_[SET_COUNT]{};
  bool ac_charge_{false};
  uint8_t phase_{INV_PHASE_L1};

  // register backed selects, looked up by holding address
  static const uint8_t MAX_REG_SELECTS = 4;
  uint16_t reg_select_addr_[MAX_REG_SELECTS]{};
  select::Select *reg_select_[MAX_REG_SELECTS]{};
  uint8_t reg_select_count_{0};
  select::Select *phase_select_{nullptr};

  // register backed switches
  static const uint8_t MAX_REG_SWITCHES = 6;
  uint16_t reg_switch_addr_[MAX_REG_SWITCHES]{};
  uint16_t reg_switch_on_[MAX_REG_SWITCHES]{};
  switch_::Switch *reg_switch_[MAX_REG_SWITCHES]{};
  uint8_t reg_switch_count_{0};
  // On by default: the controller rewrites the power rate every few seconds,
  // and letting those reach the EEPROM would wear it out. Clearing holding 2
  // also means a unit left alone comes back unrestricted rather than stuck at
  // whatever limit was last applied.
  bool protect_eeprom_{true};

  // Registers this unit answered an exception to. A MIN rejects a write to the
  // reconnect delay at holding 19 that a MOD accepts, and an exception is a
  // capability answer rather than a transient fault - so it is remembered, and
  // the register is not written again until the next identification. Without
  // this, every protection pass would retry the same futile write.
  static const uint8_t MAX_REJECTED = 8;
  uint16_t rejected_[MAX_REJECTED]{};
  uint8_t rejected_count_{0};
  bool is_rejected_(uint16_t address) const;
  void mark_rejected_(uint16_t address);

  // outgoing write commands, drained before any read
  PendingWrite write_queue_[WRITE_QUEUE_SIZE];
  uint8_t write_head_{0};
  uint8_t write_count_{0};
  bool writing_{false};

  number::Number *win_num_[MODE_COUNT][PERIOD_COUNT][PART_COUNT]{};
  switch_::Switch *win_sw_[MODE_COUNT][PERIOD_COUNT]{};
  number::Number *setting_num_[SET_COUNT]{};
  switch_::Switch *ac_charge_sw_{nullptr};

  // Overrides for the two capabilities that cannot always be measured. Both
  // feed the same persisted fields the YAML options set, so a select is just
  // another way in, not a second source of truth.
  select::Select *phase_count_select_{nullptr};
  select::Select *strings_select_{nullptr};
  number::Number *address_num_{nullptr};
  // Republished whenever any of these change, not just at boot: the address in
  // particular can move from several places and a stale entity is worse than
  // none, because it looks authoritative.
  void publish_cfg_entities_();

  int8_t cfg_phases_{0};
  int8_t cfg_strings_{0};
  int8_t cfg_ups_{CFG_AUTO};
  int8_t cfg_battery_{CFG_AUTO};

  // battery pack geometry, defaults for Growatt ARK 2.5
  float module_voltage_{53.75f};
  float module_capacity_{2.5f};
  float discharge_hours_{2.5f};

  // kept for the derived calculations
  float normal_power_va_{0};
  bool normal_power_valid_{false};
  // The Storage family reports nameplate in whole VA instead of 0.1 VA, so the
  // figure reads ten times low. The plausibility guard at parse time cannot
  // catch all of it: an SPH 10000 yields 1000 VA, which is perfectly plausible
  // for a small single phase unit. Revised later from things not known yet at
  // identification - the phase count and the output actually observed.
  bool nameplate_revised_{false};
  void revise_nameplate_();
  float battery_voltage_v_{0};
  float ups_load_pct_{NAN};
  float ups_load_avg_pct_{NAN};
  float battery_soc_pct_{NAN};
  float grid_power_w_{NAN};
  float pv_power_w_{NAN};
  float ups_phase_power_[3]{};
  uint16_t ups_avg_buf_[UPS_AVG_MAX]{};
  uint8_t ups_avg_pos_{0};
  uint8_t ups_avg_count_{0};
  uint8_t ups_avg_window_{UPS_AVG_MAX};

  GrowattCaps caps_;

  SensorTriple phases_[3];
  SensorTriple pvs_[MAX_STRINGS];
  SensorTriple ups_[3];
  sensor::Sensor *line_voltages_[3]{};
  sensor::Sensor *pv_energy_today_[MAX_STRINGS]{};
  sensor::Sensor *pv_energy_total_[MAX_STRINGS]{};

  sensor::Sensor *status_code_{nullptr};
  sensor::Sensor *pv_active_power_{nullptr};
  sensor::Sensor *grid_active_power_{nullptr};
  sensor::Sensor *frequency_{nullptr};
  sensor::Sensor *energy_today_{nullptr};
  sensor::Sensor *energy_total_{nullptr};
  sensor::Sensor *work_time_total_{nullptr};
  sensor::Sensor *pv_energy_total_all_{nullptr};
  sensor::Sensor *temperature_{nullptr};
  sensor::Sensor *ipm_temperature_{nullptr};
  sensor::Sensor *boost_temperature_{nullptr};
  sensor::Sensor *battery_voltage_dsp_{nullptr};
  sensor::Sensor *bus_voltage_p_{nullptr};
  sensor::Sensor *bus_voltage_n_{nullptr};
  sensor::Sensor *output_power_factor_{nullptr};
  sensor::Sensor *real_power_percent_{nullptr};
  sensor::Sensor *output_max_power_{nullptr};
  sensor::Sensor *derating_mode_{nullptr};
  sensor::Sensor *fault_code_{nullptr};
  sensor::Sensor *fault_subcode_{nullptr};
  sensor::Sensor *warning_bits_{nullptr};
  sensor::Sensor *warning_subcode_{nullptr};
  sensor::Sensor *warning_code_{nullptr};
  sensor::Sensor *ac_charge_e_today_{nullptr};
  sensor::Sensor *ac_charge_e_total_{nullptr};
  sensor::Sensor *ac_charge_power_{nullptr};
  sensor::Sensor *priority_{nullptr};
  sensor::Sensor *battery_type_{nullptr};

  sensor::Sensor *normal_power_{nullptr};
  sensor::Sensor *modbus_version_{nullptr};
  sensor::Sensor *active_rate_{nullptr};
  sensor::Sensor *reactive_rate_{nullptr};
  sensor::Sensor *power_factor_set_{nullptr};
  sensor::Sensor *pv_nominal_voltage_{nullptr};
  sensor::Sensor *pv_start_voltage_{nullptr};
  sensor::Sensor *start_time_{nullptr};
  sensor::Sensor *restart_delay_{nullptr};
  sensor::Sensor *com_address_{nullptr};
  sensor::Sensor *grid_voltage_low_{nullptr};
  sensor::Sensor *grid_voltage_high_{nullptr};
  sensor::Sensor *grid_freq_low_{nullptr};
  sensor::Sensor *grid_freq_high_{nullptr};
  sensor::Sensor *pf_model_{nullptr};
  sensor::Sensor *export_limit_enable_{nullptr};
  sensor::Sensor *export_limit_rate_{nullptr};
  sensor::Sensor *tracker_model_{nullptr};

  sensor::Sensor *system_work_mode_{nullptr};
  sensor::Sensor *fault_word_{nullptr};
  sensor::Sensor *battery_voltage_{nullptr};
  sensor::Sensor *battery_soc_sens_{nullptr};
  sensor::Sensor *bat_charge_power_{nullptr};
  sensor::Sensor *bat_discharge_power_{nullptr};
  sensor::Sensor *battery_capacity_{nullptr};
  sensor::Sensor *battery_cycles_{nullptr};
  sensor::Sensor *battery_health_{nullptr};
  sensor::Sensor *battery_temperature_{nullptr};
  sensor::Sensor *charge_energy_today_{nullptr};
  sensor::Sensor *charge_energy_total_{nullptr};
  sensor::Sensor *discharge_energy_today_{nullptr};
  sensor::Sensor *discharge_energy_total_{nullptr};
  sensor::Sensor *power_to_user_{nullptr};
  sensor::Sensor *power_to_grid_{nullptr};
  sensor::Sensor *local_load_power_{nullptr};
  sensor::Sensor *e_to_user_today_{nullptr};
  sensor::Sensor *e_to_user_total_{nullptr};
  sensor::Sensor *e_to_grid_today_{nullptr};
  sensor::Sensor *e_to_grid_total_{nullptr};
  sensor::Sensor *e_load_today_{nullptr};
  sensor::Sensor *e_load_total_{nullptr};
  sensor::Sensor *ups_frequency_{nullptr};
  sensor::Sensor *ups_load_{nullptr};
  sensor::Sensor *ups_power_factor_{nullptr};
  sensor::Sensor *bms_soc_{nullptr};
  sensor::Sensor *bms_voltage_{nullptr};
  sensor::Sensor *bms_current_{nullptr};
  sensor::Sensor *bms_temperature_{nullptr};

  sensor::Sensor *ups_total_power_{nullptr};
  sensor::Sensor *ups_load_avg_{nullptr};
  sensor::Sensor *ups_max_power_{nullptr};
  sensor::Sensor *battery_modules_{nullptr};

  text_sensor::TextSensor *control_ts_{nullptr};
  text_sensor::TextSensor *info_ts_{nullptr};
  text_sensor::TextSensor *firmware_ts_{nullptr};
  text_sensor::TextSensor *fw_build_ts_{nullptr};
  text_sensor::TextSensor *serial_ts_{nullptr};
  text_sensor::TextSensor *manufacturer_ts_{nullptr};
  text_sensor::TextSensor *model_ts_{nullptr};
  text_sensor::TextSensor *bootloader_ts_{nullptr};
  text_sensor::TextSensor *system_time_ts_{nullptr};
  text_sensor::TextSensor *status_ts_{nullptr};
  text_sensor::TextSensor *fault_ts_{nullptr};
  text_sensor::TextSensor *derating_ts_{nullptr};
  text_sensor::TextSensor *state_ts_{nullptr};
};

// ---------------------------------------------------------------------------
// Control entities. They carry no logic of their own: every edit is forwarded
// to the inverter object, which decides whether it goes out immediately or is
// staged until the matching Apply button is pressed.
// ---------------------------------------------------------------------------

// Independent setting, written straight away with function 0x06.
class GrowattSettingNumber : public number::Number {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }
  void set_field(uint8_t f) { this->field_ = f; }

 protected:
  void control(float value) override {
    this->publish_state(value);
    if (this->parent_ != nullptr)
      this->parent_->set_setting(this->field_, (uint16_t) value);
  }
  GrowattInverter *parent_{nullptr};
  uint8_t field_{0};
};

// One part of a time window: staged only, nothing is sent until Apply.
class GrowattWindowNumber : public number::Number {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }
  void set_target(uint8_t mode, uint8_t period, uint8_t part) {
    this->mode_ = mode;
    this->period_ = period;
    this->part_ = part;
  }

 protected:
  void control(float value) override {
    this->publish_state(value);
    if (this->parent_ != nullptr)
      this->parent_->set_window_part(this->mode_, this->period_, this->part_,
                                     (uint8_t) value);
  }
  GrowattInverter *parent_{nullptr};
  uint8_t mode_{0}, period_{0}, part_{0};
};

// Enable flag of a time window: staged only.
class GrowattWindowSwitch : public switch_::Switch {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }
  void set_target(uint8_t mode, uint8_t period) {
    this->mode_ = mode;
    this->period_ = period;
  }

 protected:
  void write_state(bool state) override {
    this->publish_state(state);
    if (this->parent_ != nullptr)
      this->parent_->set_window_enabled(this->mode_, this->period_, state);
  }
  GrowattInverter *parent_{nullptr};
  uint8_t mode_{0}, period_{0};
};

// AC charge enable, written straight away.
class GrowattAcChargeSwitch : public switch_::Switch {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }

 protected:
  void write_state(bool state) override {
    this->publish_state(state);
    if (this->parent_ != nullptr)
      this->parent_->set_ac_charge(state);
  }
  GrowattInverter *parent_{nullptr};
};

// Commits a whole window block atomically.
class GrowattApplyButton : public button::Button {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }
  void set_mode(uint8_t m) { this->mode_ = m; }

 protected:
  void press_action() override {
    if (this->parent_ != nullptr)
      this->parent_->apply_windows(this->mode_);
  }
  GrowattInverter *parent_{nullptr};
  uint8_t mode_{0};
};

// Re-runs capability detection.
class GrowattRefreshButton : public button::Button {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }

 protected:
  void press_action() override {
    if (this->parent_ != nullptr)
      this->parent_->restart_identification();
  }
  GrowattInverter *parent_{nullptr};
};

// Which mains phase a single phase inverter feeds. Not written anywhere on the
// inverter, only kept locally for the power controller.
class GrowattPhaseSelect : public select::Select {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }

 protected:
  void control(const std::string &value) override {
    this->publish_state(value);
    if (this->parent_ == nullptr)
      return;
    if (value == "L2")
      this->parent_->set_phase(INV_PHASE_L2);
    else if (value == "L3")
      this->parent_->set_phase(INV_PHASE_L3);
    else
      this->parent_->set_phase(INV_PHASE_L1);
  }
  GrowattInverter *parent_{nullptr};
};

// Select whose chosen index is written straight into a holding register.
// Used for battery type (1048) and export limit mode (122).
class GrowattRegisterSelect : public select::Select {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }
  void set_address(uint16_t a) { this->address_ = a; }

 protected:
  void control(const std::string &value) override {
    this->publish_state(value);
    auto idx = this->index_of(value);
    if (idx.has_value() && this->parent_ != nullptr)
      this->parent_->write_register(this->address_, (uint16_t) idx.value());
  }
  GrowattInverter *parent_{nullptr};
  uint16_t address_{0};
};

// Switch whose two states are two values written to one holding register.
// Covers setting memory (2: 1/0), inverter on/off (0: 1/0) and BDC on/off
// (0: 3/2).
class GrowattRegisterSwitch : public switch_::Switch {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }
  void set_registers(uint16_t address, uint16_t on_value, uint16_t off_value) {
    this->address_ = address;
    this->on_value_ = on_value;
    this->off_value_ = off_value;
  }

 protected:
  void write_state(bool state) override {
    this->publish_state(state);
    if (this->parent_ != nullptr)
      this->parent_->write_register(this->address_,
                                    state ? this->on_value_ : this->off_value_);
  }
  GrowattInverter *parent_{nullptr};
  uint16_t address_{0};
  uint16_t on_value_{1};
  uint16_t off_value_{0};
};

// Dumps the register map to the log.
class GrowattDumpButton : public button::Button {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }

 protected:
  void press_action() override {
    if (this->parent_ != nullptr)
      this->parent_->start_dump();
  }
  GrowattInverter *parent_{nullptr};
};

// Runtime editable inverter settings. All of them were compile time options,
// and all of them are things a person ends up wanting to change while watching
// the log rather than between flashes.
class GrowattRateNumber : public number::Number {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }
  void set_kind(uint8_t k) { this->kind_ = k; }

 protected:
  void control(float value) override;
  GrowattInverter *parent_{nullptr};
  uint8_t kind_{0};
};

class GrowattConventionSelect : public select::Select {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }

 protected:
  void control(const std::string &value) override;
  GrowattInverter *parent_{nullptr};
};

class GrowattInverterOptionSwitch : public switch_::Switch {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }
  void set_is_eeprom(bool v) { this->is_eeprom_ = v; }

 protected:
  void write_state(bool state) override;
  GrowattInverter *parent_{nullptr};
  bool is_eeprom_{false};
};

// The rate this inverter falls back to when the meter is long gone. Raising it
// above what the unit is doing now takes effect immediately, because the point
// of raising it is to get production back.
class GrowattSafeRateNumber : public number::Number {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }

 protected:
  void control(float value) override {
    this->publish_state(value);
    if (this->parent_ != nullptr)
      this->parent_->apply_safe_power_rate(value);
  }
  GrowattInverter *parent_{nullptr};
};

// Which address this slot talks to. 0 disables the slot entirely, which is the
// only way to silence a unit that is not on the bus without rebuilding.
class GrowattInverterAddressNumber : public number::Number {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }

 protected:
  void control(float value) override;
  GrowattInverter *parent_{nullptr};
};

// Capability overrides. "Auto" hands the decision back to detection, which is
// the right answer whenever the inverter is on grid and can simply be measured.
class GrowattPhaseCountSelect : public select::Select {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }

 protected:
  void control(const std::string &value) override;
  GrowattInverter *parent_{nullptr};
};

class GrowattStringsSelect : public select::Select {
 public:
  void set_parent(GrowattInverter *p) { this->parent_ = p; }

 protected:
  void control(const std::string &value) override;
  GrowattInverter *parent_{nullptr};
};

}  // namespace growatt_master
}  // namespace esphome
