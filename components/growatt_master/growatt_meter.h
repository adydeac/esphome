#pragma once

#include "growatt_master.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/number/number.h"

namespace esphome {
namespace growatt_master {

// ---------------------------------------------------------------------------
// Eastron SDM family smart meter.
//
// All Eastron meters and their many clones share the same input register map,
// which is exactly why they are drop in replacements for each other. That also
// means the map cannot be used to tell models apart - what actually differs is
// the number of phases, and that is detectable from the live voltages.
//
// Values are IEEE754 floats, big endian, read with function 0x04.
// ---------------------------------------------------------------------------

// Main block, input registers 0x0000..0x004F (80 registers), split into a fast
// part that carries the control relevant values and a slow part with the
// counters. At 9600 baud the fast part costs about 126 ms.
static const uint16_t SDM_MAIN_BASE = 0x0000;
static const uint8_t SDM_MAIN_CNT = 80;
static const uint16_t SDM_FAST_BASE = 0x0000;
static const uint8_t SDM_FAST_CNT = 54;   // through total active power
static const uint16_t SDM_SLOW_BASE = 54;  // 0x0036
static const uint8_t SDM_SLOW_CNT = 26;   // 54..79
static const uint8_t SDM_V[3] = {0, 2, 4};
static const uint8_t SDM_I[3] = {6, 8, 10};
static const uint8_t SDM_P[3] = {12, 14, 16};
static const uint8_t SDM_S[3] = {18, 20, 22};
static const uint8_t SDM_Q[3] = {24, 26, 28};
static const uint8_t SDM_PF[3] = {30, 32, 34};
static const uint8_t SDM_AVG_V = 42;
static const uint8_t SDM_AVG_I = 46;
static const uint8_t SDM_SUM_I = 48;
static const uint8_t SDM_TOTAL_P = 52;
static const uint8_t SDM_TOTAL_S = 56;
static const uint8_t SDM_TOTAL_Q = 60;
static const uint8_t SDM_TOTAL_PF = 62;
static const uint8_t SDM_FREQ = 70;
static const uint8_t SDM_IMPORT_E = 72;
static const uint8_t SDM_EXPORT_E = 74;
static const uint8_t SDM_IMPORT_RE = 76;
static const uint8_t SDM_EXPORT_RE = 78;

// Line to line voltages and neutral current, input 0x00C8..0x00E1.
static const uint16_t SDM_LINE_BASE = 0x00C8;
static const uint8_t SDM_LINE_CNT = 26;
static const uint8_t SDM_LINE_V[3] = {0, 2, 4};  // L1-L2, L2-L3, L3-L1
static const uint8_t SDM_LINE_AVG = 6;
static const uint8_t SDM_NEUTRAL_I = 24;

// Total energies, input 0x0156..0x0159.
static const uint16_t SDM_ENERGY_BASE = 0x0156;
static const uint8_t SDM_ENERGY_CNT = 4;
static const uint8_t SDM_TOTAL_ACTIVE_E = 0;
static const uint8_t SDM_TOTAL_REACTIVE_E = 2;

// Identification block. The exact meaning of these holding registers is not
// confirmed for every model, so the raw words are logged rather than mapped.
static const uint16_t SDM_ID_BASE = 0xFC00;
static const uint8_t SDM_ID_CNT = 4;

// Voltage above which a phase counts as connected, in volts.
static const float SDM_VOLTAGE_PRESENT = 50.0f;

enum MeterModel : uint8_t {
  METER_AUTO = 0,
  METER_SDM120,
  METER_SDM220,
  METER_SDM230,
  METER_SDM630,
  METER_MODEL_COUNT,
};

enum MeterStep : uint8_t {
  MSTEP_START = 0,
  MSTEP_ID,     // holding 0xFC00..0xFC03
  MSTEP_MAIN,   // input 0x0000..0x004F, also used for phase detection
  MSTEP_DONE,
};

enum MeterPoll : uint8_t {
  MPOLL_IDLE = 0,
  MPOLL_FAST,    // input 0x0000..0x0035
  MPOLL_SLOW,    // input 0x0036..0x004F
  MPOLL_LINE,
  MPOLL_ENERGY,
};

// See PREFS_VERSION in growatt_master.h for the convention.
struct MeterPrefs {
  uint8_t version;
  uint8_t address;
  uint8_t model;
  uint16_t update_interval;  // seconds
  uint16_t slow_interval;    // seconds
  uint8_t reserved[8];
} __attribute__((packed));

struct MeterTriple {
  sensor::Sensor *voltage{nullptr};
  sensor::Sensor *current{nullptr};
  sensor::Sensor *active_power{nullptr};
  sensor::Sensor *apparent_power{nullptr};
  sensor::Sensor *reactive_power{nullptr};
  sensor::Sensor *power_factor{nullptr};
};

// One callback for both register tables. The identification sequence reads
// holding registers for the id block and input registers for the main block, so
// splitting into on_read_holding_registers()/on_read_input_registers() would
// mean two entry points into one state machine for no gain: the step being
// served already says which table was asked for.
class GrowattMeter : public PollingComponent, public modbus::ModbusClientDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Success and exception both land here, the outcome in status.
  void on_read_registers(modbus::EntityType entity_type, uint16_t start_address,
                         std::span<const uint16_t> data,
                         modbus::ResponseStatus status) override;
  // Replies that do not match the request's shape are diverted here instead of
  // being delivered short; see the definition.
  void on_custom_response(std::span<const uint8_t> request_pdu,
                          std::span<const uint8_t> response_pdu,
                          modbus::ResponseStatus status) override;

  // ------------------------------ public API ------------------------------
  void change_address(uint8_t addr);
  uint8_t get_address() const { return this->address_; }
  void set_model(uint8_t m);
  uint8_t get_model() const { return this->model_; }
  void restart_identification();

  void set_slot_index(uint8_t i) { this->slot_index_ = i; }
  void set_address_number(number::Number *n) { this->address_num_ = n; }
  void set_model_select(select::Select *s) { this->model_select_ = s; }
  void set_slow_interval(uint32_t ms) { this->slow_interval_ = ms; }
  void apply_update_interval(float seconds);
  void apply_slow_interval(float seconds);
  void set_update_number(number::Number *n) { this->update_num_ = n; }
  void set_slow_number(number::Number *n) { this->slow_num_ = n; }

  bool is_enabled() const { return this->address_ != 0; }
  uint8_t get_phases() const { return this->phases_; }
  // Live per phase values, read by the hub for its aggregates and thresholds.
  float get_phase_power(uint8_t i) const {
    return i < 3 ? this->phase_power_[i] : NAN;
  }
  float get_phase_voltage(uint8_t i) const {
    return i < 3 ? this->phase_voltage_[i] : NAN;
  }
  uint32_t get_last_update() const { return this->last_update_; }
  bool is_stale(uint32_t timeout_us) const {
    return this->last_update_ == 0 || (micros() - this->last_update_) > timeout_us;
  }

  // ------------------------------ sensor setters ------------------------------
  // kind indices match the MeterTriple field order
  void set_phase_sensor(uint8_t i, uint8_t kind, sensor::Sensor *s);
  void set_line_voltage(uint8_t i, sensor::Sensor *s);

#define GM_SETTER(name, member) \
  void set_##name(sensor::Sensor *s) { this->member = s; }
  GM_SETTER(total_active_power, total_p_)
  GM_SETTER(total_apparent_power, total_s_)
  GM_SETTER(total_reactive_power, total_q_)
  GM_SETTER(total_power_factor, total_pf_)
  GM_SETTER(average_voltage, avg_v_)
  GM_SETTER(average_current, avg_i_)
  GM_SETTER(sum_current, sum_i_)
  GM_SETTER(frequency, frequency_)
  GM_SETTER(import_active_energy, import_e_)
  GM_SETTER(export_active_energy, export_e_)
  GM_SETTER(import_reactive_energy, import_re_)
  GM_SETTER(export_reactive_energy, export_re_)
  GM_SETTER(average_line_voltage, avg_line_v_)
  GM_SETTER(neutral_current, neutral_i_)
  GM_SETTER(total_active_energy, total_active_e_)
  GM_SETTER(total_reactive_energy, total_reactive_e_)
#undef GM_SETTER

  void set_info_text_sensor(text_sensor::TextSensor *ts) { this->info_ts_ = ts; }

 protected:
  void try_send_();
  // Starts the wait when a request was accepted; handles the refusal otherwise.
  bool queued_(bool ok);
  void send_step_();
  void advance_(bool ok);
  void start_poll_();
  void send_poll_();
  void advance_poll_();
  void parse_main_(std::span<const uint16_t> data);
  void parse_slow_(std::span<const uint16_t> data);
  void parse_line_(std::span<const uint16_t> data);
  void parse_energy_(std::span<const uint16_t> data);
  void detect_phases_(std::span<const uint16_t> data);
  void publish_info_();
  // Republished on every change, not just at boot: the address is editable and
  // a stale entity looks authoritative while being wrong.
  void publish_cfg_entities_();
  void save_prefs_();
  bool needs_line_block_() const;
  bool needs_energy_block_() const;

  ESPPreferenceObject pref_;

  uint8_t slot_index_{0};
  MeterStep step_{MSTEP_START};
  MeterPoll poll_{MPOLL_IDLE};
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

  uint8_t model_{METER_AUTO};
  uint8_t phases_{0};
  uint16_t id_words_[SDM_ID_CNT]{};
  float phase_power_[3]{NAN, NAN, NAN};
  float phase_voltage_[3]{NAN, NAN, NAN};

  select::Select *model_select_{nullptr};
  number::Number *address_num_{nullptr};
  number::Number *update_num_{nullptr};
  number::Number *slow_num_{nullptr};
  text_sensor::TextSensor *info_ts_{nullptr};

  MeterTriple phases_sens_[3];
  sensor::Sensor *line_voltages_[3]{};
  sensor::Sensor *total_p_{nullptr};
  sensor::Sensor *total_s_{nullptr};
  sensor::Sensor *total_q_{nullptr};
  sensor::Sensor *total_pf_{nullptr};
  sensor::Sensor *avg_v_{nullptr};
  sensor::Sensor *avg_i_{nullptr};
  sensor::Sensor *sum_i_{nullptr};
  sensor::Sensor *frequency_{nullptr};
  sensor::Sensor *import_e_{nullptr};
  sensor::Sensor *export_e_{nullptr};
  sensor::Sensor *import_re_{nullptr};
  sensor::Sensor *export_re_{nullptr};
  sensor::Sensor *avg_line_v_{nullptr};
  sensor::Sensor *neutral_i_{nullptr};
  sensor::Sensor *total_active_e_{nullptr};
  sensor::Sensor *total_reactive_e_{nullptr};
};

// Which address this meter answers on. 0 means the slot is empty, and every
// path in the component checks that before touching the bus.
// Poll intervals, editable at runtime. The meter feeds every control cycle, so
// its rate is the one most likely to need changing while watching the bus.
class GrowattMeterIntervalNumber : public number::Number {
 public:
  void set_parent(GrowattMeter *p) { this->parent_ = p; }
  void set_is_slow(bool v) { this->is_slow_ = v; }

 protected:
  void control(float value) override;
  GrowattMeter *parent_{nullptr};
  bool is_slow_{false};
};

class GrowattMeterAddressNumber : public number::Number {
 public:
  void set_parent(GrowattMeter *p) { this->parent_ = p; }

 protected:
  void control(float value) override;
  GrowattMeter *parent_{nullptr};
};

// Model override. "Auto" leaves phase detection in charge.
class GrowattMeterModelSelect : public select::Select {
 public:
  void set_parent(GrowattMeter *p) { this->parent_ = p; }

 protected:
  void control(const std::string &value) override;
  GrowattMeter *parent_{nullptr};
};

}  // namespace growatt_master
}  // namespace esphome
