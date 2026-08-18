#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/modbus/modbus.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/button/button.h"
#include <vector>
#include <string>
#include <cmath>

namespace esphome {
namespace growatt_master {

// Modbus function codes (per "Growatt Inverter Modbus RTU Protocol V1.24")
static const uint8_t CMD_READ_HOLDING = 0x03;
static const uint8_t CMD_READ_INPUT = 0x04;
static const uint8_t CMD_WRITE_SINGLE = 0x06;
static const uint8_t CMD_WRITE_MULTI = 0x10;  // Function 16, page 7

// Common register scaling factors
static const float ONE_DEC = 0.1f;
static const float TWO_DEC = 0.01f;

class GrowattInverter;
class GrowattMeter;

// Where each family keeps its own Modbus address, and how it has to be written.
// Growatt holds it as a plain integer at holding 30 and takes function 6.
// Eastron holds it as a float32 spanning holding 20-21 and only accepts
// function 16, and rejects any request for an odd number of registers - which
// is why the probe length is part of the profile rather than a constant.
// Kept in step by hand with the literals in __init__.py, which is where the
// tools are configured: generated code runs in the global setup(), where these
// names are not visible without their namespace.
static const uint16_t GROWATT_COM_ADDRESS = 0x001E;   // holding 30
static const uint16_t EASTRON_COM_ADDRESS = 0x0014;   // holding 20-21

enum AddrToolStep : uint8_t {
  ADDR_IDLE = 0,
  ADDR_PROBE,   // is anything already answering at the target address?
  ADDR_WRITE,
};

// Bus level address change tool. Deliberately unrelated to the devices declared
// in the configuration: its whole purpose is commissioning a unit that is not
// in the configuration yet, or moving one off a clashing address before it can
// be declared at all.
//
// One instance per bus. A ModbusClientDevice belongs to exactly one bus, so a
// hub spanning two of them cannot be the tool itself - it owns them instead.
class GrowattAddressTool : public Component, public modbus::ModbusClientDevice {
 public:
  void loop() override;
  void on_modbus_data(const std::vector<uint8_t> &data) override;
  void on_modbus_error(uint8_t function_code, uint8_t exception_code) override;
  float get_setup_priority() const override { return setup_priority::DATA - 2; }

  void set_from(uint8_t a) { this->from_ = a; }
  void set_to(uint8_t a) { this->to_ = a; }
  void set_status(text_sensor::TextSensor *ts) { this->status_ = ts; }
  void set_label(const char *l) { this->label_ = l; }
  // Device family profile, set from the configuration.
  void set_address_register(uint16_t r) { this->addr_reg_ = r; }
  void set_float_format(bool f) { this->float_format_ = f; }
  void start();

 protected:
  void send_();
  void finish_(const char *status, bool ok);

  AddrToolStep step_{ADDR_IDLE};
  uint8_t from_{0};
  uint8_t to_{0};
  uint8_t tries_{0};
  uint32_t sent_{0};
  bool waiting_{false};
  uint16_t addr_reg_{GROWATT_COM_ADDRESS};
  bool float_format_{false};
  const char *label_{"device"};
  text_sensor::TextSensor *status_{nullptr};
};

// Editable thresholds owned by the hub. They are the single place where a
// limit is written down; the derived binary sensors below let the YAML side
// consume the resulting condition without repeating the number.
enum HubSetting : uint8_t {
  HUB_PHASE_V_LOW = 0,     // V, phase to neutral, below this the mains is faulty
  HUB_PHASE_V_HIGH,        // V, phase to neutral, above this no inverter is raised
  HUB_LINE_V_LOW,          // V, line to line, used for inverters that report it
  HUB_LINE_V_HIGH,
  HUB_UPS_MAX_LOAD,        // %, instantaneous
  HUB_UPS_MAX_LOAD_AVG,    // %, rolling average
  HUB_BATTERY_SOC_MIN,     // %
  HUB_BATTERY_SOC_MAX,     // %
  HUB_GRID_EXPORT_LIMIT,   // W, hard cap enforced by the controller
  // Everything below was a compile time option until it became clear that
  // tuning a controller through recompile-and-flash cycles is its own kind of
  // obstacle. The YAML value is now the starting point; flash wins after that.
  HUB_UPDATE_INTERVAL,     // s, hub tick
  HUB_STEP_INTERVAL,       // s, how often the controller may act
  HUB_REFRESH_INTERVAL,    // s, setpoint rewrite even when unchanged
  HUB_AVERAGE_SAMPLES,     // count
  HUB_STALLED_TIMEOUT,     // s
  HUB_OFFLINE_TIMEOUT,     // s
  HUB_OFFLINE_PROBE,       // s
  HUB_IMPORT_THRESHOLD,    // W
  HUB_EXPORT_THRESHOLD,    // W
  HUB_INCREASE_GAIN,
  HUB_DECREASE_GAIN,
  HUB_MIN_STEP,            // %
  HUB_MAX_STEP,            // %
  HUB_OFFGRID_RATE,        // %
  HUB_PROTECTION_MARGIN,   // %
  HUB_RESTART_DELAY,       // s
  HUB_VOLTAGE_SOFT_MARGIN, // V
  HUB_SETTING_COUNT,
};

// Persistence layout. Every stored structure carries a version and a block of
// reserved bytes, so a new field can be added later by spending reserved space
// without changing sizeof - which is what makes the difference between "the new
// setting starts at its default" and "every stored setting is lost", since a
// size mismatch makes load() fail wholesale.
//
// Bump PREFS_VERSION only when the meaning of existing fields changes. Adding a
// field out of the reserved block does not need it.
// 2: the startup rate was removed and every setting after it shifted down one
// slot. Same size, different meaning - exactly what the version byte is for.
static const uint8_t PREFS_VERSION = 2;

struct GrowattHubPrefs {
  uint8_t version;
  uint8_t offline_action;
  float values[HUB_SETTING_COUNT];
  // Grew by the four bytes the startup rate used to occupy. Keeping sizeof
  // constant is what lets load() still succeed; the version byte is what stops
  // it misreading the shifted fields.
  uint8_t reserved[20];
} __attribute__((packed));

// What to do once the meter is definitively gone. Stopping is the safe default
// for an export limited site, because without the meter there is no way to know
// whether anything is being exported. It is also the most expensive: a comms
// fault at midday throws away real production. Holding trades that for the risk
// of exporting blind, and the third option splits the difference.
enum OfflineAction : uint8_t {
  OFF_STOP = 0,
  OFF_HOLD,
  OFF_DECAY,   // hold, then walk down to each inverter's safe rate
  OFF_ACTION_COUNT,
};

// Meter health. A single missed frame must not trigger anything dramatic, so
// there is a middle state between working and gone.
enum MeterHealth : uint8_t {
  METER_ONLINE = 0,
  METER_STALLED,
  METER_OFFLINE,
};

static const uint8_t HUB_AVG_MAX = 60;

// Owns the inverters and meters sharing one Modbus bus. Inverter order is
// significant: it is the priority order used by the power controller.
class GrowattHub : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA - 1; }

  void set_max_inverters(uint8_t n) { this->max_inverters_ = n; }
  void add_inverter(GrowattInverter *inv) { this->inverters_.push_back(inv); }
  void add_meter(GrowattMeter *m) { this->meters_.push_back(m); }

  GrowattInverter *at(size_t i) { return this->inverters_.at(i); }
  size_t size() const { return this->inverters_.size(); }
  const std::vector<GrowattInverter *> &all() const { return this->inverters_; }

  GrowattMeter *meter_at(size_t i) { return this->meters_.at(i); }
  size_t meter_count() const { return this->meters_.size(); }


  // ------------------------------ thresholds ------------------------------
  void set_setting(uint8_t field, float value);
  float get_setting(uint8_t field) const {
    return field < HUB_SETTING_COUNT ? this->values_[field] : NAN;
  }
  void set_setting_number(uint8_t field, number::Number *n) {
    if (field < HUB_SETTING_COUNT)
      this->setting_num_[field] = n;
  }
  void set_default(uint8_t field, float v) {
    if (field < HUB_SETTING_COUNT)
      this->values_[field] = v;
  }

  // Grid presence comes from an external contactor sensed on a GPIO and
  // declared in YAML. Without one the mains is assumed to be always available.
  void set_grid_power_source(binary_sensor::BinarySensor *b) {
    this->grid_power_bs_ = b;
  }
  bool grid_available() const;

  void set_offline_action(uint8_t a);
  uint8_t get_offline_action() const { return this->offline_action_; }
  void set_offline_action_select(select::Select *s) { this->offline_select_ = s; }
  void set_offline_hold(uint32_t ms) { this->offline_hold_ms_ = ms; }
  // How rarely an inverter that has gone offline is checked for a return. A
  // model without storage simply shuts down when the panels go dark, and each
  // pointless query costs more bus time in timeouts than a whole valid cycle.
  void set_avg_window(uint8_t n) {
    this->avg_window_ = (n == 0 || n > HUB_AVG_MAX) ? HUB_AVG_MAX : n;
  }

  uint8_t meter_health() const { return this->health_; }
  float get_import() const { return this->import_w_; }
  float get_export() const { return this->export_w_; }

  // ------------------------------ controller ------------------------------
  // Steps are proportional to the measured error rather than fixed, so a large
  // deficit is closed in a few cycles instead of dozens. The gains damp the
  // correction because the meter only reflects a change several seconds later.
  // They are asymmetric on purpose: overshooting upwards means exporting, which
  // costs money, while overshooting downwards only means importing a little
  // longer.
  void set_controller_state(text_sensor::TextSensor *ts) { this->ctrl_ts_ = ts; }

  // Widens each inverter's own trip window past the range the controller works
  // in, so we get a chance to reduce power before the hardware disconnects.
  // Voltage does not respond to a power change the way the meter reading does:
  // the grid impedance decides how far it moves, the neighbours contribute, and
  // on the PV side the MPPT needs tens of seconds to re-track after a limit is
  // lifted. So near the limit the proportional step is abandoned and increases
  // creep, rather than jumping by up to max_step and tripping straight back
  // over.

  // A single phase inverter sitting on the phase with the least headroom holds
  // back every three phase unit, because those are bound by the tightest
  // phase. Trading some of its output unlocks three times as much elsewhere.
  void set_rebalancing(bool v) { this->rebalance_ = v; }
  void set_rebalance_threshold(float w) { this->rebalance_threshold_ = w; }

  // ------------------------------ entities ------------------------------
  void set_meter_import(sensor::Sensor *s) { this->import_sens_ = s; }
  void set_meter_export(sensor::Sensor *s) { this->export_sens_ = s; }
  void set_meter_import_avg(sensor::Sensor *s) { this->import_avg_sens_ = s; }
  void set_meter_export_avg(sensor::Sensor *s) { this->export_avg_sens_ = s; }
  void set_meter_state(text_sensor::TextSensor *ts) { this->state_ts_ = ts; }

  // Mirrors what the controller believes about the mains, for observability.
  void set_grid_power_sensor(binary_sensor::BinarySensor *b) { this->bs_grid_ = b; }
  void set_grid_over_voltage(binary_sensor::BinarySensor *b) { this->bs_over_v_ = b; }
  void set_grid_under_voltage(binary_sensor::BinarySensor *b) { this->bs_under_v_ = b; }
  void set_ups_overloaded(binary_sensor::BinarySensor *b) { this->bs_ups_ = b; }
  void set_ups_overloaded_avg(binary_sensor::BinarySensor *b) { this->bs_ups_avg_ = b; }
  void set_battery_below_min(binary_sensor::BinarySensor *b) { this->bs_soc_lo_ = b; }
  void set_battery_above_max(binary_sensor::BinarySensor *b) { this->bs_soc_hi_ = b; }

 protected:
  void save_prefs_();
  void apply_setting_(uint8_t field);
  void publish_settings_();
  void update_meter_health_();
  void update_aggregates_();
  void update_conditions_();
  void control_power_();
  void set_all_(float pct, const char *reason);
  void refresh_all_();
  // How much more this inverter could deliver before the tightest phase starts
  // exporting. Single phase units only look at the phase they feed; three phase
  // units spread evenly, so they are bound by the worst phase and need three
  // times the correction to move one phase by a given amount.
  float headroom_up_(GrowattInverter *inv, const float *err);
  // How much this inverter must shed to clear the export on one specific
  // phase. Targeting the phase being fixed avoids over-correcting when several
  // phases are exporting by different amounts.
  float excess_on_(GrowattInverter *inv, const float *err, uint8_t phase);
  float step_for_(GrowattInverter *inv, float power_w, float gain);
  void set_ctrl_state_(const char *s);

  ESPPreferenceObject pref_;

  uint8_t max_inverters_{1};
  std::vector<GrowattInverter *> inverters_;
  std::vector<GrowattMeter *> meters_;

  float values_[HUB_SETTING_COUNT]{};
  number::Number *setting_num_[HUB_SETTING_COUNT]{};

  binary_sensor::BinarySensor *grid_power_bs_{nullptr};
  uint8_t offline_action_{OFF_STOP};
  uint32_t offline_hold_ms_{300000};
  uint32_t offline_since_{0};
  bool offline_logged_{false};
  select::Select *offline_select_{nullptr};
  uint32_t stalled_ms_{10000};
  uint32_t offline_ms_{20000};
  uint32_t offline_probe_ms_{60000};
  uint8_t health_{METER_OFFLINE};
  uint32_t meter_age_ms_{0};

  // Import and export are summed separately across phases: with no netting
  // between phases at the meter, +500 W on L1 and -300 W on L2 is billed as
  // 500 W drawn and 300 W given away, not 200 W net.
  float import_w_{NAN};
  float export_w_{NAN};
  float import_buf_[HUB_AVG_MAX]{};
  float export_buf_[HUB_AVG_MAX]{};
  uint8_t avg_pos_{0};
  uint8_t avg_count_{0};
  uint8_t avg_window_{HUB_AVG_MAX};

  sensor::Sensor *import_sens_{nullptr};
  sensor::Sensor *export_sens_{nullptr};
  sensor::Sensor *import_avg_sens_{nullptr};
  sensor::Sensor *export_avg_sens_{nullptr};
  text_sensor::TextSensor *state_ts_{nullptr};

  binary_sensor::BinarySensor *bs_grid_{nullptr};
  binary_sensor::BinarySensor *bs_over_v_{nullptr};
  binary_sensor::BinarySensor *bs_under_v_{nullptr};
  binary_sensor::BinarySensor *bs_ups_{nullptr};
  binary_sensor::BinarySensor *bs_ups_avg_{nullptr};
  binary_sensor::BinarySensor *bs_soc_lo_{nullptr};
  binary_sensor::BinarySensor *bs_soc_hi_{nullptr};

  // controller
  float import_threshold_{100.0f};
  float export_threshold_{0.0f};
  float increase_gain_{0.5f};
  float decrease_gain_{0.8f};
  float min_step_{1.0f};
  float max_step_{20.0f};
  uint32_t step_interval_{6000};
  uint32_t refresh_interval_{60000};
  float offgrid_rate_{100.0f};
  uint32_t last_step_{0};
  uint32_t last_refresh_{0};
  bool started_{false};
  const char *ctrl_state_{""};
  text_sensor::TextSensor *ctrl_ts_{nullptr};
  float voltage_soft_margin_{8.0f};
  float protection_margin_{10.0f};
  uint16_t restart_delay_s_{30};
  bool rebalance_{true};
  float rebalance_threshold_{300.0f};
};

// Behaviour when the meter is gone. Persisted under its own key.
class GrowattOfflineActionSelect : public select::Select {
 public:
  void set_parent(GrowattHub *p) { this->parent_ = p; }

 protected:
  void control(const std::string &value) override;
  GrowattHub *parent_{nullptr};
};

// Threshold entity. Values live in the hub and are persisted there.
class GrowattHubNumber : public number::Number {
 public:
  void set_parent(GrowattHub *p) { this->parent_ = p; }
  void set_field(uint8_t f) { this->field_ = f; }

 protected:
  void control(float value) override {
    this->publish_state(value);
    if (this->parent_ != nullptr)
      this->parent_->set_setting(this->field_, value);
  }
  GrowattHub *parent_{nullptr};
  uint8_t field_{0};
};

// Plain inputs for the address tool. They hold nothing but a number and are
// deliberately not persisted: both default to 0, and 0 is the Modbus broadcast
// address that no slave ever answers to, so a stray press does nothing.
class GrowattAddressNumber : public number::Number {
 public:
  void set_parent(GrowattAddressTool *p) { this->parent_ = p; }
  void set_is_target(bool t) { this->is_target_ = t; }

 protected:
  void control(float value) override {
    this->publish_state(value);
    if (this->parent_ == nullptr)
      return;
    uint8_t a = (uint8_t) lroundf(value);
    if (this->is_target_)
      this->parent_->set_to(a);
    else
      this->parent_->set_from(a);
  }
  GrowattAddressTool *parent_{nullptr};
  bool is_target_{false};
};

class GrowattAddressButton : public button::Button {
 public:
  void set_parent(GrowattAddressTool *p) { this->parent_ = p; }

 protected:
  void press_action() override {
    if (this->parent_ != nullptr)
      this->parent_->start();
  }
  GrowattAddressTool *parent_{nullptr};
};

}  // namespace growatt_master
}  // namespace esphome
