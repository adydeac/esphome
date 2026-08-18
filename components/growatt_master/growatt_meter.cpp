#include "growatt_meter.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstring>

namespace esphome {
namespace growatt_master {

static const char *const TAG = "growatt_meter";

static const uint32_t METER_TIMEOUT_MS = 1500;
static const uint8_t METER_MAX_RETRIES = 2;

// Matches BUS_YIELD_MS in growatt_inverter.cpp: step aside after our own
// transaction so the other devices on the bus get a turn.
static const uint32_t METER_BUS_YIELD_MS = 15;

// kinds accepted by set_phase_sensor, must match __init__.py
static const uint8_t MK_VOLTAGE = 0;
static const uint8_t MK_CURRENT = 1;
static const uint8_t MK_ACTIVE = 2;
static const uint8_t MK_APPARENT = 3;
static const uint8_t MK_REACTIVE = 4;
static const uint8_t MK_PF = 5;

static const char *const MODEL_NAMES[METER_MODEL_COUNT] = {
    "Auto", "SDM120", "SDM220", "SDM230", "SDM630",
};

// IEEE754 single precision, big endian (ABCD byte order).
static inline float fp32(const std::vector<uint8_t> &d, size_t reg) {
  uint32_t raw = ((uint32_t) d[reg * 2] << 24) | ((uint32_t) d[reg * 2 + 1] << 16) |
                 ((uint32_t) d[reg * 2 + 2] << 8) | (uint32_t) d[reg * 2 + 3];
  float f;
  memcpy(&f, &raw, sizeof(f));
  return f;
}

static inline void pubf(sensor::Sensor *s, const std::vector<uint8_t> &d,
                        size_t reg) {
  if (s != nullptr)
    s->publish_state(fp32(d, reg));
}

static inline void pub_val_m(sensor::Sensor *s, float v) {
  if (s != nullptr)
    s->publish_state(v);
}

static inline uint16_t reg16m(const std::vector<uint8_t> &d, size_t reg) {
  return encode_uint16(d[reg * 2], d[reg * 2 + 1]);
}

// ============================== GrowattMeter ==============================

void GrowattMeter::setup() {
  uint32_t hash = fnv1_hash("growatt_meter_" + std::to_string(this->slot_index_));
  this->pref_ = global_preferences->make_preference<MeterPrefs>(hash);

  MeterPrefs p{};
  if (this->pref_.load(&p)) {
    if (p.version == PREFS_VERSION) {
      this->address_ = p.address;
      this->model_ = (p.model < METER_MODEL_COUNT) ? p.model
                                                   : (uint8_t) METER_AUTO;
      if (p.update_interval > 0)
        this->set_update_interval((uint32_t) p.update_interval * 1000);
      if (p.slow_interval > 0)
        this->slow_interval_ = (uint32_t) p.slow_interval * 1000;
      ESP_LOGI(TAG, "meter %u: restored addr=%u model=%s", this->slot_index_,
               p.address, MODEL_NAMES[this->model_]);
    } else {
      ESP_LOGW(TAG, "meter %u: stored settings are version %u, expected %u - "
               "using defaults", this->slot_index_, p.version, PREFS_VERSION);
    }
  }

  this->publish_cfg_entities_();

  if (!this->is_enabled()) {
    ESP_LOGCONFIG(TAG, "meter %u: address 0 -> not present", this->slot_index_);
    this->step_ = MSTEP_DONE;
    return;
  }
  ESP_LOGI(TAG, "meter %u @addr %u: starting identification...",
           this->slot_index_, this->address_);
}

void GrowattMeter::apply_update_interval(float seconds) {
  uint32_t ms = (uint32_t) (seconds * 1000.0f);
  if (ms < 200)
    return;
  // A PollingComponent does not notice a new interval by itself.
  this->stop_poller();
  this->set_update_interval(ms);
  this->start_poller();
  this->save_prefs_();
  ESP_LOGI(TAG, "meter %u: poll interval %u ms", this->slot_index_,
           (unsigned) ms);
}

void GrowattMeter::apply_slow_interval(float seconds) {
  uint32_t ms = (uint32_t) (seconds * 1000.0f);
  if (ms < 1000)
    return;
  this->slow_interval_ = ms;
  this->save_prefs_();
  ESP_LOGI(TAG, "meter %u: slow block interval %u ms", this->slot_index_,
           (unsigned) ms);
}

void GrowattMeterIntervalNumber::control(float value) {
  this->publish_state(value);
  if (this->parent_ == nullptr)
    return;
  if (this->is_slow_)
    this->parent_->apply_slow_interval(value);
  else
    this->parent_->apply_update_interval(value);
}

void GrowattMeter::publish_cfg_entities_() {
  if (this->address_num_ != nullptr)
    this->address_num_->publish_state(this->address_);
  if (this->update_num_ != nullptr)
    this->update_num_->publish_state(this->get_update_interval() / 1000.0f);
  if (this->slow_num_ != nullptr)
    this->slow_num_->publish_state(this->slow_interval_ / 1000.0f);
  if (this->model_select_ != nullptr)
    this->model_select_->publish_state(MODEL_NAMES[this->model_]);
}

void GrowattMeter::save_prefs_() {
  MeterPrefs p{};
  p.version = PREFS_VERSION;
  p.address = this->address_;
  p.model = this->model_;
  p.update_interval = (uint16_t) (this->get_update_interval() / 1000);
  p.slow_interval = (uint16_t) (this->slow_interval_ / 1000);
  this->pref_.save(&p);
}

void GrowattMeter::change_address(uint8_t addr) {
  if (addr == this->address_)
    return;
  ESP_LOGI(TAG, "meter %u: address %u -> %u", this->slot_index_, this->address_,
           addr);
  this->address_ = addr;
  this->publish_cfg_entities_();
  this->waiting_ = false;
  this->poll_ = MPOLL_IDLE;
  this->save_prefs_();
  if (addr == 0) {
    this->step_ = MSTEP_DONE;
    this->want_send_ = false;
  } else {
    this->restart_identification();
  }
}

void GrowattMeter::set_model(uint8_t m) {
  if (m >= METER_MODEL_COUNT)
    return;
  this->model_ = m;
  this->save_prefs_();
  // A forced model fixes the phase count; Auto hands it back to detection.
  switch (m) {
    case METER_SDM120:
    case METER_SDM220:
    case METER_SDM230:
      this->phases_ = 1;
      break;
    case METER_SDM630:
      this->phases_ = 3;
      break;
    default:
      this->phases_ = 0;  // re-detect
      break;
  }
  ESP_LOGI(TAG, "meter %u: model set to %s", this->slot_index_, MODEL_NAMES[m]);
  this->restart_identification();
}

void GrowattMeter::restart_identification() {
  if (!this->is_enabled()) {
    ESP_LOGW(TAG, "meter %u: cannot identify, address is 0", this->slot_index_);
    return;
  }
  this->step_ = MSTEP_START;
  this->poll_ = MPOLL_IDLE;
  this->retries_ = 0;
  this->waiting_ = false;
  this->want_send_ = true;
}

void GrowattMeter::set_phase_sensor(uint8_t i, uint8_t kind, sensor::Sensor *s) {
  if (i >= 3)
    return;
  MeterTriple &t = this->phases_sens_[i];
  switch (kind) {
    case MK_VOLTAGE:  t.voltage = s; break;
    case MK_CURRENT:  t.current = s; break;
    case MK_ACTIVE:   t.active_power = s; break;
    case MK_APPARENT: t.apparent_power = s; break;
    case MK_REACTIVE: t.reactive_power = s; break;
    default:          t.power_factor = s; break;
  }
}

void GrowattMeter::set_line_voltage(uint8_t i, sensor::Sensor *s) {
  if (i < 3)
    this->line_voltages_[i] = s;
}

// Extra blocks cost a Modbus transaction each, so they are only requested when
// at least one of their sensors is actually declared.
bool GrowattMeter::needs_line_block_() const {
  if (this->avg_line_v_ != nullptr || this->neutral_i_ != nullptr)
    return true;
  for (uint8_t i = 0; i < 3; i++)
    if (this->line_voltages_[i] != nullptr)
      return true;
  return false;
}

bool GrowattMeter::needs_energy_block_() const {
  return this->total_active_e_ != nullptr || this->total_reactive_e_ != nullptr;
}

// ------------------------------ scheduling ------------------------------

void GrowattMeter::update() {
  if (!this->is_enabled() || this->waiting_)
    return;
  if (this->step_ != MSTEP_DONE) {
    this->want_send_ = true;
    this->try_send_();
    return;
  }
  this->start_poll_();
}

void GrowattMeter::start_poll_() {
  uint32_t now = millis();
  this->slow_due_ = (this->last_slow_ == 0) ||
                    (now - this->last_slow_ >= this->slow_interval_);
  if (this->slow_due_)
    this->last_slow_ = now;

  this->poll_ = MPOLL_FAST;
  this->retries_ = 0;
  this->want_send_ = true;
  this->try_send_();
}

void GrowattMeter::try_send_() {
  if (millis() - this->bus_release_ < METER_BUS_YIELD_MS)
    return;
  if (!this->ready_for_immediate_send()) {
    if (!this->busy_logged_) {
      ESP_LOGV(TAG, "meter %u: bus busy, will retry", this->slot_index_);
      this->busy_logged_ = true;
    }
    return;
  }
  this->want_send_ = false;
  this->busy_logged_ = false;
  if (this->poll_ != MPOLL_IDLE)
    this->send_poll_();
  else
    this->send_step_();
}

void GrowattMeter::loop() {
  if (this->want_send_ && !this->waiting_)
    this->try_send_();

  if (!this->waiting_)
    return;
  if (millis() - this->last_send_ < METER_TIMEOUT_MS)
    return;

  this->waiting_ = false;
  this->retries_++;
  this->bus_release_ = millis();

  if (this->retries_ <= METER_MAX_RETRIES) {
    ESP_LOGD(TAG, "meter %u: timeout, retrying (%u/%u)", this->slot_index_,
             this->retries_, METER_MAX_RETRIES);
    this->want_send_ = true;
    return;
  }

  if (this->poll_ != MPOLL_IDLE) {
    ESP_LOGW(TAG, "meter %u: no answer on poll block %u", this->slot_index_,
             this->poll_);
    this->advance_poll_();
  } else {
    ESP_LOGW(TAG, "meter %u: no answer on step %u", this->slot_index_,
             this->step_);
    this->advance_(false);
  }
}

void GrowattMeter::send_step_() {
  switch (this->step_) {
    case MSTEP_START:
      this->step_ = MSTEP_ID;
      // fallthrough
    case MSTEP_ID:
      this->send(CMD_READ_HOLDING, SDM_ID_BASE, SDM_ID_CNT);
      break;
    case MSTEP_MAIN:
      this->send(CMD_READ_INPUT, SDM_MAIN_BASE, SDM_MAIN_CNT);
      break;
    default:
      return;
  }
  this->last_send_ = millis();
  this->waiting_ = true;
}

void GrowattMeter::send_poll_() {
  switch (this->poll_) {
    case MPOLL_FAST:
      this->send(CMD_READ_INPUT, SDM_FAST_BASE, SDM_FAST_CNT);
      break;
    case MPOLL_SLOW:
      this->send(CMD_READ_INPUT, SDM_SLOW_BASE, SDM_SLOW_CNT);
      break;
    case MPOLL_LINE:
      this->send(CMD_READ_INPUT, SDM_LINE_BASE, SDM_LINE_CNT);
      break;
    case MPOLL_ENERGY:
      this->send(CMD_READ_INPUT, SDM_ENERGY_BASE, SDM_ENERGY_CNT);
      break;
    default:
      return;
  }
  this->last_send_ = millis();
  this->waiting_ = true;
}

void GrowattMeter::advance_(bool ok) {
  this->retries_ = 0;
  this->waiting_ = false;
  switch (this->step_) {
    case MSTEP_ID:
      this->step_ = MSTEP_MAIN;
      break;
    case MSTEP_MAIN:
      this->step_ = MSTEP_DONE;
      this->publish_info_();
      break;
    default:
      break;
  }
  this->want_send_ = (this->step_ != MSTEP_DONE);
}

void GrowattMeter::advance_poll_() {
  this->retries_ = 0;
  this->waiting_ = false;
  switch (this->poll_) {
    case MPOLL_FAST:
      this->poll_ = this->slow_due_ ? MPOLL_SLOW : MPOLL_IDLE;
      break;
    case MPOLL_SLOW:
      // Line to line voltages only make sense on a three phase meter.
      if (this->phases_ >= 3 && this->needs_line_block_())
        this->poll_ = MPOLL_LINE;
      else if (this->needs_energy_block_())
        this->poll_ = MPOLL_ENERGY;
      else
        this->poll_ = MPOLL_IDLE;
      break;
    case MPOLL_LINE:
      this->poll_ = this->needs_energy_block_() ? MPOLL_ENERGY : MPOLL_IDLE;
      break;
    default:
      this->poll_ = MPOLL_IDLE;
      break;
  }
  this->want_send_ = (this->poll_ != MPOLL_IDLE);
}

// ------------------------------ parsing ------------------------------

void GrowattMeter::detect_phases_(const std::vector<uint8_t> &data) {
  // A forced model already decided the phase count.
  if (this->model_ != METER_AUTO)
    return;

  uint8_t n = 0;
  for (uint8_t i = 0; i < 3; i++) {
    if (fp32(data, SDM_V[i]) >= SDM_VOLTAGE_PRESENT)
      n++;
  }
  if (n == 0) {
    ESP_LOGW(TAG, "meter %u: no voltage on any phase", this->slot_index_);
    return;
  }
  uint8_t detected = (n >= 2) ? 3 : 1;
  if (detected != this->phases_) {
    this->phases_ = detected;
    ESP_LOGI(TAG, "meter %u: %u voltage(s) present -> %u phase(s)",
             this->slot_index_, n, this->phases_);
  }
}

// Fast part, input 0x0000..0x0035: per phase values plus total active power.
void GrowattMeter::parse_main_(const std::vector<uint8_t> &data) {
  for (uint8_t i = 0; i < 3; i++) {
    MeterTriple &t = this->phases_sens_[i];
    this->phase_voltage_[i] = fp32(data, SDM_V[i]);
    this->phase_power_[i] = fp32(data, SDM_P[i]);
    pub_val_m(t.voltage, this->phase_voltage_[i]);
    pubf(t.current, data, SDM_I[i]);
    pub_val_m(t.active_power, this->phase_power_[i]);
    pubf(t.apparent_power, data, SDM_S[i]);
    pubf(t.reactive_power, data, SDM_Q[i]);
    pubf(t.power_factor, data, SDM_PF[i]);
  }
  pubf(this->avg_v_, data, SDM_AVG_V);
  pubf(this->avg_i_, data, SDM_AVG_I);
  pubf(this->sum_i_, data, SDM_SUM_I);
  pubf(this->total_p_, data, SDM_TOTAL_P);
}

// Slow part, input 0x0036..0x004F: totals, frequency and energy counters.
void GrowattMeter::parse_slow_(const std::vector<uint8_t> &data) {
  const uint8_t B = SDM_SLOW_BASE;  // rebase absolute offsets
  pubf(this->total_s_, data, SDM_TOTAL_S - B);
  pubf(this->total_q_, data, SDM_TOTAL_Q - B);
  pubf(this->total_pf_, data, SDM_TOTAL_PF - B);
  pubf(this->frequency_, data, SDM_FREQ - B);
  pubf(this->import_e_, data, SDM_IMPORT_E - B);
  pubf(this->export_e_, data, SDM_EXPORT_E - B);
  pubf(this->import_re_, data, SDM_IMPORT_RE - B);
  pubf(this->export_re_, data, SDM_EXPORT_RE - B);
}

void GrowattMeter::parse_line_(const std::vector<uint8_t> &data) {
  for (uint8_t i = 0; i < 3; i++)
    pubf(this->line_voltages_[i], data, SDM_LINE_V[i]);
  pubf(this->avg_line_v_, data, SDM_LINE_AVG);
  pubf(this->neutral_i_, data, SDM_NEUTRAL_I);
}

void GrowattMeter::parse_energy_(const std::vector<uint8_t> &data) {
  pubf(this->total_active_e_, data, SDM_TOTAL_ACTIVE_E);
  pubf(this->total_reactive_e_, data, SDM_TOTAL_REACTIVE_E);
}

void GrowattMeter::publish_info_() {
  char buf[96];
  snprintf(buf, sizeof(buf), "%s | %uph | id %04X %04X %04X %04X",
           MODEL_NAMES[this->model_], this->phases_, this->id_words_[0],
           this->id_words_[1], this->id_words_[2], this->id_words_[3]);
  ESP_LOGI(TAG, "meter %u @addr %u IDENTIFIED: %s", this->slot_index_,
           this->address_, buf);
  if (this->info_ts_ != nullptr)
    this->info_ts_->publish_state(std::string(buf));
}

// ------------------------------ responses ------------------------------

void GrowattMeter::on_modbus_error(uint8_t function_code,
                                   uint8_t exception_code) {
  if (!this->is_enabled() || !this->waiting_)
    return;
  this->last_update_ = micros();
  this->waiting_ = false;
  this->bus_release_ = millis();
  ESP_LOGD(TAG, "meter %u: exception %u on function 0x%02X", this->slot_index_,
           exception_code, function_code);
  if (this->poll_ != MPOLL_IDLE)
    this->advance_poll_();
  else
    this->advance_(false);
}

void GrowattMeter::on_modbus_data(const std::vector<uint8_t> &data) {
  if (!this->is_enabled())
    return;
  this->last_update_ = micros();

  if (!this->waiting_)
    return;
  this->waiting_ = false;
  this->bus_release_ = millis();

  if (this->poll_ != MPOLL_IDLE) {
    if (this->poll_ == MPOLL_FAST && data.size() >= SDM_FAST_CNT * 2) {
      this->detect_phases_(data);
      this->parse_main_(data);
    } else if (this->poll_ == MPOLL_SLOW && data.size() >= SDM_SLOW_CNT * 2) {
      this->parse_slow_(data);
    } else if (this->poll_ == MPOLL_LINE && data.size() >= SDM_LINE_CNT * 2) {
      this->parse_line_(data);
    } else if (this->poll_ == MPOLL_ENERGY && data.size() >= SDM_ENERGY_CNT * 2) {
      this->parse_energy_(data);
    }
    this->advance_poll_();
    return;
  }

  switch (this->step_) {
    case MSTEP_ID:
      if (data.size() < SDM_ID_CNT * 2) { this->advance_(false); return; }
      // The meaning of these words is not confirmed across models, so they are
      // recorded and shown raw until a documented mapping is available.
      for (uint8_t i = 0; i < SDM_ID_CNT; i++)
        this->id_words_[i] = reg16m(data, i);
      ESP_LOGI(TAG, "meter %u: id block %04X %04X %04X %04X", this->slot_index_,
               this->id_words_[0], this->id_words_[1], this->id_words_[2],
               this->id_words_[3]);
      break;
    case MSTEP_MAIN:
      if (data.size() < SDM_MAIN_CNT * 2) { this->advance_(false); return; }
      this->detect_phases_(data);
      this->parse_main_(data);
      break;
    default:
      return;
  }
  this->advance_(true);
}

void GrowattMeter::dump_config() {
  ESP_LOGCONFIG(TAG, "Growatt Meter %u:", this->slot_index_);
  ESP_LOGCONFIG(TAG, "  Address: %u%s", this->address_,
                this->is_enabled() ? "" : " (NOT PRESENT)");
  ESP_LOGCONFIG(TAG, "  Model: %s", MODEL_NAMES[this->model_]);
}

void GrowattMeterAddressNumber::control(float value) {
  this->publish_state(value);
  if (this->parent_ != nullptr)
    this->parent_->change_address((uint8_t) lroundf(value));
}

// ------------------------- GrowattMeterModelSelect -------------------------

void GrowattMeterModelSelect::control(const std::string &value) {
  this->publish_state(value);
  if (this->parent_ == nullptr)
    return;
  for (uint8_t i = 0; i < METER_MODEL_COUNT; i++) {
    if (value == MODEL_NAMES[i]) {
      this->parent_->set_model(i);
      return;
    }
  }
}

}  // namespace growatt_master
}  // namespace esphome
