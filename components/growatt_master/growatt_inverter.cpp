#include "growatt_inverter.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cmath>

namespace esphome {
namespace growatt_master {

static const char *const TAG = "growatt_inverter";

// Must match CONVENTIONS in __init__.py.
static const char *const CONV_NAMES[CONV_MODE_COUNT] = {"Auto", "Phase", "Line"};


// Growatt doc page 8: minimum 850ms between commands, 1s suggested.
static const uint32_t IDENT_TIMEOUT_MS = 1500;
static const uint8_t IDENT_MAX_RETRIES = 2;

// After finishing a transaction a device waits this long before asking for the
// bus again. ESPHome runs loop() in registration order, so without it the
// first component would chain its blocks back to back and starve the rest.
// A few milliseconds are enough: loop() runs thousands of times a second.
static const uint32_t BUS_YIELD_MS = 15;

// Sensor kinds used by the indexed setters, must match __init__.py
static const uint8_t KIND_VOLTAGE = 0;
static const uint8_t KIND_CURRENT = 1;
static const uint8_t KIND_POWER = 2;

static inline uint16_t reg16(const std::vector<uint8_t> &d, size_t reg) {
  return encode_uint16(d[reg * 2], d[reg * 2 + 1]);
}

static inline uint32_t reg32(const std::vector<uint8_t> &d, size_t reg) {
  return (((uint32_t) reg16(d, reg)) << 16) | reg16(d, reg + 1);
}

static inline void pub1(sensor::Sensor *s, const std::vector<uint8_t> &d,
                        size_t reg, float unit) {
  if (s != nullptr)
    s->publish_state(reg16(d, reg) * unit);
}

static inline void pub2(sensor::Sensor *s, const std::vector<uint8_t> &d,
                        size_t reg, float unit) {
  if (s != nullptr)
    s->publish_state(reg32(d, reg) * unit);
}

static inline void pub_val(sensor::Sensor *s, float v) {
  if (s != nullptr)
    s->publish_state(v);
}

// Extract printable ASCII from a register range, trailing spaces removed.
static std::string ascii_from(const std::vector<uint8_t> &d, size_t reg,
                              size_t count) {
  std::string s;
  for (size_t i = 0; i < count * 2; i++) {
    char c = (char) d[reg * 2 + i];
    if (c >= 32 && c <= 126)
      s += c;
  }
  while (!s.empty() && s.back() == ' ')
    s.pop_back();
  return s;
}

static inline void pub_text(text_sensor::TextSensor *ts, const std::string &v) {
  if (ts != nullptr)
    ts->publish_state(v);
}

// Inverter run state. Codes 0..3 come from the protocol document; the rest were
// established by observing storage units in the field.
static const char *status_text(uint16_t code) {
  switch (code) {
    case 0: return "Waiting";
    case 1: return "Normal";
    case 2: return "Discharge";
    case 3: return "Fault";
    case 4: return "Permanent Fault / Flash";
    case 5: return "PV Charging / Standby";
    case 6: return "PV Off / Night";
    case 7: return "Off Grid / PV Charging";
    case 8: return "Off Grid / PV Off / Night";
    case 9: return "Island Mode";
    case 10: return "AC Charging & Bypass";
    case 11: return "Bypass";
    case 12: return "PV Charge and Discharge";
    default: return "Unknown";
  }
}

// Codes 7, 8 and 9 mean the inverter is running disconnected from the mains.
static inline bool status_is_off_grid(uint16_t code) {
  return code >= 7 && code <= 9;
}

static std::string fault_text(uint16_t code) {
  switch (code) {
    case 0: return "No error";
    case 24: return "Auto test failed";
    case 25: return "No AC connection";
    case 26: return "PV isolation low";
    case 27: return "Residual current high";
    case 28: return "Output DC current high";
    case 29: return "PV voltage high";
    case 30: return "AC voltage out of range";
    case 31: return "AC frequency out of range";
    case 32: return "Module temperature high";
    default: return "Fault code " + std::to_string(code);
  }
}

// Below this the panels are not delivering anything worth chasing, so raising
// the setpoint would only walk the inverter up to 100 % for nothing.
static const float MIN_PV_POWER_W = 50.0f;

// Holding address and engineering scale of every editable setting. Order must
// match SettingField in the header. The UI works in engineering units while
// the register keeps its raw encoding.
static const uint16_t SETTING_ADDR[SET_COUNT] = {
    REG_ACTIVE_POWER_RATE, HO_GF_DISCHARGE_RATE, HO_GF_STOP_SOC,
    HO_BF_CHARGE_RATE,     HO_BF_STOP_SOC,       HO_PV_START_VOLT,
    HO_START_TIME,         HO_RESTART_DELAY,     HO_GRID_V_LOW,
    HO_GRID_V_HIGH,        HO_GRID_F_LOW,        HO_GRID_F_HIGH,
    HO_EXPORT_LIMIT_RATE,
};
static const float SETTING_SCALE[SET_COUNT] = {
    1.0f,    1.0f,    1.0f,    1.0f,    1.0f, ONE_DEC, 1.0f,
    1.0f,    ONE_DEC, ONE_DEC, TWO_DEC, TWO_DEC, ONE_DEC,
};

// Why the inverter is limiting itself. Mode 7 is our own command and is the
// normal state whenever the setpoint is below 100 %.
static const char *derating_text(uint8_t m) {
  switch (m) {
    case 0: return "none";
    case 1: return "PV";
    case 3: return "grid voltage";
    case 4: return "grid frequency";
    case 5: return "boost temperature";
    case 6: return "inverter temperature";
    case 7: return "our command";
    case 9: return "over back by time";
    default: return "other";
  }
}

// ============================ GrowattInverter ============================

void GrowattInverter::setup() {
  // Flash wins over the YAML defaults: whatever was last set from the UI is
  // what the user expects to find after a reboot.
  uint32_t hash = fnv1_hash("growatt_slot_" + std::to_string(this->slot_index_));
  this->pref_ = global_preferences->make_preference<GrowattSlotPrefs>(hash);

  GrowattSlotPrefs p{};
  if (this->pref_.load(&p)) {
    if (p.version == PREFS_VERSION) {
      this->address_ = p.address;
      this->cfg_phases_ = p.cfg_phases;
      this->cfg_strings_ = p.cfg_strings;
      this->phase_ = (p.phase > INV_PHASE_L3) ? (uint8_t) INV_PHASE_L1 : p.phase;
      if (p.safe_power_rate <= 100)
        this->safe_power_rate_ = p.safe_power_rate;
      if (p.max_power_rate > 0 && p.max_power_rate <= 100) {
        this->min_power_rate_ = p.min_power_rate;
        this->max_power_rate_ = p.max_power_rate;
      }
      if (p.convention < CONV_MODE_COUNT)
        this->cfg_convention_ = p.convention;
      this->auto_protection_ = p.auto_protection != 0;
      this->protect_eeprom_ = p.protect_eeprom != 0;
      if (p.update_interval > 0)
        this->set_update_interval((uint32_t) p.update_interval * 1000);
      if (p.slow_interval > 0)
        this->slow_interval_ = (uint32_t) p.slow_interval * 1000;
      ESP_LOGI(TAG,
               "slot %u: restored addr=%u phases=%d strings=%d wired=L%u safe=%u%%",
               this->slot_index_, p.address, p.cfg_phases, p.cfg_strings,
               this->phase_ + 1, this->safe_power_rate_);
    } else {
      ESP_LOGW(TAG, "slot %u: stored settings are version %u, expected %u - "
               "using defaults", this->slot_index_, p.version, PREFS_VERSION);
    }
  }
  if (this->safe_rate_num_ != nullptr)
    this->safe_rate_num_->publish_state(this->safe_power_rate_);
  if (this->min_rate_num_ != nullptr)
    this->min_rate_num_->publish_state(this->min_power_rate_);
  if (this->max_rate_num_ != nullptr)
    this->max_rate_num_->publish_state(this->max_power_rate_);
  if (this->update_num_ != nullptr)
    this->update_num_->publish_state(this->get_update_interval() / 1000.0f);
  if (this->slow_num_ != nullptr)
    this->slow_num_->publish_state(this->slow_interval_ / 1000.0f);
  if (this->auto_prot_sw_ != nullptr)
    this->auto_prot_sw_->publish_state(this->auto_protection_);
  if (this->eeprom_sw_ != nullptr)
    this->eeprom_sw_->publish_state(this->protect_eeprom_);
  if (this->convention_select_ != nullptr &&
      this->cfg_convention_ < CONV_MODE_COUNT)
    this->convention_select_->publish_state(CONV_NAMES[this->cfg_convention_]);

  // After the restore, so the selects show what is actually in force rather
  // than the YAML defaults. Done before the empty slot exit: a slot with no
  // address still has overrides worth displaying.
  this->publish_cfg_entities_();

  if (!this->is_enabled()) {
    ESP_LOGCONFIG(TAG, "slot %u: address 0 -> empty slot, will not be polled",
                  this->slot_index_);
    this->step_ = IDENT_DONE;
    return;
  }
  ESP_LOGI(TAG, "slot %u @addr %u: starting identification...", this->slot_index_,
           this->address_);
}

void GrowattInverter::save_prefs_() {
  GrowattSlotPrefs p{};
  p.version = PREFS_VERSION;
  p.address = this->address_;
  p.cfg_phases = this->cfg_phases_;
  p.cfg_strings = this->cfg_strings_;
  p.phase = this->phase_;
  p.safe_power_rate = this->safe_power_rate_;
  p.min_power_rate = this->min_power_rate_;
  p.max_power_rate = this->max_power_rate_;
  p.convention = this->cfg_convention_;
  p.auto_protection = this->auto_protection_ ? 1 : 0;
  p.protect_eeprom = this->protect_eeprom_ ? 1 : 0;
  p.update_interval = (uint16_t) (this->get_update_interval() / 1000);
  p.slow_interval = (uint16_t) (this->slow_interval_ / 1000);
  this->pref_.save(&p);
}

// Option strings, which must match PHASE_COUNT_OPTIONS and STRING_OPTIONS in
// __init__.py. Nothing checks that at build time.
//
// Not named CFG_*: the header already has CFG_AUTO, an int8_t sentinel for the
// UPS and battery overrides, and these are a different kind of thing entirely.
// Note the two conventions do not match either - phases and strings use 0 for
// automatic, while CFG_AUTO is -1.
static const char *const OPT_AUTO = "Auto";
static const char *const OPT_SINGLE_PHASE = "Single phase";
static const char *const OPT_THREE_PHASE = "Three phase";

void GrowattInverter::publish_cfg_entities_() {
  if (this->address_num_ != nullptr)
    this->address_num_->publish_state(this->address_);
  if (this->phase_count_select_ != nullptr) {
    const char *v = OPT_AUTO;
    if (this->cfg_phases_ == 1)
      v = OPT_SINGLE_PHASE;
    else if (this->cfg_phases_ == 3)
      v = OPT_THREE_PHASE;
    this->phase_count_select_->publish_state(v);
  }
  if (this->strings_select_ != nullptr) {
    this->strings_select_->publish_state(
        this->cfg_strings_ > 0 ? std::to_string((int) this->cfg_strings_)
                               : std::string(OPT_AUTO));
  }
}

void GrowattInverter::set_cfg_phases(int8_t v) {
  this->cfg_phases_ = v;
  this->save_prefs_();
}

void GrowattInverter::set_cfg_strings(int8_t v) {
  this->cfg_strings_ = v;
  this->save_prefs_();
}

static inline void assign_triple(SensorTriple &t, uint8_t kind, sensor::Sensor *s) {
  if (kind == KIND_VOLTAGE) t.voltage = s;
  else if (kind == KIND_CURRENT) t.current = s;
  else t.power = s;
}

void GrowattInverter::set_phase_sensor(uint8_t i, uint8_t kind, sensor::Sensor *s) {
  if (i < 3)
    assign_triple(this->phases_[i], kind, s);
}
void GrowattInverter::set_pv_sensor(uint8_t i, uint8_t kind, sensor::Sensor *s) {
  if (i < MAX_STRINGS)
    assign_triple(this->pvs_[i], kind, s);
}
void GrowattInverter::set_ups_sensor(uint8_t i, uint8_t kind, sensor::Sensor *s) {
  if (i < 3)
    assign_triple(this->ups_[i], kind, s);
}
void GrowattInverter::set_line_voltage(uint8_t i, sensor::Sensor *s) {
  if (i < 3)
    this->line_voltages_[i] = s;
}
void GrowattInverter::set_pv_energy_today(uint8_t i, sensor::Sensor *s) {
  if (i < MAX_STRINGS)
    this->pv_energy_today_[i] = s;
}
void GrowattInverter::set_pv_energy_total(uint8_t i, sensor::Sensor *s) {
  if (i < MAX_STRINGS)
    this->pv_energy_total_[i] = s;
}

// ------------------------------ public API ------------------------------

void GrowattInverter::change_address(uint8_t addr) {
  if (addr == this->address_)
    return;
  ESP_LOGI(TAG, "slot %u: address %u -> %u", this->slot_index_, this->address_,
           addr);
  this->address_ = addr;
  this->publish_cfg_entities_();
  this->waiting_ = false;
  this->dump_active_ = false;
  this->poll_ = POLL_IDLE;
  this->save_prefs_();
  if (addr == 0) {
    this->step_ = IDENT_DONE;
    this->want_send_ = false;
  } else {
    this->restart_identification();
  }
}

// Identification only runs at boot, on an address change, or when the user
// asks for it. A run that could not complete is retried a few times and then
// left alone rather than hammering the bus indefinitely.
static const uint32_t IDENT_RETRY_MS = 60000;
static const uint8_t IDENT_MAX_RUNS = 3;

void GrowattInverter::begin_identification_() {
  this->step_ = IDENT_START;
  this->poll_ = POLL_IDLE;
  this->retries_ = 0;
  this->waiting_ = false;
  this->dump_active_ = false;
  this->ident_incomplete_ = false;
  this->ident_retry_at_ = 0;
  this->protection_applied_ = false;  // limits are re-applied on every run
  this->pac_is_total_ = false;        // re-detected from the next live block
  this->pac_total_hits_ = 0;
  this->nameplate_revised_ = false;
  this->rejected_count_ = 0;  // a fresh look includes what it will accept
  this->caps_ = GrowattCaps{};        // clears the remembered string count too
  this->want_send_ = true;
}

void GrowattInverter::restart_identification() {
  if (!this->is_enabled()) {
    ESP_LOGW(TAG, "slot %u: cannot identify, address is 0", this->slot_index_);
    return;
  }
  ESP_LOGI(TAG, "slot %u: restarting identification", this->slot_index_);
  this->ident_runs_ = 0;
  this->begin_identification_();
}

void GrowattInverter::start_dump() {
  if (!this->is_enabled()) {
    ESP_LOGW(TAG, "slot %u: cannot dump, address is 0", this->slot_index_);
    return;
  }
  ESP_LOGI(TAG, "slot %u @addr %u: === REGISTER DUMP START ===",
           this->slot_index_, this->address_);
  this->dump_active_ = true;
  this->dump_range_ = 0;
  this->dump_offset_ = 0;
  this->poll_ = POLL_IDLE;
  this->waiting_ = false;
  this->want_send_ = true;
}

// ------------------------------ scheduling ------------------------------

const char *GrowattInverter::health_text() const {
  switch (this->health_) {
    case INV_ONLINE: return "online";
    case INV_STALLED: return "stalled";
    default: return "offline";
  }
}

void GrowattInverter::update_health_() {
  uint32_t now = millis();
  uint8_t h;
  if (this->last_update_ == 0) {
    // Nothing heard yet. Assume it is there long enough for identification to
    // get a chance, then give up if it never answers.
    h = (now > this->offline_ms_) ? INV_OFFLINE : INV_ONLINE;
  } else {
    uint32_t age = (micros() - this->last_update_) / 1000;
    if (age < this->stalled_ms_)
      h = INV_ONLINE;
    else if (age < this->offline_ms_)
      h = INV_STALLED;
    else
      h = INV_OFFLINE;
  }
  if (h == this->health_)
    return;

  uint8_t was = this->health_;
  this->health_ = h;
  if (this->state_ts_ != nullptr)
    this->state_ts_->publish_state(this->health_text());

  if (h == INV_OFFLINE) {
    ESP_LOGW(TAG, "slot %u went offline, backing off to a probe every %u s",
             this->slot_index_, (unsigned) (this->offline_probe_ms_ / 1000));
    this->waiting_ = false;
    this->want_send_ = false;
    this->poll_ = POLL_IDLE;
    this->last_probe_ = now;
  } else if (was == INV_OFFLINE) {
    // Back from the dead. Everything it was told may have been lost across a
    // power cycle, so identify again, which also re-applies the trip limits.
    ESP_LOGI(TAG, "slot %u is back, re-identifying", this->slot_index_);
    this->probing_ = false;
    this->ident_runs_ = 0;
    this->begin_identification_();
  } else {
    ESP_LOGD(TAG, "slot %u is %s", this->slot_index_, this->health_text());
  }
}

void GrowattInverter::send_probe_() {
  this->send(CMD_READ_INPUT, PROBE_BASE, PROBE_CNT);
  this->last_send_ = millis();
  this->waiting_ = true;
  ESP_LOGV(TAG, "slot %u: probing", this->slot_index_);
}

void GrowattInverter::update() {
  if (!this->is_enabled())
    return;
  this->update_health_();
  if (this->waiting_)
    return;

  if (this->health_ == INV_OFFLINE) {
    if (millis() - this->last_probe_ < this->offline_probe_ms_)
      return;
    this->last_probe_ = millis();
    this->probing_ = true;
    this->want_send_ = true;
    this->try_send_();
    return;
  }

  if (this->dump_active_ || this->step_ != IDENT_DONE) {
    this->want_send_ = true;
    this->try_send_();
    return;
  }
  // A previous run left the capability picture in doubt; try again now that
  // the bus has had time to settle.
  if (this->ident_retry_at_ != 0 && millis() >= this->ident_retry_at_) {
    this->begin_identification_();
    return;
  }
  // A hub threshold changed since we last wrote the trip limits.
  if (!this->protection_applied_)
    this->apply_protection_limits_();
  this->start_poll_();
}

void GrowattInverter::start_poll_() {
  // Decide once per cycle whether the slow blocks ride along.
  uint32_t now = millis();
  this->slow_due_ = (this->last_slow_ == 0) ||
                    (now - this->last_slow_ >= this->slow_interval_);
  if (this->slow_due_)
    this->last_slow_ = now;

  this->poll_ = POLL_FAST_MAIN;
  this->retries_ = 0;
  this->want_send_ = true;
  this->try_send_();
}

void GrowattInverter::try_send_() {
  // Step aside briefly after our own transaction so the other devices get a
  // turn before we ask again.
  if (millis() - this->bus_release_ < BUS_YIELD_MS)
    return;
  if (!this->ready_for_immediate_send()) {
    // Log once per pending send, not once per loop iteration.
    if (!this->busy_logged_) {
      ESP_LOGV(TAG, "slot %u: bus busy, will retry", this->slot_index_);
      this->busy_logged_ = true;
    }
    return;
  }
  this->want_send_ = false;
  this->busy_logged_ = false;
  // Writes jump the queue so a user action is not delayed by a poll cycle.
  if (this->write_count_ > 0)
    this->send_write_();
  else if (this->probing_)
    this->send_probe_();
  else if (this->dump_active_)
    this->send_dump_chunk_();
  else if (this->poll_ != POLL_IDLE)
    this->send_poll_();
  else
    this->send_step_();
}

void GrowattInverter::loop() {
  if (this->want_send_ && !this->waiting_)
    this->try_send_();

  if (!this->waiting_)
    return;
  if (millis() - this->last_send_ < IDENT_TIMEOUT_MS)
    return;

  this->waiting_ = false;
  this->retries_++;
  this->bus_release_ = millis();

  // A probe that fails just means it is still gone; waiting through the retry
  // budget would defeat the point of backing off.
  if (this->probing_) {
    this->probing_ = false;
    this->retries_ = 0;
    ESP_LOGV(TAG, "slot %u: still offline", this->slot_index_);
    return;
  }

  if (this->retries_ <= IDENT_MAX_RETRIES) {
    ESP_LOGD(TAG, "slot %u: timeout (step %u, poll %u), retrying (%u/%u)",
             this->slot_index_, this->step_, this->poll_, this->retries_,
             IDENT_MAX_RETRIES);
    this->want_send_ = true;
    return;
  }

  if (this->writing_) {
    ESP_LOGE(TAG, "slot %u: write to %u not acknowledged, dropping",
             this->slot_index_, this->write_queue_[this->write_head_].address);
    this->writing_ = false;
    this->write_head_ = (this->write_head_ + 1) % WRITE_QUEUE_SIZE;
    this->write_count_--;
    this->retries_ = 0;
    this->want_send_ = this->write_count_ > 0;
    return;
  }

  if (this->dump_active_) {
    ESP_LOGW(TAG, "DUMP slot %u: no answer, skipping rest of range %u",
             this->slot_index_, this->dump_range_);
    this->dump_skip_range_();
    return;
  }
  if (this->poll_ != POLL_IDLE) {
    ESP_LOGW(TAG, "slot %u: no answer on poll block %u", this->slot_index_,
             this->poll_);
    this->advance_poll_();
    return;
  }
  ESP_LOGW(TAG, "slot %u: no answer on step %u", this->slot_index_, this->step_);
  this->advance_(false);
}

void GrowattInverter::send_step_() {
  switch (this->step_) {
    case IDENT_START:
      this->step_ = IDENT_LIVE;
      // fallthrough
    case IDENT_LIVE:
      this->send(CMD_READ_INPUT, IN_BASE, FIRST_GROUP_CNT);
      break;
    case IDENT_INFO:
      this->send(CMD_READ_HOLDING, HOLD_BASE, FIRST_GROUP_CNT);
      break;
    case IDENT_TYPE:
      this->send(CMD_READ_HOLDING, REG_TYPE_BASE, REG_TYPE_CNT);
      break;
    case IDENT_CAPS:
      this->send(CMD_READ_HOLDING, REG_PVSTRSCAN, REG_CAPS_CNT);
      break;
    case IDENT_STORAGE:
      this->send(CMD_READ_HOLDING, REG_STORAGE_BASE, REG_STORAGE_CNT);
      break;
    case IDENT_BATTERY:
      this->send(CMD_READ_INPUT, REG_BAT_BASE, REG_BAT_CNT);
      break;
    case IDENT_SETTINGS:
      this->send(CMD_READ_HOLDING, HO_SETTINGS_BASE, HO_SETTINGS_CNT);
      break;
    default:
      return;
  }
  this->last_send_ = millis();
  this->waiting_ = true;
  ESP_LOGV(TAG, "slot %u: sent step %u", this->slot_index_, this->step_);
}

void GrowattInverter::send_poll_() {
  switch (this->poll_) {
    case POLL_FAST_MAIN:
      this->send(CMD_READ_INPUT, POLL_FAST_MAIN_BASE, POLL_FAST_MAIN_CNT);
      break;
    case POLL_FAST_STATUS:
      this->send(CMD_READ_INPUT, POLL_FAST_STATUS_BASE, POLL_FAST_STATUS_CNT);
      break;
    case POLL_FAST_BAT:
      this->send(CMD_READ_INPUT, POLL_FAST_BAT_BASE, POLL_FAST_BAT_CNT);
      break;
    case POLL_FAST_UPS:
      this->send(CMD_READ_INPUT, POLL_FAST_UPS_BASE, POLL_FAST_UPS_CNT);
      break;
    case POLL_SLOW_MAIN:
      this->send(CMD_READ_INPUT, POLL_SLOW_MAIN_BASE, POLL_SLOW_MAIN_CNT);
      break;
    case POLL_SLOW_STOR:
      this->send(CMD_READ_INPUT, POLL_SLOW_STOR_BASE, POLL_SLOW_STOR_CNT);
      break;
    default:
      return;
  }
  this->last_send_ = millis();
  this->waiting_ = true;
  ESP_LOGV(TAG, "slot %u: sent poll block %u", this->slot_index_, this->poll_);
}

// A step that never answered leaves the capability picture unreliable, so the
// whole identification is repeated after this delay instead of running with
// possibly wrong defaults.
void GrowattInverter::advance_(bool ok) {
  this->retries_ = 0;
  this->waiting_ = false;
  if (!ok)
    this->ident_incomplete_ = true;

  switch (this->step_) {
    case IDENT_LIVE:    this->step_ = IDENT_INFO; break;
    case IDENT_INFO:    this->step_ = IDENT_TYPE; break;
    case IDENT_TYPE:    this->step_ = IDENT_CAPS; break;
    case IDENT_CAPS:    this->step_ = IDENT_STORAGE; break;
    case IDENT_STORAGE: this->step_ = IDENT_BATTERY; break;
    case IDENT_BATTERY:
      // Window and rate settings only exist on storage models.
      this->step_ = this->caps_.has_storage ? IDENT_SETTINGS : IDENT_DONE;
      if (this->step_ == IDENT_DONE) {
        this->apply_overrides_();
        this->publish_info_();
      }
      break;
    case IDENT_SETTINGS:
      this->step_ = IDENT_DONE;
      this->apply_overrides_();
      this->publish_info_();
      break;
    default: break;
  }
  if (this->step_ == IDENT_DONE && !this->ident_incomplete_) {
    // A unit reaching this point has either just booted or just come back from
    // an outage, and with holding 2 cleared it has forgotten register 3 either
    // way - so it is running unrestricted while we still believe our last
    // setpoint is in force. Reassert it now rather than waiting out the refresh
    // interval, which is a long time to be producing at 100 %.
    ESP_LOGI(TAG, "slot %u: reasserting %u%% after identification",
             this->slot_index_, this->power_percent_);
    this->apply_power_rate(this->power_percent_);
  }
  if (this->step_ == IDENT_DONE && this->ident_incomplete_) {
    this->ident_runs_++;
    if (this->ident_runs_ < IDENT_MAX_RUNS) {
      this->ident_retry_at_ = millis() + IDENT_RETRY_MS;
      ESP_LOGW(TAG,
               "slot %u: identification incomplete (attempt %u of %u), "
               "retrying in %u s",
               this->slot_index_, this->ident_runs_, IDENT_MAX_RUNS,
               (unsigned) (IDENT_RETRY_MS / 1000));
    } else {
      this->ident_retry_at_ = 0;
      ESP_LOGE(TAG,
               "slot %u: identification still incomplete after %u attempts; "
               "capabilities may be wrong, press Refresh to try again",
               this->slot_index_, IDENT_MAX_RUNS);
    }
  }
  // The voltage convention is only known once the live block has been read.
  if (this->step_ == IDENT_DONE)
    this->apply_protection_limits_();
  this->want_send_ = (this->step_ != IDENT_DONE);
}

void GrowattInverter::advance_poll_() {
  this->retries_ = 0;
  this->waiting_ = false;

  // Storage blocks are skipped entirely on grid tie models, which is the
  // concrete payoff of the capability detection.
  bool stor = this->caps_.has_storage;
  switch (this->poll_) {
    case POLL_FAST_MAIN:
      this->poll_ = POLL_FAST_STATUS;
      break;
    case POLL_FAST_STATUS:
      this->poll_ = stor ? POLL_FAST_BAT
                         : (this->slow_due_ ? POLL_SLOW_MAIN : POLL_IDLE);
      break;
    case POLL_FAST_BAT:
      this->poll_ = POLL_FAST_UPS;
      break;
    case POLL_FAST_UPS:
      this->poll_ = this->slow_due_ ? POLL_SLOW_MAIN : POLL_IDLE;
      break;
    case POLL_SLOW_MAIN:
      this->poll_ = stor ? POLL_SLOW_STOR : POLL_IDLE;
      break;
    default:
      this->poll_ = POLL_IDLE;
      break;
  }
  if (this->poll_ == POLL_IDLE)
    this->publish_derived_();
  this->want_send_ = (this->poll_ != POLL_IDLE);
}

// ------------------------------ identification ------------------------------

void GrowattInverter::detect_from_live_(const std::vector<uint8_t> &data) {
  uint8_t phases = 0;
  for (uint8_t i = 0; i < 3; i++) {
    if (reg16(data, IN_VAC[i]) >= VOLTAGE_PRESENT)
      phases++;
  }
  if (phases > 0) {
    this->caps_.phases = (phases >= 2) ? 3 : 1;
    ESP_LOGI(TAG, "slot %u: %u grid voltages present -> %u phase(s)",
             this->slot_index_, phases, this->caps_.phases);
  } else {
    ESP_LOGW(TAG, "slot %u: no grid voltage, cannot detect phases",
             this->slot_index_);
  }

  // Strings read zero at night or when the array is disconnected, so keep the
  // highest count ever seen and let the user press Refresh during the day.
  uint8_t strings = 0;
  for (uint8_t i = 0; i < MAX_STRINGS; i++) {
    if (reg16(data, IN_VPV_FIRST + i * IN_VPV_STEP) >= VOLTAGE_PRESENT)
      strings = i + 1;  // highest populated index, keeps gaps intact
  }
  if (strings > this->caps_.strings) {
    this->caps_.strings = strings;
    ESP_LOGI(TAG, "slot %u: %u PV string(s) detected", this->slot_index_, strings);
  } else if (strings == 0) {
    ESP_LOGD(TAG, "slot %u: no PV voltage right now, keeping strings=%u",
             this->slot_index_, this->caps_.strings);
  }
}

void GrowattInverter::parse_device_info_(const std::vector<uint8_t> &data) {
  this->caps_.dtc = reg16(data, HO_DTC);
  this->caps_.serial = ascii_from(data, HO_SERIAL, HO_SERIAL_CNT);

  // Nameplate power is documented as 0.1 VA and reads that way on MOD and MIN
  // units, but the Storage family reports whole VA instead (an SPH 10000
  // returns 10000, not 100000). Rather than key this off a model list, the
  // value is checked for plausibility: no real inverter is rated under 500 VA.
  float p = reg32(data, HO_NORMAL_POWER) * ONE_DEC;
  if (p > 0 && p < 500.0f)
    p *= 10.0f;
  this->normal_power_va_ = p;
  // The controller scales its steps by this figure, so an implausible reading
  // must disable proportional control rather than produce wild jumps.
  this->normal_power_valid_ = (p >= 500.0f && p <= 100000.0f);
  if (!this->normal_power_valid_ && p > 0) {
    ESP_LOGW(TAG, "slot %u: implausible nameplate power %.0f VA, ignoring",
             this->slot_index_, p);
  }

  pub_val(this->normal_power_, this->normal_power_va_);
  pub1(this->modbus_version_, data, HO_MODBUS_VER, TWO_DEC);
  pub1(this->active_rate_, data, HO_ACTIVE_RATE, 1.0f);
  pub1(this->reactive_rate_, data, HO_REACTIVE_RATE, 1.0f);
  pub1(this->power_factor_set_, data, HO_PF_SET, 1.0f);
  pub1(this->pv_nominal_voltage_, data, HO_VNORMAL, ONE_DEC);
  pub1(this->com_address_, data, HO_COM_ADDRESS, 1.0f);
  pub1(this->pf_model_, data, HO_PF_MODEL, 1.0f);
  pub1(this->tracker_model_, data, HO_TRACKER_MODEL, 1.0f);

  // Editable settings that live in the first holding group get their initial
  // value here, so the UI starts out matching the inverter.
  for (uint8_t f = 0; f < SET_COUNT; f++) {
    if (SETTING_ADDR[f] < FIRST_GROUP_CNT)
      this->settings_[f] = reg16(data, SETTING_ADDR[f]);
  }
  // Register backed selects and switches that live in the first holding group
  this->publish_reg_entities_(data, HOLD_BASE, FIRST_GROUP_CNT);

  // Writing the power rate every few seconds would wear the EEPROM out if the
  // inverter is set to remember it. Clearing holding 2 makes those writes
  // volatile instead.
  if (this->protect_eeprom_ && reg16(data, HO_PF_CMD_MEMORY) != 0) {
    ESP_LOGW(TAG,
             "slot %u: setting memory is on, clearing it to protect the EEPROM",
             this->slot_index_);
    this->write_register(HO_PF_CMD_MEMORY, 0);
  }

  this->publish_settings_();

  pub_text(this->firmware_ts_, ascii_from(data, HO_FIRMWARE, HO_FIRMWARE_CNT));
  pub_text(this->fw_build_ts_, ascii_from(data, HO_FW_BUILD, HO_FW_BUILD_CNT));
  pub_text(this->serial_ts_, this->caps_.serial);
  pub_text(this->manufacturer_ts_,
           ascii_from(data, HO_MANUFACTURER, HO_MANUFACTURER_CNT));

  if (this->system_time_ts_ != nullptr) {
    // Register 45 holds the full year (2026), not an offset from 2000.
    char t[40];
    snprintf(t, sizeof(t), "%04u-%02u-%02u %02u:%02u:%02u",
             reg16(data, HO_SYS_TIME), reg16(data, HO_SYS_TIME + 1),
             reg16(data, HO_SYS_TIME + 2), reg16(data, HO_SYS_TIME + 3),
             reg16(data, HO_SYS_TIME + 4), reg16(data, HO_SYS_TIME + 5));
    this->system_time_ts_->publish_state(std::string(t));
  }

  // TP (register 44) packs the installed MPPT tracker count in the high byte
  // and the phase count in the low byte. It is accurate on MOD and MIN units
  // but returns garbage on the Storage family, so it is filtered for
  // plausibility and only used to fill gaps: the tracker count, which no live
  // measurement can reveal, and the phase count when the inverter is off grid
  // and there are no voltages to count.
  uint16_t tp = reg16(data, HO_TP);
  uint8_t tp_trackers = (tp >> 8) & 0xFF;
  uint8_t tp_phases = tp & 0xFF;
  bool tp_ok = (tp_phases == 1 || tp_phases == 3) && tp_trackers >= 1 &&
               tp_trackers <= MAX_STRINGS;
  if (tp_ok) {
    this->caps_.trackers = tp_trackers;
    if (this->caps_.phases == 0) {
      this->caps_.phases = tp_phases;
      ESP_LOGI(TAG, "slot %u: no grid voltage, taking %u phase(s) from TP",
               this->slot_index_, tp_phases);
    }
  }

  ESP_LOGI(TAG, "slot %u: DTC=%u, serial='%s', TP=0x%04X (%s)",
           this->slot_index_, this->caps_.dtc, this->caps_.serial.c_str(), tp,
           tp_ok ? "plausible" : "not usable on this model");
}
// ------------------------------ poll parsing ------------------------------

// Fast block, input 0..56. Everything the control logic needs.
void GrowattInverter::parse_fast_main_(const std::vector<uint8_t> &data) {
  uint16_t status = reg16(data, IN_STATUS);
  pub_val(this->status_code_, status);
  pub_text(this->status_ts_, status_text(status));

  pub2(this->pv_active_power_, data, IN_PV_POWER, ONE_DEC);
  this->pv_power_w_ = reg32(data, IN_PV_POWER) * ONE_DEC;

  for (uint8_t i = 0; i < MAX_STRINGS; i++) {
    size_t v = IN_VPV_FIRST + i * IN_VPV_STEP;
    pub1(this->pvs_[i].voltage, data, v, ONE_DEC);
    pub1(this->pvs_[i].current, data, v + 1, ONE_DEC);
    pub2(this->pvs_[i].power, data, v + 2, ONE_DEC);
  }

  pub2(this->grid_active_power_, data, IN_AC_POWER, ONE_DEC);
  this->grid_power_w_ = reg32(data, IN_AC_POWER) * ONE_DEC;
  this->revise_nameplate_();
  this->update_capability_();
  pub1(this->frequency_, data, IN_FREQUENCY, TWO_DEC);

  float iac[3], pac[3];
  uint8_t i_present = 0, p_present = 0;
  for (uint8_t i = 0; i < 3; i++) {
    size_t v = IN_VAC[i];
    this->ac_voltage_[i] = reg16(data, v) * ONE_DEC;
    iac[i] = reg16(data, v + 1) * ONE_DEC;
    pac[i] = reg32(data, v + 2) * ONE_DEC;
    if (iac[i] >= PHASE_CURRENT_PRESENT_A)
      i_present++;
    if (pac[i] >= PHASE_POWER_PRESENT_W)
      p_present++;
    this->ac_line_voltage_[i] = reg16(data, IN_LINE_VOLT + i) * ONE_DEC;
  }

  // Some three phase units report the whole AC output in Pac1 and leave Pac2
  // and Pac3 at zero, while Iac2 and Iac3 carry real current - so the phases
  // are genuinely working and the power registers simply are not per phase.
  // Detected rather than keyed off a model, because a MOD 40K populates all
  // three: current on at least two phases, power on exactly the first, and
  // that first figure matching the total output.
  //
  // The output floor exists because a phase carrying almost no active power
  // reads the same as one that is not reported at all. At first light an
  // inverter can push reactive current on every phase while the active figures
  // are still rounding to zero, which is exactly the shape being looked for.
  // Requiring the picture to hold for several consecutive cycles is the real
  // defence; the floor only keeps the question from being asked when the
  // answer cannot mean anything.
  //
  // Latched until the next identification: once known, it stays known through
  // the night, when nothing can be measured.
  if (!this->pac_is_total_ && this->caps_.phases >= 3 &&
      this->grid_power_w_ >= this->phase_detect_min_w_) {
    bool looks_total =
        i_present >= 2 && p_present == 1 &&
        pac[0] >= PHASE_POWER_PRESENT_W && this->grid_power_w_ > 0 &&
        fabsf(pac[0] - this->grid_power_w_) <=
            PAC_TOTAL_TOLERANCE * this->grid_power_w_;
    if (!looks_total) {
      this->pac_total_hits_ = 0;
    } else if (++this->pac_total_hits_ >= PAC_TOTAL_CONFIRMATIONS) {
      this->pac_is_total_ = true;
      ESP_LOGW(TAG,
               "slot %u: whole %.0f W output reported in Pac1 with Pac2/Pac3 "
               "at zero (Iac %.1f/%.1f/%.1f A) on %u consecutive cycles; per "
               "phase power is not available on this model, use "
               "grid_active_power",
               this->slot_index_, this->grid_power_w_, iac[0], iac[1], iac[2],
               PAC_TOTAL_CONFIRMATIONS);
    } else {
      ESP_LOGD(TAG, "slot %u: Pac1 carries the whole %.0f W (%u/%u)",
               this->slot_index_, this->grid_power_w_, this->pac_total_hits_,
               PAC_TOTAL_CONFIRMATIONS);
    }
  }

  for (uint8_t i = 0; i < 3; i++) {
    pub_val(this->phases_[i].voltage, this->ac_voltage_[i]);
    pub_val(this->phases_[i].current, iac[i]);
    // NaN on all three, not just the two reading zero: Pac1 is the total, not
    // the first phase's share of it, and the total already has its own sensor.
    pub_val(this->phases_[i].power, this->pac_is_total_ ? NAN : pac[i]);
    pub_val(this->line_voltages_[i], this->ac_line_voltage_[i]);
  }

  pub2(this->energy_today_, data, IN_E_TODAY, ONE_DEC);
  pub2(this->energy_total_, data, IN_E_TOTAL, ONE_DEC);
}

// Fast status block, input 101..105. Small on purpose: it runs on every cycle
// so the controller knows whether the inverter can follow a higher setpoint.
void GrowattInverter::parse_fast_status_(const std::vector<uint8_t> &data) {
  // The document specifies 1 % per count here. Earlier YAML based setups often
  // used 0.1; verify against the inverter display if the value looks off.
  this->real_percent_val_ = reg16(data, 0);  // 101
  pub_val(this->real_power_percent_, this->real_percent_val_);
  pub2(this->output_max_power_, data, 1, ONE_DEC);  // 102..103

  this->derating_val_ = reg16(data, 3) & 0xFF;  // 104
  if (this->derating_val_ != this->derating_prev_) {
    ESP_LOGI(TAG, "slot %u: derating %u (%s)", this->slot_index_,
             this->derating_val_, derating_text(this->derating_val_));
    this->derating_prev_ = this->derating_val_;
  }
  pub_val(this->derating_mode_, this->derating_val_);
  pub_text(this->derating_ts_, derating_text(this->derating_val_));

  uint16_t fault = reg16(data, 4);  // 105
  pub_val(this->fault_code_, fault);
  pub_text(this->fault_ts_, fault_text(fault));
}

// Slow block, input 57..124. Counters, temperatures and diagnostics.
void GrowattInverter::parse_slow_main_(const std::vector<uint8_t> &data) {
  const uint8_t B = POLL_SLOW_MAIN_BASE;  // rebase absolute addresses

  // work time is counted in half seconds, published as hours
  pub2(this->work_time_total_, data, IN_WORK_TIME - B, 0.5f / 3600.0f);
  for (uint8_t i = 0; i < MAX_STRINGS; i++) {
    pub2(this->pv_energy_today_[i], data,
         IN_EPV_TODAY_FIRST - B + i * IN_EPV_STEP, ONE_DEC);
    pub2(this->pv_energy_total_[i], data,
         IN_EPV_TOTAL_FIRST - B + i * IN_EPV_STEP, ONE_DEC);
  }
  pub2(this->pv_energy_total_all_, data, IN_EPV_TOTAL_ALL - B, ONE_DEC);

  pub1(this->temperature_, data, IN_TEMP - B, ONE_DEC);
  pub1(this->ipm_temperature_, data, IN_TEMP_IPM - B, ONE_DEC);
  pub1(this->boost_temperature_, data, IN_TEMP_BOOST - B, ONE_DEC);
  pub1(this->battery_voltage_dsp_, data, IN_BAT_VOLT_DSP - B, ONE_DEC);
  pub1(this->bus_voltage_p_, data, IN_BUS_P - B, ONE_DEC);
  pub1(this->bus_voltage_n_, data, IN_BUS_N - B, ONE_DEC);
  // raw 0..20000 where 10000 means unity, published unscaled
  pub1(this->output_power_factor_, data, IN_OUTPUT_PF - B, 1.0f);

  // 101..105 belong to the fast status block, not repeated here.
  pub1(this->fault_subcode_, data, IN_FAULT_SUB - B, 1.0f);
  pub1(this->warning_bits_, data, IN_WARN_BITS - B, 1.0f);
  pub1(this->warning_subcode_, data, IN_WARN_SUB - B, 1.0f);

  // Registers 112..115 carry AC charge energy on storage models and warning
  // codes on MAX class inverters.
  if (this->caps_.has_storage) {
    pub2(this->ac_charge_e_today_, data, IN_EACHARGE_TODAY - B, ONE_DEC);
    pub2(this->ac_charge_e_total_, data, IN_EACHARGE_TOTAL - B, ONE_DEC);
    pub2(this->ac_charge_power_, data, IN_AC_CHARGE_POWER - B, ONE_DEC);
    pub1(this->priority_, data, IN_PRIORITY - B, 1.0f);
    pub1(this->battery_type_, data, IN_BATTERY_TYPE - B, 1.0f);
  } else {
    pub1(this->warning_code_, data, IN_WARN_MAIN - B, 1.0f);
  }
}

// Fast storage block, input 1009..1014.
void GrowattInverter::parse_fast_bat_(const std::vector<uint8_t> &data) {
  pub2(this->bat_discharge_power_, data, 0, ONE_DEC);  // 1009
  pub2(this->bat_charge_power_, data, 2, ONE_DEC);     // 1011
  this->battery_voltage_v_ = reg16(data, 4) * ONE_DEC;  // 1013
  pub_val(this->battery_voltage_, this->battery_voltage_v_);
  this->battery_soc_pct_ = reg16(data, 5);              // 1014
  pub_val(this->battery_soc_sens_, this->battery_soc_pct_);
}

// Fast UPS block, input 1067..1081. Also feeds the load average window.
void GrowattInverter::parse_fast_ups_(const std::vector<uint8_t> &data) {
  // The frequency register reads 0 while the UPS output is idle, which is not
  // a frequency of zero. Publishing NaN marks the sensor unavailable instead.
  uint16_t freq = reg16(data, 0);  // 1067
  if (this->ups_frequency_ != nullptr)
    this->ups_frequency_->publish_state(freq == 0 ? NAN : freq * TWO_DEC);
  for (uint8_t i = 0; i < 3; i++) {
    size_t v = 1 + i * ST_UPS_STEP;  // 1068, 1072, 1076
    pub1(this->ups_[i].voltage, data, v, ONE_DEC);
    pub1(this->ups_[i].current, data, v + 1, ONE_DEC);
    this->ups_phase_power_[i] = reg32(data, v + 2) * ONE_DEC;
    pub_val(this->ups_[i].power, this->ups_phase_power_[i]);
  }
  uint16_t load_raw = reg16(data, 13);  // 1080
  this->ups_load_pct_ = load_raw * ONE_DEC;
  pub_val(this->ups_load_, this->ups_load_pct_);
  pub1(this->ups_power_factor_, data, 14, ONE_DEC);  // 1081

  this->ups_avg_buf_[this->ups_avg_pos_] = load_raw;
  this->ups_avg_pos_ = (this->ups_avg_pos_ + 1) % this->ups_avg_window_;
  if (this->ups_avg_count_ < this->ups_avg_window_)
    this->ups_avg_count_++;

  uint32_t acc = 0;
  for (uint8_t i = 0; i < this->ups_avg_count_; i++)
    acc += this->ups_avg_buf_[i];
  this->ups_load_avg_pct_ = acc * ONE_DEC / this->ups_avg_count_;
}

void GrowattInverter::parse_storage_(const std::vector<uint8_t> &data) {
  // Slow storage block, input 1000..1096. Values already covered by the fast
  // blocks are simply refreshed here.
  pub1(this->system_work_mode_, data, ST_WORK_MODE, 1.0f);
  pub1(this->fault_word_, data, ST_FAULT_WORD, 1.0f);
  pub1(this->battery_temperature_, data, ST_BAT_TEMP, ONE_DEC);

  pub2(this->power_to_user_, data, ST_P_TO_USER, ONE_DEC);
  pub2(this->power_to_grid_, data, ST_P_TO_GRID, ONE_DEC);
  pub2(this->local_load_power_, data, ST_P_LOCAL_LOAD, ONE_DEC);

  pub2(this->e_to_user_today_, data, ST_E_TO_USER_TODAY, ONE_DEC);
  pub2(this->e_to_user_total_, data, ST_E_TO_USER_TOTAL, ONE_DEC);
  pub2(this->e_to_grid_today_, data, ST_E_TO_GRID_TODAY, ONE_DEC);
  pub2(this->e_to_grid_total_, data, ST_E_TO_GRID_TOTAL, ONE_DEC);
  pub2(this->discharge_energy_today_, data, ST_E_DISCHARGE_TODAY, ONE_DEC);
  pub2(this->discharge_energy_total_, data, ST_E_DISCHARGE_TOTAL, ONE_DEC);
  pub2(this->charge_energy_today_, data, ST_E_CHARGE_TODAY, ONE_DEC);
  pub2(this->charge_energy_total_, data, ST_E_CHARGE_TOTAL, ONE_DEC);
  pub2(this->e_load_today_, data, ST_E_LOAD_TODAY, ONE_DEC);
  pub2(this->e_load_total_, data, ST_E_LOAD_TOTAL, ONE_DEC);

  pub1(this->bms_soc_, data, ST_BMS_SOC, 1.0f);
  pub1(this->bms_voltage_, data, ST_BMS_VOLT, TWO_DEC);
  pub1(this->bms_current_, data, ST_BMS_CURR, TWO_DEC);
  pub1(this->bms_temperature_, data, ST_BMS_TEMP, ONE_DEC);
  pub1(this->battery_capacity_, data, ST_BAT_CAPACITY, ONE_DEC);
  pub1(this->battery_cycles_, data, ST_BAT_CYCLES, 1.0f);
  pub1(this->battery_health_, data, ST_BAT_HEALTH, 1.0f);
}

// Values that are computed rather than read. Kept in the component so the
// YAML side only has to declare the sensor it wants to see.
void GrowattInverter::publish_derived_() {
  if (!this->caps_.has_storage)
    return;

  if (this->ups_total_power_ != nullptr) {
    float sum = this->ups_phase_power_[0] + this->ups_phase_power_[1] +
                this->ups_phase_power_[2];
    this->ups_total_power_->publish_state(sum);
  }

  if (this->ups_load_avg_ != nullptr && this->ups_avg_count_ > 0)
    this->ups_load_avg_->publish_state(this->ups_load_avg_pct_);

  // Module count from pack voltage. module_voltage_ is configurable because it
  // depends on the battery model, not on the inverter.
  float modules = 0;
  if (this->module_voltage_ > 0 && this->battery_voltage_v_ > 0)
    modules = roundf(this->battery_voltage_v_ / this->module_voltage_);
  pub_val(this->battery_modules_, modules);

  // Maximum sustainable discharge expressed as a percentage of the inverter
  // rating: usable pack energy divided by the discharge window.
  if (this->ups_max_power_ != nullptr && this->normal_power_va_ > 0 &&
      this->discharge_hours_ > 0) {
    float pack_wh = modules * this->module_capacity_ * 1000.0f;
    float max_w = pack_wh / this->discharge_hours_;
    float pct = roundf(max_w / this->normal_power_va_ * 100.0f);
    if (pct > 100.0f)
      pct = 100.0f;
    this->ups_max_power_->publish_state(pct);
  }
}

// ------------------------------ write path ------------------------------

bool GrowattInverter::is_rejected_(uint16_t address) const {
  for (uint8_t i = 0; i < this->rejected_count_; i++)
    if (this->rejected_[i] == address)
      return true;
  return false;
}

void GrowattInverter::mark_rejected_(uint16_t address) {
  if (this->is_rejected_(address) || this->rejected_count_ >= MAX_REJECTED)
    return;
  this->rejected_[this->rejected_count_++] = address;
}

bool GrowattInverter::queue_write_(uint8_t function, uint16_t address,
                                   const uint16_t *values, uint8_t count) {
  // Silently skipped rather than logged every time: the unit has already said
  // it will not take this register, and repeating that at every protection pass
  // would bury the log in a fact we already know.
  if (this->is_rejected_(address)) {
    ESP_LOGV(TAG, "slot %u: skipping write to %u, previously rejected",
             this->slot_index_, address);
    return false;
  }
  if (this->write_count_ >= WRITE_QUEUE_SIZE) {
    ESP_LOGW(TAG, "slot %u: write queue full, dropping write to %u",
             this->slot_index_, address);
    return false;
  }
  uint8_t idx = (this->write_head_ + this->write_count_) % WRITE_QUEUE_SIZE;
  PendingWrite &w = this->write_queue_[idx];
  w.function = function;
  w.address = address;
  w.count = count;
  for (uint8_t i = 0; i < count && i < WINDOW_REGS; i++)
    w.values[i] = values[i];
  this->write_count_++;
  this->want_send_ = true;
  return true;
}

void GrowattInverter::send_write_() {
  const PendingWrite &w = this->write_queue_[this->write_head_];
  uint8_t payload[WINDOW_REGS * 2];
  for (uint8_t i = 0; i < w.count; i++) {
    payload[i * 2] = w.values[i] >> 8;
    payload[i * 2 + 1] = w.values[i] & 0xFF;
  }
  this->send(w.function, w.address, w.count, w.count * 2, payload);
  this->last_send_ = millis();
  this->waiting_ = true;
  this->writing_ = true;
  ESP_LOGI(TAG, "slot %u: write fn 0x%02X addr %u, %u register(s)",
           this->slot_index_, w.function, w.address, w.count);
}

// Kinds accepted by GrowattRateNumber, must match __init__.py.
static const uint8_t RATE_MIN = 0;
static const uint8_t RATE_MAX = 1;
static const uint8_t RATE_UPDATE = 2;
static const uint8_t RATE_SLOW = 3;

void GrowattRateNumber::control(float value) {
  this->publish_state(value);
  if (this->parent_ == nullptr)
    return;
  switch (this->kind_) {
    case RATE_MIN:    this->parent_->apply_min_power_rate(value); break;
    case RATE_MAX:    this->parent_->apply_max_power_rate(value); break;
    case RATE_UPDATE: this->parent_->apply_update_interval(value); break;
    default:          this->parent_->apply_slow_interval(value); break;
  }
}

void GrowattInverter::apply_min_power_rate(float v) {
  uint8_t r = (uint8_t) lroundf(v);
  if (r > this->max_power_rate_)
    r = this->max_power_rate_;
  this->min_power_rate_ = r;
  this->save_prefs_();
  ESP_LOGI(TAG, "slot %u: min power rate %u%%", this->slot_index_, r);
  // A bound that no longer contains the current setpoint is not a bound, so it
  // is enforced at once rather than at the controller's convenience.
  if (this->power_percent_ < r)
    this->apply_power_rate(r);
  if (this->min_rate_num_ != nullptr)
    this->min_rate_num_->publish_state(r);
}

void GrowattInverter::apply_max_power_rate(float v) {
  uint8_t r = (uint8_t) lroundf(v);
  if (r < this->min_power_rate_)
    r = this->min_power_rate_;
  this->max_power_rate_ = r;
  this->save_prefs_();
  ESP_LOGI(TAG, "slot %u: max power rate %u%%", this->slot_index_, r);
  if (this->power_percent_ > r)
    this->apply_power_rate(r);
  if (this->max_rate_num_ != nullptr)
    this->max_rate_num_->publish_state(r);
}

void GrowattInverter::apply_update_interval(float seconds) {
  uint32_t ms = (uint32_t) (seconds * 1000.0f);
  if (ms < 1000)
    return;
  // A PollingComponent does not notice a new interval by itself.
  this->stop_poller();
  this->set_update_interval(ms);
  this->start_poller();
  this->save_prefs_();
  ESP_LOGI(TAG, "slot %u: poll interval %u ms", this->slot_index_, (unsigned) ms);
}

void GrowattInverter::apply_slow_interval(float seconds) {
  uint32_t ms = (uint32_t) (seconds * 1000.0f);
  if (ms < 1000)
    return;
  this->slow_interval_ = ms;
  this->save_prefs_();
  ESP_LOGI(TAG, "slot %u: slow block interval %u ms", this->slot_index_,
           (unsigned) ms);
}

void GrowattInverter::apply_convention(uint8_t c) {
  if (c >= CONV_MODE_COUNT)
    return;
  this->cfg_convention_ = c;
  this->save_prefs_();
  ESP_LOGI(TAG, "slot %u: voltage convention %u", this->slot_index_, c);
  // The trip thresholds were written in the old convention, so they have to go
  // out again in the new one.
  this->protection_applied_ = false;
}

void GrowattInverter::apply_auto_protection(bool on) {
  this->auto_protection_ = on;
  this->save_prefs_();
  ESP_LOGI(TAG, "slot %u: automatic protection limits %s", this->slot_index_,
           on ? "on" : "off");
  if (on)
    this->protection_applied_ = false;  // apply them on the next cycle
  if (this->auto_prot_sw_ != nullptr)
    this->auto_prot_sw_->publish_state(on);
}

void GrowattInverter::apply_protect_eeprom(bool on) {
  this->protect_eeprom_ = on;
  this->save_prefs_();
  ESP_LOGI(TAG, "slot %u: EEPROM setting memory %s", this->slot_index_,
           on ? "cleared on identification" : "left alone");
  if (this->eeprom_sw_ != nullptr)
    this->eeprom_sw_->publish_state(on);
}

void GrowattInverterOptionSwitch::write_state(bool state) {
  this->publish_state(state);
  if (this->parent_ == nullptr)
    return;
  if (this->is_eeprom_)
    this->parent_->apply_protect_eeprom(state);
  else
    this->parent_->apply_auto_protection(state);
}

void GrowattInverter::apply_safe_power_rate(float v) {
  if (v < 0)
    v = 0;
  if (v > 100)
    v = 100;
  uint8_t r = (uint8_t) lroundf(v);
  this->safe_power_rate_ = r;
  this->save_prefs_();
  ESP_LOGI(TAG, "slot %u: safe power rate set to %u%%", this->slot_index_, r);
  // Raising it above the current output is a request for production now, not a
  // note for later. Lowering it is not: that would cut output while the meter
  // is perfectly healthy and the controller is in charge.
  if (r > this->power_percent_) {
    ESP_LOGI(TAG, "slot %u: currently at %u%%, raising to the new safe rate",
             this->slot_index_, this->power_percent_);
    this->apply_power_rate(r);
  }
}

void GrowattInverter::apply_power_rate(float pct) {
  if (pct < this->min_power_rate_)
    pct = this->min_power_rate_;
  if (pct > this->max_power_rate_)
    pct = this->max_power_rate_;
  uint16_t v = (uint16_t) lroundf(pct);
  this->settings_[SET_ACTIVE_POWER_RATE] = v;
  this->power_percent_ = (uint8_t) v;
  this->queue_write_(CMD_WRITE_SINGLE, REG_ACTIVE_POWER_RATE, &v, 1);
  if (this->setting_num_[SET_ACTIVE_POWER_RATE] != nullptr)
    this->setting_num_[SET_ACTIVE_POWER_RATE]->publish_state(v);
}

const char *GrowattInverter::get_derating_text() const {
  return derating_text(this->derating_val_);
}

// Called from the fast poll, where the phase count and the real output are
// known - neither is available when the nameplate register is first parsed.
// The inverter never says "you are what is limiting me": derating mode 7 is
// reported whenever a limit is set, whether or not it binds. But the arithmetic
// tells us. If output has reached the limit our setpoint implies, the setpoint
// is the constraint and output scales with it; if output sits far below, the
// panels are the constraint and raising the setpoint does nothing.
//
// Measured on real hardware: an SPH clipping at 9-18 % extrapolated to 9600,
// 9670, 9671 and 9689 W on four different setpoints, and two MIN units to
// within 10 W of their 6000 W nameplate. A MOD that was PV limited throughout
// gave 451, 647, 672, 1292 and 1325 W - which is why the ratio test has to gate
// this, or the rolling maximum would keep the worst overestimate.
void GrowattInverter::update_capability_() {
  if (!this->normal_power_valid_ || std::isnan(this->grid_power_w_) ||
      this->power_percent_ == 0)
    return;

  float limit = this->normal_power_va_ * this->power_percent_ / 100.0f;
  this->rate_binding_ = limit > 0 && this->grid_power_w_ >= this->cap_ratio_ * limit;

  uint32_t now = millis();
  if (!std::isnan(this->capability_w_) &&
      now - this->cap_time_ > this->cap_window_ms_) {
    ESP_LOGD(TAG, "slot %u: capability estimate expired", this->slot_index_);
    this->capability_w_ = NAN;
    if (this->capability_sens_ != nullptr)
      this->capability_sens_->publish_state(NAN);
  }
  if (!this->rate_binding_)
    return;

  float est = this->grid_power_w_ * 100.0f / this->power_percent_;
  if (est > this->normal_power_va_)
    est = this->normal_power_va_;
  if (std::isnan(this->capability_w_) || est >= this->capability_w_) {
    this->capability_w_ = est;
    this->cap_time_ = now;
    if (this->capability_sens_ != nullptr)
      this->capability_sens_->publish_state(est);
  }
}

// Deliberately zero when the setpoint is not binding: an inverter producing
// 388 W of a 24000 W allowance will not produce more because we allow more.
float GrowattInverter::available_headroom() const {
  if (std::isnan(this->capability_w_))
    return 0.0f;
  int16_t room = (int16_t) this->max_power_rate_ - (int16_t) this->power_percent_;
  if (room <= 0)
    return 0.0f;
  return this->capability_w_ * room / 100.0f;
}

void GrowattInverter::revise_nameplate_() {
  if (this->nameplate_revised_ || this->normal_power_va_ <= 0)
    return;

  const char *why = nullptr;
  // No three phase inverter is built under 3 kVA, so a three phase unit
  // claiming less than that is reporting in the wrong unit.
  if (this->caps_.phases >= 3 && this->normal_power_va_ < 3000.0f)
    why = "three phase unit rated below 3 kVA";
  // Whatever the register says, a unit cannot exceed its own nameplate. This
  // catches any model the phase test misses, at the cost of only firing once
  // the inverter has actually produced that much.
  else if (!std::isnan(this->grid_power_w_) &&
           this->grid_power_w_ > this->normal_power_va_)
    why = "output above the stated nameplate";
  if (why == nullptr)
    return;

  float from = this->normal_power_va_;
  float to = from * 10.0f;
  // Only believe the correction if it lands somewhere sane.
  if (to > 100000.0f)
    return;
  this->normal_power_va_ = to;
  this->normal_power_valid_ = true;
  this->nameplate_revised_ = true;
  ESP_LOGW(TAG,
           "slot %u: %s (%.0f VA) - reporting whole VA rather than 0.1 VA, "
           "nameplate corrected to %.0f VA",
           this->slot_index_, why, from, to);
  pub_val(this->normal_power_, this->normal_power_va_);
}

bool GrowattInverter::can_produce_more() const {
  // Derating mode 7 means "limited by the command you gave me", which is the
  // normal state whenever our setpoint is below 100 % and must not count as a
  // reason to stop raising it. Other modes are genuine self-imposed limits.
  if (this->derating_val_ != 0 && this->derating_val_ != 7)
    return false;

  // Is there anything to draw on at all? For a grid tie unit that means light
  // on the panels; without this the controller would walk every inverter up to
  // 100 % through the night and they would all be wide open at first light.
  // A unit with storage is different: it can keep supplying from the battery
  // long after dark, so the daylight test must not apply to it.
  bool has_source =
      std::isnan(this->pv_power_w_) || this->pv_power_w_ >= MIN_PV_POWER_W;
  if (!has_source && this->caps_.has_storage) {
    has_source = !std::isnan(this->battery_soc_pct_) &&
                 this->battery_soc_pct_ > this->discharge_floor_();
  }
  if (!has_source)
    return false;

  // Nothing more to give once the setpoint is at maximum, or the inverter is
  // already producing its nameplate power. Anything short of that is worth
  // trying: an inverter held back by cloud cover simply climbs to 100 % over a
  // few cycles and then stops being a candidate on its own, which is both
  // correct and free of timing guesswork.
  if (this->power_percent_ >= 100)
    return false;
  if (this->normal_power_valid_ && !std::isnan(this->grid_power_w_) &&
      this->grid_power_w_ >= this->normal_power_va_)
    return false;
  return true;
}

// State of charge below which the inverter stops discharging on its own, so
// asking it for more would achieve nothing. Falls back to a low floor when the
// setting has not been read.
float GrowattInverter::discharge_floor_() const {
  float f = this->settings_[SET_GF_STOP_SOC];
  return f > 0 ? f : 5.0f;
}

// NaN when the inverter has published nothing yet, which callers must treat as
// "no reading" rather than as a low voltage.
static float peak_of_(const float *v) {
  float peak = NAN;
  for (uint8_t i = 0; i < 3; i++) {
    if (std::isnan(v[i]))
      continue;
    if (std::isnan(peak) || v[i] > peak)
      peak = v[i];
  }
  return peak;
}

float GrowattInverter::peak_ac_voltage() const {
  return peak_of_(this->ac_voltage_);
}

float GrowattInverter::peak_line_voltage() const {
  return peak_of_(this->ac_line_voltage_);
}

// The configured convention is honoured, but not blindly: a unit told to work
// in line terms because that is what its protection registers want may still
// report phase voltages here, and comparing 240 V against a 440 V line limit
// would silently disable the whole over voltage check. Magnitude settles it -
// the two conventions are 170 V apart and cannot be confused.
bool GrowattInverter::ac_voltage_is_line() const {
  float peak = this->peak_ac_voltage();
  bool plausible_line = !std::isnan(peak) && peak >= LINE_VOLTAGE_FLOOR;
  if (this->cfg_convention_ == CONV_PHASE)
    return false;
  if (this->cfg_convention_ == CONV_LINE)
    return plausible_line || std::isnan(peak) || peak <= 0;
  return plausible_line;
}

bool GrowattInverter::reports_line_voltage() const {
  if (this->cfg_convention_ == CONV_PHASE)
    return false;
  if (this->cfg_convention_ == CONV_LINE)
    return true;
  // A unit that populates the line to line registers works in line terms even
  // when it also reports phase voltages around 230 V, and it expects its trip
  // thresholds written the same way.
  for (uint8_t i = 0; i < 3; i++) {
    if (!std::isnan(this->ac_line_voltage_[i]) &&
        this->ac_line_voltage_[i] > 100.0f)
      return true;
  }
  // Fallback for models that only publish one set: a phase register reading
  // above 300 V can only be a line voltage.
  for (uint8_t i = 0; i < 3; i++) {
    if (!std::isnan(this->ac_voltage_[i]) && this->ac_voltage_[i] > 300.0f)
      return true;
  }
  return false;
}

void GrowattInverter::set_protection_targets(float phase_low, float phase_high,
                                             float line_low, float line_high,
                                             uint16_t restart_delay_s) {
  bool changed = phase_low != this->tgt_phase_low_ ||
                 phase_high != this->tgt_phase_high_ ||
                 line_low != this->tgt_line_low_ ||
                 line_high != this->tgt_line_high_ ||
                 restart_delay_s != this->tgt_restart_delay_;
  this->tgt_phase_low_ = phase_low;
  this->tgt_phase_high_ = phase_high;
  this->tgt_line_low_ = line_low;
  this->tgt_line_high_ = line_high;
  this->tgt_restart_delay_ = restart_delay_s;
  if (changed)
    this->protection_applied_ = false;  // re-apply with the new targets
}

// Widening the inverter's own trip thresholds beyond the range the controller
// reacts to leaves room to intervene before the hardware disconnects, which
// would otherwise cost minutes of production while it waits to reconnect.
void GrowattInverter::apply_protection_limits_() {
  if (!this->auto_protection_ || this->protection_applied_)
    return;
  if (this->tgt_phase_low_ <= 0 || this->tgt_phase_high_ <= 0)
    return;

  bool line = this->reports_line_voltage();
  float low = line ? this->tgt_line_low_ : this->tgt_phase_low_;
  float high = line ? this->tgt_line_high_ : this->tgt_phase_high_;
  if (low <= 0 || high <= 0)
    return;

  ESP_LOGI(TAG,
           "slot %u: using %s limits (%s), Vac %.0f/%.0f/%.0f, "
           "line %.0f/%.0f/%.0f",
           this->slot_index_, line ? "line" : "phase",
           this->cfg_convention_ == CONV_AUTO ? "detected" : "forced",
           this->ac_voltage_[0], this->ac_voltage_[1], this->ac_voltage_[2],
           this->ac_line_voltage_[0], this->ac_line_voltage_[1],
           this->ac_line_voltage_[2]);

  uint16_t raw_low = (uint16_t) lroundf(low * 10.0f);
  uint16_t raw_high = (uint16_t) lroundf(high * 10.0f);

  if (this->settings_[SET_GRID_V_LOW] != raw_low) {
    ESP_LOGI(TAG, "slot %u: grid low limit %.1f -> %.1f V (%s convention)",
             this->slot_index_, this->settings_[SET_GRID_V_LOW] * 0.1f, low,
             line ? "line" : "phase");
    this->settings_[SET_GRID_V_LOW] = raw_low;
    this->queue_write_(CMD_WRITE_SINGLE, HO_GRID_V_LOW, &raw_low, 1);
  }
  if (this->settings_[SET_GRID_V_HIGH] != raw_high) {
    ESP_LOGI(TAG, "slot %u: grid high limit %.1f -> %.1f V (%s convention)",
             this->slot_index_, this->settings_[SET_GRID_V_HIGH] * 0.1f, high,
             line ? "line" : "phase");
    this->settings_[SET_GRID_V_HIGH] = raw_high;
    this->queue_write_(CMD_WRITE_SINGLE, HO_GRID_V_HIGH, &raw_high, 1);
  }
  if (this->tgt_restart_delay_ > 0 &&
      this->settings_[SET_RESTART_DELAY] != this->tgt_restart_delay_) {
    ESP_LOGI(TAG, "slot %u: restart delay %u -> %u s", this->slot_index_,
             this->settings_[SET_RESTART_DELAY], this->tgt_restart_delay_);
    this->settings_[SET_RESTART_DELAY] = this->tgt_restart_delay_;
    uint16_t v = this->tgt_restart_delay_;
    this->queue_write_(CMD_WRITE_SINGLE, HO_RESTART_DELAY, &v, 1);
  }

  this->protection_applied_ = true;
  this->publish_settings_();
}

void GrowattInverter::set_setting(uint8_t field, float value) {
  if (field >= SET_COUNT)
    return;
  uint16_t raw = (uint16_t) lroundf(value / SETTING_SCALE[field]);
  this->settings_[field] = raw;
  this->queue_write_(CMD_WRITE_SINGLE, SETTING_ADDR[field], &raw, 1);
}

float GrowattInverter::get_setting(uint8_t field) const {
  return field < SET_COUNT ? this->settings_[field] * SETTING_SCALE[field] : 0.0f;
}

void GrowattInverter::set_phase(uint8_t p) {
  if (p > INV_PHASE_L3)
    return;
  this->phase_ = p;
  this->save_prefs_();
  ESP_LOGI(TAG, "slot %u: wired to L%u", this->slot_index_, p + 1);
}

void GrowattInverter::write_register(uint16_t address, uint16_t value) {
  this->queue_write_(CMD_WRITE_SINGLE, address, &value, 1);
}

void GrowattInverter::set_register_select(uint16_t address, select::Select *s) {
  if (this->reg_select_count_ >= MAX_REG_SELECTS)
    return;
  this->reg_select_addr_[this->reg_select_count_] = address;
  this->reg_select_[this->reg_select_count_] = s;
  this->reg_select_count_++;
}

void GrowattInverter::publish_reg_entities_(const std::vector<uint8_t> &data,
                                            uint16_t base, uint16_t count) {
  for (uint8_t i = 0; i < this->reg_select_count_; i++) {
    uint16_t a = this->reg_select_addr_[i];
    if (a >= base && a < base + count && this->reg_select_[i] != nullptr) {
      auto opt = this->reg_select_[i]->at(reg16(data, a - base));
      if (opt.has_value())
        this->reg_select_[i]->publish_state(opt.value());
    }
  }
  for (uint8_t i = 0; i < this->reg_switch_count_; i++) {
    uint16_t a = this->reg_switch_addr_[i];
    if (a >= base && a < base + count && this->reg_switch_[i] != nullptr)
      this->reg_switch_[i]->publish_state(reg16(data, a - base) ==
                                          this->reg_switch_on_[i]);
  }
}

void GrowattInverter::set_register_switch(uint16_t address, uint16_t on_value,
                                          switch_::Switch *s) {
  if (this->reg_switch_count_ >= MAX_REG_SWITCHES)
    return;
  this->reg_switch_addr_[this->reg_switch_count_] = address;
  this->reg_switch_on_[this->reg_switch_count_] = on_value;
  this->reg_switch_[this->reg_switch_count_] = s;
  this->reg_switch_count_++;
}

void GrowattInverter::set_ac_charge(bool on) {
  this->ac_charge_ = on;
  uint16_t v = on ? 1 : 0;
  this->queue_write_(CMD_WRITE_SINGLE, HO_BF_AC_CHARGE, &v, 1);
}

// ---------------------------- time windows ----------------------------

void GrowattInverter::set_window_part(uint8_t mode, uint8_t period, uint8_t part,
                                      uint8_t v) {
  if (mode >= MODE_COUNT || period >= PERIOD_COUNT || part >= PART_COUNT)
    return;
  TimeWindow &w = this->windows_[mode][period];
  switch (part) {
    case PART_START_HOUR: w.start_h = v > 23 ? 23 : v; break;
    case PART_START_MIN:  w.start_m = v > 59 ? 59 : v; break;
    case PART_STOP_HOUR:  w.stop_h = v > 23 ? 23 : v; break;
    default:              w.stop_m = v > 59 ? 59 : v; break;
  }
}

uint8_t GrowattInverter::get_window_part(uint8_t mode, uint8_t period,
                                         uint8_t part) const {
  if (mode >= MODE_COUNT || period >= PERIOD_COUNT || part >= PART_COUNT)
    return 0;
  const TimeWindow &w = this->windows_[mode][period];
  switch (part) {
    case PART_START_HOUR: return w.start_h;
    case PART_START_MIN:  return w.start_m;
    case PART_STOP_HOUR:  return w.stop_h;
    default:              return w.stop_m;
  }
}

void GrowattInverter::set_window_enabled(uint8_t mode, uint8_t period, bool on) {
  if (mode < MODE_COUNT && period < PERIOD_COUNT)
    this->windows_[mode][period].enabled = on;
}

bool GrowattInverter::get_window_enabled(uint8_t mode, uint8_t period) const {
  return (mode < MODE_COUNT && period < PERIOD_COUNT)
             ? this->windows_[mode][period].enabled
             : false;
}

// Windows may wrap past midnight (23:00-06:59 is accepted by the firmware and
// is what the inverters ship configured with), so a window is split at 00:00
// into up to two same day segments before comparing.
static bool ranges_overlap(uint16_t a1, uint16_t a2, uint16_t b1, uint16_t b2) {
  return a1 < b2 && b1 < a2;
}

static uint8_t split_window(const TimeWindow &w, uint16_t seg[2][2]) {
  uint16_t s = w.start_h * 60 + w.start_m;
  uint16_t e = w.stop_h * 60 + w.stop_m;
  if (s == e)
    return 0;  // empty window, covers nothing
  if (e > s) {
    seg[0][0] = s;
    seg[0][1] = e;
    return 1;
  }
  seg[0][0] = s;
  seg[0][1] = 1440;
  seg[1][0] = 0;
  seg[1][1] = e;
  return 2;
}

static bool window_pair_overlaps(const TimeWindow &a, const TimeWindow &b) {
  uint16_t as[2][2], bs[2][2];
  uint8_t an = split_window(a, as);
  uint8_t bn = split_window(b, bs);
  for (uint8_t i = 0; i < an; i++)
    for (uint8_t j = 0; j < bn; j++)
      if (ranges_overlap(as[i][0], as[i][1], bs[j][0], bs[j][1]))
        return true;
  return false;
}

// The firmware rejects an enabled Grid First period that overlaps an enabled
// Battery First period. Catching it here gives a readable message instead of a
// bare Modbus exception.
bool GrowattInverter::windows_overlap(std::string *reason) const {
  for (uint8_t g = 0; g < PERIOD_COUNT; g++) {
    const TimeWindow &gw = this->windows_[MODE_GRID_FIRST][g];
    if (!gw.enabled)
      continue;
    for (uint8_t b = 0; b < PERIOD_COUNT; b++) {
      const TimeWindow &bw = this->windows_[MODE_BATTERY_FIRST][b];
      if (!bw.enabled)
        continue;
      if (window_pair_overlaps(gw, bw)) {
        if (reason != nullptr) {
          char buf[96];
          snprintf(buf, sizeof(buf),
                   "grid first period %u (%02u:%02u-%02u:%02u) overlaps "
                   "battery first period %u (%02u:%02u-%02u:%02u)",
                   g + 1, gw.start_h, gw.start_m, gw.stop_h, gw.stop_m, b + 1,
                   bw.start_h, bw.start_m, bw.stop_h, bw.stop_m);
          *reason = buf;
        }
        return true;
      }
    }
  }
  return false;
}

bool GrowattInverter::apply_windows(uint8_t mode) {
  if (mode >= MODE_COUNT)
    return false;
  if (!this->caps_.has_storage) {
    ESP_LOGW(TAG, "slot %u: no storage, ignoring window apply",
             this->slot_index_);
    return false;
  }

  std::string reason;
  if (this->windows_overlap(&reason)) {
    ESP_LOGE(TAG, "slot %u: refusing to write, %s", this->slot_index_,
             reason.c_str());
    return false;
  }

  uint16_t regs[WINDOW_REGS];
  for (uint8_t p = 0; p < PERIOD_COUNT; p++) {
    const TimeWindow &w = this->windows_[mode][p];
    regs[p * 3 + 0] = ((uint16_t) w.start_h << 8) | w.start_m;
    regs[p * 3 + 1] = ((uint16_t) w.stop_h << 8) | w.stop_m;
    regs[p * 3 + 2] = w.enabled ? 1 : 0;
  }
  uint16_t base = (mode == MODE_GRID_FIRST) ? HO_GF_WINDOW_BASE
                                            : HO_BF_WINDOW_BASE;
  ESP_LOGI(TAG, "slot %u: applying %s windows to %u",
           this->slot_index_,
           mode == MODE_GRID_FIRST ? "grid first" : "battery first", base);
  return this->queue_write_(CMD_WRITE_MULTI, base, regs, WINDOW_REGS);
}

void GrowattInverter::set_window_number(uint8_t mode, uint8_t period,
                                        uint8_t part, number::Number *n) {
  if (mode < MODE_COUNT && period < PERIOD_COUNT && part < PART_COUNT)
    this->win_num_[mode][period][part] = n;
}
void GrowattInverter::set_window_switch(uint8_t mode, uint8_t period,
                                        switch_::Switch *s) {
  if (mode < MODE_COUNT && period < PERIOD_COUNT)
    this->win_sw_[mode][period] = s;
}
void GrowattInverter::set_setting_number(uint8_t field, number::Number *n) {
  if (field < SET_COUNT)
    this->setting_num_[field] = n;
}

// Reads back everything the UI can change so the entities start out matching
// the inverter instead of showing zeros.
void GrowattInverter::parse_settings_(const std::vector<uint8_t> &data) {
  // offsets relative to 1070
  for (uint8_t f = 0; f < SET_COUNT; f++) {
    uint16_t addr = SETTING_ADDR[f];
    if (addr >= HO_SETTINGS_BASE && addr < HO_SETTINGS_BASE + HO_SETTINGS_CNT)
      this->settings_[f] = reg16(data, addr - HO_SETTINGS_BASE);
  }
  this->ac_charge_ = reg16(data, HO_BF_AC_CHARGE - HO_SETTINGS_BASE) != 0;
  this->publish_reg_entities_(data, HO_SETTINGS_BASE, HO_SETTINGS_CNT);

  for (uint8_t m = 0; m < MODE_COUNT; m++) {
    uint16_t base = (m == MODE_GRID_FIRST) ? HO_GF_WINDOW_BASE
                                           : HO_BF_WINDOW_BASE;
    size_t off = base - 1070;
    for (uint8_t p = 0; p < PERIOD_COUNT; p++) {
      uint16_t start = reg16(data, off + p * 3 + 0);
      uint16_t stop = reg16(data, off + p * 3 + 1);
      TimeWindow &w = this->windows_[m][p];
      w.start_h = start >> 8;
      w.start_m = start & 0xFF;
      w.stop_h = stop >> 8;
      w.stop_m = stop & 0xFF;
      w.enabled = reg16(data, off + p * 3 + 2) != 0;
    }
  }
  this->publish_settings_();
  ESP_LOGI(TAG, "slot %u: settings read back (GF %u%%/%u%%, BF %u%%/%u%%, AC %s)",
           this->slot_index_, this->settings_[SET_GF_DISCHARGE_RATE],
           this->settings_[SET_GF_STOP_SOC], this->settings_[SET_BF_CHARGE_RATE],
           this->settings_[SET_BF_STOP_SOC], this->ac_charge_ ? "on" : "off");
}

void GrowattInverter::publish_settings_() {
  for (uint8_t f = 0; f < SET_COUNT; f++) {
    if (this->setting_num_[f] != nullptr)
      this->setting_num_[f]->publish_state(this->get_setting(f));
  }
  if (this->ac_charge_sw_ != nullptr)
    this->ac_charge_sw_->publish_state(this->ac_charge_);
  if (this->phase_select_ != nullptr) {
    static const char *const NAMES[3] = {"L1", "L2", "L3"};
    this->phase_select_->publish_state(NAMES[this->phase_]);
  }
  for (uint8_t m = 0; m < MODE_COUNT; m++) {
    for (uint8_t p = 0; p < PERIOD_COUNT; p++) {
      for (uint8_t k = 0; k < PART_COUNT; k++) {
        if (this->win_num_[m][p][k] != nullptr)
          this->win_num_[m][p][k]->publish_state(
              this->get_window_part(m, p, k));
      }
      if (this->win_sw_[m][p] != nullptr)
        this->win_sw_[m][p]->publish_state(this->windows_[m][p].enabled);
    }
  }
}

// ------------------------------ dump ------------------------------

void GrowattInverter::send_dump_chunk_() {
  if (this->dump_range_ >= DUMP_RANGE_COUNT) {
    this->dump_active_ = false;
    ESP_LOGI(TAG, "slot %u: === REGISTER DUMP END ===", this->slot_index_);
    return;
  }
  const DumpRange &r = DUMP_RANGES[this->dump_range_];
  uint16_t remaining = r.count - this->dump_offset_;
  uint16_t n = remaining > DUMP_CHUNK ? DUMP_CHUNK : remaining;

  this->send(r.function, r.start + this->dump_offset_, n);
  this->last_send_ = millis();
  this->waiting_ = true;
}

void GrowattInverter::handle_dump_(const std::vector<uint8_t> &data) {
  const DumpRange &r = DUMP_RANGES[this->dump_range_];
  uint16_t base = r.start + this->dump_offset_;
  size_t regs = data.size() / 2;

  char line[96];
  for (size_t i = 0; i < regs; i += 8) {
    int pos = 0;
    for (size_t j = i; j < i + 8 && j < regs; j++)
      pos += snprintf(line + pos, sizeof(line) - pos, "%04X ", reg16(data, j));
    ESP_LOGI(TAG, "DUMP a%u %s %u: %s", this->address_,
             r.function == CMD_READ_HOLDING ? "hold" : "inp",
             (unsigned) (base + i), line);
  }

  this->retries_ = 0;
  this->dump_offset_ += regs;
  if (this->dump_offset_ >= r.count) {
    this->dump_range_++;
    this->dump_offset_ = 0;
  }
  if (this->dump_range_ >= DUMP_RANGE_COUNT) {
    this->dump_active_ = false;
    ESP_LOGI(TAG, "slot %u: === REGISTER DUMP END ===", this->slot_index_);
  } else {
    this->want_send_ = true;
  }
}

void GrowattInverter::dump_skip_range_() {
  this->retries_ = 0;
  this->dump_range_++;
  this->dump_offset_ = 0;
  if (this->dump_range_ >= DUMP_RANGE_COUNT) {
    this->dump_active_ = false;
    ESP_LOGI(TAG, "slot %u: === REGISTER DUMP END ===", this->slot_index_);
  } else {
    this->want_send_ = true;
  }
}

// ------------------------------ responses ------------------------------

void GrowattInverter::on_modbus_error(uint8_t function_code,
                                      uint8_t exception_code) {
  if (!this->is_enabled() || !this->waiting_)
    return;

  // An exception is still proof the inverter is alive and talking.
  this->last_update_ = micros();
  this->waiting_ = false;
  this->bus_release_ = millis();

  ESP_LOGD(TAG, "slot %u: exception %u on function 0x%02X", this->slot_index_,
           exception_code, function_code);

  // A write must be dealt with first: it is not part of the identification
  // sequence, and letting it fall through would advance that state machine on
  // the strength of an unrelated failure.
  if (this->writing_) {
    const PendingWrite &w = this->write_queue_[this->write_head_];
    ESP_LOGW(TAG,
             "slot %u: register %u rejected the write (exception %u); this "
             "model does not accept it, not trying again until the next "
             "identification",
             this->slot_index_, w.address, exception_code);
    this->mark_rejected_(w.address);
    this->writing_ = false;
    this->write_head_ = (this->write_head_ + 1) % WRITE_QUEUE_SIZE;
    this->write_count_--;
    this->retries_ = 0;
    this->want_send_ = this->write_count_ > 0;
    return;
  }

  if (this->dump_active_) {
    ESP_LOGI(TAG, "DUMP slot %u: range %u not implemented, skipping",
             this->slot_index_, this->dump_range_);
    this->dump_skip_range_();
  } else if (this->poll_ != POLL_IDLE) {
    this->advance_poll_();
  } else {
    this->advance_(false);
  }
}

void GrowattInverter::on_modbus_data(const std::vector<uint8_t> &data) {
  if (!this->is_enabled())
    return;

  // Any frame from our address proves the inverter is alive.
  this->last_update_ = micros();

  if (!this->waiting_)
    return;
  this->waiting_ = false;
  this->bus_release_ = millis();

  // A probe answering is all we needed; update_health_() notices the fresh
  // timestamp on the next cycle and starts identification.
  if (this->probing_) {
    this->probing_ = false;
    this->retries_ = 0;
    return;
  }

  // A write echo just confirms the command; nothing to parse.
  if (this->writing_) {
    this->writing_ = false;
    ESP_LOGI(TAG, "slot %u: write to %u acknowledged", this->slot_index_,
             this->write_queue_[this->write_head_].address);
    this->write_head_ = (this->write_head_ + 1) % WRITE_QUEUE_SIZE;
    this->write_count_--;
    this->retries_ = 0;
    this->want_send_ = this->write_count_ > 0;
    return;
  }

  if (this->dump_active_) {
    this->handle_dump_(data);
    return;
  }

  if (this->poll_ != POLL_IDLE) {
    switch (this->poll_) {
      case POLL_FAST_MAIN:
        if (data.size() >= POLL_FAST_MAIN_CNT * 2)
          this->parse_fast_main_(data);
        break;
      case POLL_FAST_STATUS:
        if (data.size() >= POLL_FAST_STATUS_CNT * 2)
          this->parse_fast_status_(data);
        break;
      case POLL_FAST_BAT:
        if (data.size() >= POLL_FAST_BAT_CNT * 2)
          this->parse_fast_bat_(data);
        break;
      case POLL_FAST_UPS:
        if (data.size() >= POLL_FAST_UPS_CNT * 2)
          this->parse_fast_ups_(data);
        break;
      case POLL_SLOW_MAIN:
        if (data.size() >= POLL_SLOW_MAIN_CNT * 2)
          this->parse_slow_main_(data);
        break;
      case POLL_SLOW_STOR:
        if (data.size() >= POLL_SLOW_STOR_CNT * 2)
          this->parse_storage_(data);
        break;
      default:
        break;
    }
    this->advance_poll_();
    return;
  }

  switch (this->step_) {
    case IDENT_LIVE: {
      if (data.size() < FIRST_GROUP_CNT * 2) { this->advance_(false); return; }
      this->detect_from_live_(data);
      break;
    }
    case IDENT_INFO: {
      if (data.size() < FIRST_GROUP_CNT * 2) { this->advance_(false); return; }
      this->parse_device_info_(data);
      break;
    }
    case IDENT_TYPE: {
      if (data.size() < REG_TYPE_CNT * 2) { this->advance_(false); return; }
      this->caps_.inv_type = ascii_from(data, OFF_INV_TYPE, OFF_INV_TYPE_CNT);
      pub_text(this->model_ts_, this->caps_.inv_type);
      pub_text(this->bootloader_ts_,
               ascii_from(data, OFF_BOOTLOADER, OFF_BOOTLOADER_CNT));
      if (this->caps_.inv_type.empty()) {
        ESP_LOGD(TAG, "slot %u: no INV Type string (register not implemented)",
                 this->slot_index_);
      } else {
        ESP_LOGI(TAG, "slot %u: INV Type = '%s'", this->slot_index_,
                 this->caps_.inv_type.c_str());
      }
      break;
    }
    case IDENT_CAPS: {
      if (data.size() < REG_CAPS_CNT * 2) { this->advance_(false); return; }
      this->caps_.bdc_count = reg16(data, 1) & 0xFF;
      this->caps_.battery_packs = reg16(data, 2) & 0xFF;
      ESP_LOGI(TAG, "slot %u: PvStrScan=%u, BDC=%u, PackNum=%u",
               this->slot_index_, reg16(data, 0), this->caps_.bdc_count,
               this->caps_.battery_packs);
      break;
    }
    case IDENT_STORAGE: {
      if (data.size() < REG_STORAGE_CNT * 2) { this->advance_(false); return; }
      uint16_t acc = 0;
      for (uint8_t i = 0; i < REG_STORAGE_CHECK; i++)
        acc |= reg16(data, i);
      this->caps_.has_storage = (acc != 0);
      if (this->caps_.has_storage) {
        this->caps_.has_ups = (reg16(data, REG_UPS_OFFSET) & 0x01) != 0;
        ESP_LOGI(TAG, "slot %u: storage YES, UPS %s", this->slot_index_,
                 this->caps_.has_ups ? "enabled" : "disabled");
        // battery type (1048) and the UPS enable switch (1060) live here
        this->publish_reg_entities_(data, REG_STORAGE_BASE, REG_STORAGE_CNT);
      } else {
        this->caps_.has_ups = false;
        ESP_LOGI(TAG, "slot %u: no battery config data -> grid-tie only",
                 this->slot_index_);
      }
      break;
    }
    case IDENT_BATTERY: {
      if (data.size() < REG_BAT_CNT * 2) { this->advance_(false); return; }
      uint16_t vbat = reg16(data, 0);
      uint16_t soc = reg16(data, 1);
      this->caps_.battery_soc = soc;
      this->caps_.has_battery = (vbat > 0 || soc > 0);
      if (this->caps_.has_battery) {
        ESP_LOGI(TAG, "slot %u: battery present (%.1f V, SOC %u%%)",
                 this->slot_index_, vbat / 10.0f, soc);
      } else {
        ESP_LOGI(TAG, "slot %u: no battery connected", this->slot_index_);
      }
      break;
    }
    case IDENT_SETTINGS: {
      if (data.size() < HO_SETTINGS_CNT * 2) { this->advance_(false); return; }
      this->parse_settings_(data);
      break;
    }
    default:
      return;
  }
  this->advance_(true);
}

// ------------------------------ results ------------------------------

void GrowattInverter::apply_overrides_() {
  if (this->cfg_phases_ > 0) {
    ESP_LOGD(TAG, "slot %u: phases forced from config: %d", this->slot_index_,
             this->cfg_phases_);
    this->caps_.phases = (uint8_t) this->cfg_phases_;
  }
  if (this->cfg_strings_ > 0) {
    ESP_LOGD(TAG, "slot %u: strings forced from config: %d", this->slot_index_,
             this->cfg_strings_);
    this->caps_.strings = (uint8_t) this->cfg_strings_;
  }
  if (this->cfg_ups_ != CFG_AUTO)
    this->caps_.has_ups = (this->cfg_ups_ != 0);
  if (this->cfg_battery_ != CFG_AUTO) {
    this->caps_.has_battery = (this->cfg_battery_ != 0);
    if (this->cfg_battery_ == 0)
      this->caps_.battery_packs = 0;
  }
}

void GrowattInverter::publish_info_() {
  // A single phase inverter left on the default phase is a likely oversight:
  // the unit cannot report which phase it feeds, and every per phase decision
  // the controller makes about it depends on getting this right.
  if (this->caps_.phases == 1 && this->phase_ == INV_PHASE_L1 &&
      this->phase_select_ != nullptr) {
    ESP_LOGW(TAG,
             "slot %u is single phase and set to L1; confirm this is the phase "
             "it actually feeds, nothing can detect it",
             this->slot_index_);
  }

  char bat[24] = "no battery";
  if (this->caps_.has_battery)
    snprintf(bat, sizeof(bat), "bat %u%%", this->caps_.battery_soc);

  // Growatt exposes no commercial model name over Modbus. Fall back from the
  // INV Type string to the serial, and always show the DTC, which is the only
  // reliable family discriminator (3601 = SPH 10000TL3, 5100 = MIN 6000TL-X).
  const char *name = "?";
  if (!this->caps_.inv_type.empty())
    name = this->caps_.inv_type.c_str();
  else if (!this->caps_.serial.empty())
    name = this->caps_.serial.c_str();

  // "N/M str" reads as strings connected out of trackers installed.
  char strings[16];
  if (this->caps_.trackers > 0)
    snprintf(strings, sizeof(strings), "%u/%u str", this->caps_.strings,
             this->caps_.trackers);
  else
    snprintf(strings, sizeof(strings), "%u str", this->caps_.strings);

  char buf[192];
  snprintf(buf, sizeof(buf), "%s | DTC %u | %uph | %s | %s | %s", name,
           this->caps_.dtc, this->caps_.phases, strings, bat,
           this->caps_.has_ups ? "UPS"
                               : (this->caps_.has_storage ? "storage" : "grid-tie"));

  ESP_LOGI(TAG, "slot %u @addr %u IDENTIFIED: %s", this->slot_index_,
           this->address_, buf);
  pub_text(this->info_ts_, std::string(buf));
}

void GrowattInverter::dump_config() {
  ESP_LOGCONFIG(TAG, "Growatt Inverter slot %u:", this->slot_index_);
  ESP_LOGCONFIG(TAG, "  Address: %u%s", this->address_,
                this->is_enabled() ? "" : " (EMPTY SLOT)");
  if (!this->is_enabled())
    return;
  ESP_LOGCONFIG(TAG, "  phases from config: %d (0=auto)", this->cfg_phases_);
  ESP_LOGCONFIG(TAG, "  strings from config: %d (0=auto)", this->cfg_strings_);
  ESP_LOGCONFIG(TAG, "  ups from config: %d (-1=auto)", this->cfg_ups_);
  ESP_LOGCONFIG(TAG, "  battery from config: %d (-1=auto)", this->cfg_battery_);
  ESP_LOGCONFIG(TAG, "  battery module: %.2f V, %.2f kWh, %.2f h discharge",
                this->module_voltage_, this->module_capacity_,
                this->discharge_hours_);
}

// --------------------- capability override selects ---------------------

void GrowattInverterAddressNumber::control(float value) {
  this->publish_state(value);
  if (this->parent_ != nullptr)
    this->parent_->change_address((uint8_t) lroundf(value));
}

void GrowattConventionSelect::control(const std::string &value) {
  this->publish_state(value);
  if (this->parent_ == nullptr)
    return;
  for (uint8_t i = 0; i < CONV_MODE_COUNT; i++) {
    if (value == CONV_NAMES[i]) {
      this->parent_->apply_convention(i);
      return;
    }
  }
}

void GrowattPhaseCountSelect::control(const std::string &value) {
  this->publish_state(value);
  if (this->parent_ == nullptr)
    return;
  int8_t v = 0;  // Auto
  if (value == OPT_SINGLE_PHASE)
    v = 1;
  else if (value == OPT_THREE_PHASE)
    v = 3;
  this->parent_->set_cfg_phases(v);
  // The override only takes effect through identification, which is also what
  // re-reads everything that was derived from the old answer.
  this->parent_->restart_identification();
}

void GrowattStringsSelect::control(const std::string &value) {
  this->publish_state(value);
  if (this->parent_ == nullptr)
    return;
  int8_t v = (value == OPT_AUTO) ? 0 : (int8_t) atoi(value.c_str());
  this->parent_->set_cfg_strings(v);
  this->parent_->restart_identification();
}

}  // namespace growatt_master
}  // namespace esphome
