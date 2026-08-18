#include "growatt_master.h"
#include "growatt_inverter.h"
#include "growatt_meter.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cmath>
#include <cstring>

namespace esphome {
namespace growatt_master {

static const char *const TAG = "growatt_master";

// Must match OFFLINE_ACTIONS in __init__.py.
static const char *const OFFLINE_ACTIONS[OFF_ACTION_COUNT] = {
    "Stop", "Hold", "Hold then reduce"};


static const char *health_text(uint8_t h) {
  switch (h) {
    case METER_ONLINE: return "online";
    case METER_STALLED: return "stalled";
    default: return "offline";
  }
}

void GrowattHub::setup() {
  uint32_t hash = fnv1_hash("growatt_hub_settings");
  this->pref_ = global_preferences->make_preference<GrowattHubPrefs>(hash);


  GrowattHubPrefs p{};
  if (this->pref_.load(&p)) {
    if (p.version == PREFS_VERSION) {
      for (uint8_t i = 0; i < HUB_SETTING_COUNT; i++)
        this->values_[i] = p.values[i];
      if (p.offline_action < OFF_ACTION_COUNT)
        this->offline_action_ = p.offline_action;
      ESP_LOGI(TAG, "restored settings from flash");
    } else {
      // Same size, different meaning. Falling back to the configured defaults
      // is the only safe reading of that.
      ESP_LOGW(TAG, "stored settings are version %u, expected %u - using defaults",
               p.version, PREFS_VERSION);
    }
  }
  for (uint8_t i = 0; i < HUB_SETTING_COUNT; i++)
    this->apply_setting_(i);
  this->publish_settings_();
  if (this->offline_select_ != nullptr)
    this->offline_select_->publish_state(OFFLINE_ACTIONS[this->offline_action_]);

  // The inverters use the same idea of stalled and offline as the meter does.
  // Not because they share a bus - they may not - but because it is the same
  // question, and a second set of timeouts would be two things to tune.
  for (auto *inv : this->inverters_) {
    inv->set_health_timeouts(this->stalled_ms_, this->offline_ms_);
    inv->set_offline_probe_interval(this->offline_probe_ms_);
  }
}

void GrowattHub::save_prefs_() {
  GrowattHubPrefs p{};
  p.version = PREFS_VERSION;
  p.offline_action = this->offline_action_;
  for (uint8_t i = 0; i < HUB_SETTING_COUNT; i++)
    p.values[i] = this->values_[i];
  this->pref_.save(&p);
}

// The thresholds are read straight out of values_ where they are used, but the
// tunables below are held in members that the control code reads on every pass,
// so a change has to be pushed into them. Doing it here rather than at each use
// keeps the hot path free of lookups.
void GrowattHub::apply_setting_(uint8_t field) {
  float v = this->values_[field];
  switch (field) {
    case HUB_UPDATE_INTERVAL: {
      uint32_t ms = (uint32_t) (v * 1000.0f);
      if (ms >= 100 && ms != this->get_update_interval()) {
        // A PollingComponent will not pick up a new interval on its own.
        this->stop_poller();
        this->set_update_interval(ms);
        this->start_poller();
        ESP_LOGI(TAG, "hub tick now %u ms", (unsigned) ms);
      }
      break;
    }
    case HUB_STEP_INTERVAL:     this->step_interval_ = (uint32_t) (v * 1000.0f); break;
    case HUB_REFRESH_INTERVAL:  this->refresh_interval_ = (uint32_t) (v * 1000.0f); break;
    case HUB_AVERAGE_SAMPLES:   this->set_avg_window((uint8_t) lroundf(v)); break;
    case HUB_STALLED_TIMEOUT:   this->stalled_ms_ = (uint32_t) (v * 1000.0f); break;
    case HUB_OFFLINE_TIMEOUT:   this->offline_ms_ = (uint32_t) (v * 1000.0f); break;
    case HUB_OFFLINE_PROBE:     this->offline_probe_ms_ = (uint32_t) (v * 1000.0f); break;
    case HUB_IMPORT_THRESHOLD:  this->import_threshold_ = v; break;
    case HUB_EXPORT_THRESHOLD:  this->export_threshold_ = v; break;
    case HUB_INCREASE_GAIN:     this->increase_gain_ = v; break;
    case HUB_DECREASE_GAIN:     this->decrease_gain_ = v; break;
    case HUB_MIN_STEP:          this->min_step_ = v; break;
    case HUB_MAX_STEP:          this->max_step_ = v; break;
    case HUB_STARTUP_RATE:      this->startup_rate_ = v; break;
    case HUB_OFFGRID_RATE:      this->offgrid_rate_ = v; break;
    case HUB_PROTECTION_MARGIN: this->protection_margin_ = v; break;
    case HUB_RESTART_DELAY:     this->restart_delay_s_ = (uint16_t) lroundf(v); break;
    case HUB_VOLTAGE_SOFT_MARGIN: this->voltage_soft_margin_ = v; break;
    default: return;  // a plain threshold, read from values_ where it is used
  }
  // The health timeouts and the protection margin are pushed down to the
  // inverters, which hold their own copies.
  if (field == HUB_STALLED_TIMEOUT || field == HUB_OFFLINE_TIMEOUT ||
      field == HUB_OFFLINE_PROBE) {
    for (auto *inv : this->inverters_) {
      inv->set_health_timeouts(this->stalled_ms_, this->offline_ms_);
      inv->set_offline_probe_interval(this->offline_probe_ms_);
    }
  }
}

void GrowattHub::set_setting(uint8_t field, float value) {
  if (field >= HUB_SETTING_COUNT)
    return;
  this->values_[field] = value;
  this->apply_setting_(field);
  this->save_prefs_();
}

void GrowattHub::publish_settings_() {
  for (uint8_t i = 0; i < HUB_SETTING_COUNT; i++) {
    if (this->setting_num_[i] != nullptr)
      this->setting_num_[i]->publish_state(this->values_[i]);
  }
}

// ========================= GrowattAddressTool =========================

// Silence is the only evidence an address is free, and a single missed frame
// would fake it, so the probe is repeated before the address is declared empty.
static const uint32_t ADDR_TIMEOUT_MS = 1000;
static const uint8_t ADDR_PROBE_TRIES = 3;

void GrowattAddressTool::start() {
  if (this->step_ != ADDR_IDLE) {
    ESP_LOGW(TAG, "%s address change: already running", this->label_);
    return;
  }
  // Both default to 0, which is the Modbus broadcast address and therefore
  // never a real device. An unset field means the button was pressed by
  // accident, so nothing happens at all.
  if (this->from_ == 0 || this->to_ == 0) {
    ESP_LOGW(TAG, "%s address change: 'from' and 'to' must both be set, ignoring",
             this->label_);
    return;
  }
  if (this->from_ == this->to_) {
    ESP_LOGW(TAG, "%s address change: %u to itself, ignoring", this->label_,
             this->to_);
    return;
  }

  ESP_LOGI(TAG, "%s address change: checking whether %u is free...", this->label_,
           this->to_);
  if (this->status_ != nullptr)
    this->status_->publish_state("CHECKING");
  this->step_ = ADDR_PROBE;
  this->tries_ = 0;
  this->waiting_ = false;
}

void GrowattAddressTool::send_() {
  if (this->step_ == ADDR_PROBE) {
    // Function 3 at register 0 is about the most universal question there is:
    // a device that implements it answers with data, one that does not answers
    // with an exception. Either way it has revealed itself, which is all the
    // probe needs to know. Two registers rather than one, because Eastron
    // rejects any request for an odd number and Growatt does not mind.
    this->address_ = this->to_;
    this->send(CMD_READ_HOLDING, 0, 2);
  } else {
    this->address_ = this->from_;
    if (this->float_format_) {
      // Eastron keeps the address as a float32 across two registers and only
      // accepts function 16.
      float v = (float) this->to_;
      uint32_t bits;
      memcpy(&bits, &v, sizeof(bits));
      const uint8_t payload[4] = {(uint8_t) (bits >> 24), (uint8_t) (bits >> 16),
                                  (uint8_t) (bits >> 8), (uint8_t) bits};
      this->send(CMD_WRITE_MULTI, this->addr_reg_, 2, 4, payload);
    } else {
      const uint8_t payload[2] = {0, this->to_};
      this->send(CMD_WRITE_SINGLE, this->addr_reg_, 1, 2, payload);
    }
  }
  this->sent_ = millis();
  this->waiting_ = true;
}

void GrowattAddressTool::finish_(const char *status, bool ok) {
  if (ok)
    ESP_LOGI(TAG, "%s address change: %s", this->label_, status);
  else
    ESP_LOGE(TAG, "%s address change: %s", this->label_, status);
  if (this->status_ != nullptr)
    this->status_->publish_state(status);
  this->step_ = ADDR_IDLE;
  this->waiting_ = false;
  this->address_ = 0;  // stop matching anything on the bus
}

void GrowattAddressTool::loop() {
  if (this->step_ == ADDR_IDLE)
    return;

  if (this->waiting_) {
    if (millis() - this->sent_ < ADDR_TIMEOUT_MS)
      return;
    this->waiting_ = false;

    if (this->step_ == ADDR_PROBE) {
      if (++this->tries_ < ADDR_PROBE_TRIES) {
        ESP_LOGD(TAG, "%s address change: no answer at %u (%u/%u)", this->label_,
                 this->to_, this->tries_, ADDR_PROBE_TRIES);
        return;  // sent again below on the next pass
      }
      ESP_LOGI(TAG, "%s address change: %u is free, writing %u -> %u",
               this->label_, this->to_, this->from_, this->to_);
      this->step_ = ADDR_WRITE;
      return;
    }

    // No echo for the write. It may genuinely have failed, or the unit may have
    // adopted the new address before replying, in which case the answer came
    // from an address we were not listening on. Reported as FAILED because that
    // is what we can prove, but check the new address before assuming the worst.
    uint8_t from = this->from_, to = this->to_;
    const char *label = this->label_;
    this->finish_("FAILED", false);
    ESP_LOGW(TAG,
             "%s address change: no echo from %u. The write may still have "
             "taken effect - a unit that switches before replying answers from "
             "%u, which we were not listening on. Check both addresses.",
             label, from, to);
    return;
  }

  if (!this->ready_for_immediate_send())
    return;
  this->send_();
}

void GrowattAddressTool::on_modbus_data(const std::vector<uint8_t> &data) {
  if (this->step_ == ADDR_IDLE || !this->waiting_)
    return;
  this->waiting_ = false;
  if (this->step_ == ADDR_PROBE) {
    this->finish_("NEW ADDRESS IS IN USE", false);
    return;
  }
  this->finish_("OK", true);
}

void GrowattAddressTool::on_modbus_error(uint8_t function_code,
                                         uint8_t exception_code) {
  if (this->step_ == ADDR_IDLE || !this->waiting_)
    return;
  this->waiting_ = false;
  if (this->step_ == ADDR_PROBE) {
    // An exception is still an answer, and an answer means something is there.
    ESP_LOGI(TAG, "%s address change: exception %u from %u - the address is taken",
             this->label_, exception_code, this->to_);
    this->finish_("NEW ADDRESS IS IN USE", false);
    return;
  }
  ESP_LOGE(TAG, "%s address change: exception %u on function 0x%02X",
           this->label_, exception_code, function_code);
  this->finish_("FAILED", false);
}

// ============================== GrowattHub ==============================

void GrowattHub::set_offline_action(uint8_t a) {
  if (a >= OFF_ACTION_COUNT)
    return;
  this->offline_action_ = a;
  this->save_prefs_();
  if (this->offline_select_ != nullptr)
    this->offline_select_->publish_state(OFFLINE_ACTIONS[a]);
  ESP_LOGI(TAG, "meter offline action: %s", OFFLINE_ACTIONS[a]);
}

void GrowattOfflineActionSelect::control(const std::string &value) {
  this->publish_state(value);
  if (this->parent_ == nullptr)
    return;
  for (uint8_t i = 0; i < OFF_ACTION_COUNT; i++) {
    if (value == OFFLINE_ACTIONS[i]) {
      this->parent_->set_offline_action(i);
      return;
    }
  }
}

bool GrowattHub::grid_available() const {
  // No contactor sensor declared means we have no way of telling, and assuming
  // the mains is present is the non-intrusive default.
  if (this->grid_power_bs_ == nullptr)
    return true;
  return this->grid_power_bs_->state;
}

// The first meter is the grid reference: it is what the voltage limit and the
// import/export figures are taken from.
void GrowattHub::update_meter_health_() {
  uint8_t h = METER_OFFLINE;
  this->meter_age_ms_ = 0;
  if (!this->meters_.empty()) {
    GrowattMeter *m = this->meters_[0];
    if (m->is_enabled() && m->get_last_update() != 0) {
      this->meter_age_ms_ = (micros() - m->get_last_update()) / 1000;
      if (this->meter_age_ms_ < this->stalled_ms_)
        h = METER_ONLINE;
      else if (this->meter_age_ms_ < this->offline_ms_)
        h = METER_STALLED;
    }
  }
  if (h != this->health_) {
    ESP_LOGW(TAG, "meter is %s (last frame %u ms ago)", health_text(h),
             (unsigned) this->meter_age_ms_);
    this->health_ = h;
    if (this->state_ts_ != nullptr)
      this->state_ts_->publish_state(health_text(h));
  }
}

void GrowattHub::update_aggregates_() {
  if (this->meters_.empty() || this->health_ == METER_OFFLINE) {
    this->import_w_ = NAN;
    this->export_w_ = NAN;
    return;
  }
  GrowattMeter *m = this->meters_[0];

  float imp = 0, exp = 0;
  bool any = false;
  for (uint8_t i = 0; i < 3; i++) {
    float p = m->get_phase_power(i);
    if (std::isnan(p))
      continue;
    any = true;
    if (p > 0)
      imp += p;
    else
      exp += -p;
  }
  if (!any) {
    this->import_w_ = NAN;
    this->export_w_ = NAN;
    return;
  }

  this->import_w_ = imp;
  this->export_w_ = exp;
  if (this->import_sens_ != nullptr)
    this->import_sens_->publish_state(imp);
  if (this->export_sens_ != nullptr)
    this->export_sens_->publish_state(exp);

  this->import_buf_[this->avg_pos_] = imp;
  this->export_buf_[this->avg_pos_] = exp;
  this->avg_pos_ = (this->avg_pos_ + 1) % this->avg_window_;
  if (this->avg_count_ < this->avg_window_)
    this->avg_count_++;

  float ai = 0, ae = 0;
  for (uint8_t i = 0; i < this->avg_count_; i++) {
    ai += this->import_buf_[i];
    ae += this->export_buf_[i];
  }
  if (this->import_avg_sens_ != nullptr)
    this->import_avg_sens_->publish_state(ai / this->avg_count_);
  if (this->export_avg_sens_ != nullptr)
    this->export_avg_sens_->publish_state(ae / this->avg_count_);
}

void GrowattHub::update_conditions_() {
  if (this->bs_grid_ != nullptr)
    this->bs_grid_->publish_state(this->grid_available());

  // Grid voltage is judged from the reference meter rather than from each
  // inverter: the readings differ by a few volts between units and the meter
  // is the one consistent source.
  bool over = false, under = false;
  if (!this->meters_.empty() && this->health_ != METER_OFFLINE) {
    GrowattMeter *m = this->meters_[0];
    for (uint8_t i = 0; i < 3; i++) {
      float v = m->get_phase_voltage(i);
      if (std::isnan(v) || v < 1.0f)
        continue;  // phase not connected
      if (v > this->values_[HUB_PHASE_V_HIGH])
        over = true;
      if (v < this->values_[HUB_PHASE_V_LOW])
        under = true;
    }
  }
  if (this->bs_over_v_ != nullptr)
    this->bs_over_v_->publish_state(over);
  if (this->bs_under_v_ != nullptr)
    this->bs_under_v_->publish_state(under);

  bool ups = false, ups_avg = false, soc_lo = false, soc_hi = false;
  for (auto *inv : this->inverters_) {
    if (!inv->is_enabled() || !inv->is_online())
      continue;
    float l = inv->get_ups_load();
    if (!std::isnan(l) && l > this->values_[HUB_UPS_MAX_LOAD])
      ups = true;
    float la = inv->get_ups_load_avg();
    if (!std::isnan(la) && la > this->values_[HUB_UPS_MAX_LOAD_AVG])
      ups_avg = true;
    float soc = inv->get_battery_soc();
    if (!std::isnan(soc)) {
      if (soc < this->values_[HUB_BATTERY_SOC_MIN])
        soc_lo = true;
      if (soc >= this->values_[HUB_BATTERY_SOC_MAX])
        soc_hi = true;
    }
  }
  if (this->bs_ups_ != nullptr)
    this->bs_ups_->publish_state(ups);
  if (this->bs_ups_avg_ != nullptr)
    this->bs_ups_avg_->publish_state(ups_avg);
  if (this->bs_soc_lo_ != nullptr)
    this->bs_soc_lo_->publish_state(soc_lo);
  if (this->bs_soc_hi_ != nullptr)
    this->bs_soc_hi_->publish_state(soc_hi);
}

// ============================== power controller ==============================

void GrowattHub::set_ctrl_state_(const char *s) {
  if (this->ctrl_state_ == s)
    return;
  this->ctrl_state_ = s;
  ESP_LOGI(TAG, "controller: %s", s);
  if (this->ctrl_ts_ != nullptr)
    this->ctrl_ts_->publish_state(s);
}

void GrowattHub::set_all_(float pct, const char *reason) {
  for (auto *inv : this->inverters_) {
    if (inv->is_enabled() && inv->is_online())
      inv->apply_power_rate(pct);
  }
  this->set_ctrl_state_(reason);
}

// The inverter may expect to hear from us regularly, so the current setpoint is
// rewritten periodically even when it has not changed. With holding 2 cleared
// these writes stay out of the EEPROM.
void GrowattHub::refresh_all_() {
  for (auto *inv : this->inverters_) {
    if (inv->is_enabled() && inv->is_online())
      inv->apply_power_rate(inv->get_power_percent());
  }
}

float GrowattHub::headroom_up_(GrowattInverter *inv, const float *err) {
  if (inv->get_phases() >= 3) {
    // Spreads evenly over three phases, so the phase with the least headroom
    // decides. Moving that phase by E needs three times E in total.
    float worst = NAN;
    for (uint8_t i = 0; i < 3; i++) {
      if (std::isnan(err[i]))
        continue;
      if (std::isnan(worst) || err[i] < worst)
        worst = err[i];
    }
    return std::isnan(worst) ? 0 : worst * 3.0f;
  }
  uint8_t p = inv->get_phase();
  if (p > 2 || std::isnan(err[p]))
    return 0;
  return err[p];
}

float GrowattHub::excess_on_(GrowattInverter *inv, const float *err,
                             uint8_t phase) {
  if (phase > 2 || std::isnan(err[phase]))
    return 0;
  float e = fabsf(err[phase]);
  return inv->get_phases() >= 3 ? e * 3.0f : e;
}

// The active power register only takes whole percent, and apply_power_rate()
// rounds to it. A step under half a percent therefore changes nothing: the
// cycle spends a write, the inverter lands on the value it already had, and the
// next cycle sees the same state and repeats. Every step is rounded and floored
// at one so an iteration always moves something.
static const float MIN_EFFECTIVE_STEP = 1.0f;

float GrowattHub::step_for_(GrowattInverter *inv, float power_w, float gain) {
  // Without a trustworthy nameplate figure the proportional term cannot be
  // computed, so fall back to the smallest step rather than guessing.
  float step = this->min_step_;
  if (inv->has_valid_normal_power())
    step = gain * fabsf(power_w) / inv->get_normal_power() * 100.0f;
  if (step < this->min_step_)
    step = this->min_step_;
  if (step > this->max_step_)
    step = this->max_step_;
  step = roundf(step);
  if (step < MIN_EFFECTIVE_STEP)
    step = MIN_EFFECTIVE_STEP;
  return step;
}

// An inverter delivering less than this to the grid cannot make the export any
// smaller, whatever its setpoint says. A storage unit charging its battery at
// full PV power is exactly that case: 100 % commanded, nothing going out.
static const float MIN_INJECTING_W = 50.0f;

// Phase indices ordered by how badly they deviate, worst first. Acting on the
// worst phase before the others keeps a single bad phase from waiting behind
// two mild ones.
static void order_phases(const float *v, uint8_t *out) {
  out[0] = 0;
  out[1] = 1;
  out[2] = 2;
  for (uint8_t i = 0; i < 2; i++) {
    for (uint8_t j = i + 1; j < 3; j++) {
      float a = std::isnan(v[out[i]]) ? -1e9f : v[out[i]];
      float b = std::isnan(v[out[j]]) ? -1e9f : v[out[j]];
      if (b > a) {
        uint8_t t = out[i];
        out[i] = out[j];
        out[j] = t;
      }
    }
  }
}

void GrowattHub::control_power_() {
  uint32_t now = millis();

  // Off grid: the mains is gone, so export is impossible and every inverter is
  // needed at once to carry the house.
  if (!this->grid_available()) {
    if (now - this->last_step_ >= this->step_interval_) {
      this->last_step_ = now;
      this->set_all_(this->offgrid_rate_, "off grid, all inverters at maximum");
    }
    return;
  }

  // Meter gone while connected to the mains: we can no longer tell whether we
  // are exporting. What follows is a judgement call the site owner has to make,
  // so it is theirs to set rather than ours to assume.
  if (this->health_ == METER_OFFLINE) {
    if (this->offline_since_ == 0)
      this->offline_since_ = now;
    if (now - this->last_step_ < this->step_interval_)
      return;
    this->last_step_ = now;
    if (!this->offline_logged_) {
      ESP_LOGW(TAG, "meter offline, last frame %u ms ago",
               (unsigned) this->meter_age_ms_);
      this->offline_logged_ = true;
    }

    if (this->offline_action_ == OFF_STOP) {
      this->set_all_(0, "meter offline, production stopped");
      return;
    }
    if (this->offline_action_ == OFF_HOLD) {
      this->set_ctrl_state_("meter offline, holding");
      return;
    }

    // OFF_DECAY: hold first, because most outages are brief and cutting a
    // healthy system for a lost frame is its own kind of failure. Only once the
    // meter has stayed gone does the setpoint start walking down.
    uint32_t gone = now - this->offline_since_;
    if (gone < this->offline_hold_ms_) {
      this->set_ctrl_state_("meter offline, holding");
      return;
    }
    bool moved = false;
    for (size_t i = 0; i < this->inverters_.size(); i++) {
      GrowattInverter *inv = this->inverters_[i];
      if (!inv->is_enabled() || !inv->is_online())
        continue;
      uint8_t safe = inv->get_safe_power_rate();
      uint8_t at = inv->get_power_percent();
      if (at <= safe)
        continue;
      float from = at;
      // One step at a time rather than straight to the safe rate: if the meter
      // comes back mid-descent, little has been lost.
      float target = from - this->min_step_;
      if (target < safe)
        target = safe;
      inv->apply_power_rate(target);
      ESP_LOGW(TAG, "meter gone %u s -> slot %u %.0f%% to %u%% (safe %u%%)",
               (unsigned) (gone / 1000), (unsigned) i, from,
               inv->get_power_percent(), safe);
      moved = true;
    }
    this->set_ctrl_state_(moved ? "meter offline, reducing to safe"
                                : "meter offline, at safe rate");
    return;
  }

  this->offline_since_ = 0;
  this->offline_logged_ = false;

  // A few missed frames are not a reason to move anything.
  if (this->health_ == METER_STALLED) {
    this->set_ctrl_state_("meter stalled, holding");
    return;
  }

  if (now - this->last_step_ < this->step_interval_) {
    if (this->refresh_interval_ > 0 &&
        now - this->last_refresh_ >= this->refresh_interval_) {
      this->last_refresh_ = now;
      ESP_LOGD(TAG, "refreshing setpoints (watchdog)");
      this->refresh_all_();
    }
    return;
  }
  this->last_step_ = now;
  this->last_refresh_ = now;

  if (this->meters_.empty())
    return;
  GrowattMeter *m = this->meters_[0];

  // Positive means importing on that phase, negative means exporting.
  float err[3];
  float volt[3];
  for (uint8_t i = 0; i < 3; i++) {
    err[i] = m->get_phase_power(i);
    volt[i] = m->get_phase_voltage(i);
  }

  float limit = this->values_[HUB_PHASE_V_HIGH];
  float line_limit = this->values_[HUB_LINE_V_HIGH];

  // Diagnostics only; the checks below decide per phase and per inverter. The
  // meter is the grid reference, but it is not the first to see a rise: the
  // drop across the AC cabling means an inverter measures more at its own
  // terminals than the meter does, and it is the inverter that trips.
  bool meter_over = false;
  for (uint8_t i = 0; i < 3; i++) {
    if (!std::isnan(volt[i]) && volt[i] > limit)
      meter_over = true;
  }
  bool any_inverter_over = false;
  for (auto *inv : this->inverters_) {
    if (!inv->is_enabled() || !inv->is_online())
      continue;
    float p = inv->peak_ac_voltage();
    float l = inv->peak_line_voltage();
    float pl = inv->ac_voltage_is_line() ? line_limit : limit;
    if ((!std::isnan(p) && p > pl) || (!std::isnan(l) && l > line_limit))
      any_inverter_over = true;
  }
  float export_cap = this->values_[HUB_GRID_EXPORT_LIMIT];
  bool hard_export = (export_cap > 0 && !std::isnan(this->export_w_) &&
                      this->export_w_ > export_cap);

  ESP_LOGD(TAG, "=== control cycle, meter %u ms old ===",
           (unsigned) this->meter_age_ms_);
  ESP_LOGD(TAG, "  L1 %+.0f W %.1f V | L2 %+.0f W %.1f V | L3 %+.0f W %.1f V",
           err[0], volt[0], err[1], volt[1], err[2], volt[2]);
  ESP_LOGD(TAG,
           "  import %.0f W (threshold %.0f), export %.0f W (threshold %.0f, "
           "cap %.0f%s), voltage limit %.1f V phase / %.1f V line%s%s",
           this->import_w_, this->import_threshold_, this->export_w_,
           this->export_threshold_, export_cap,
           hard_export ? ", EXCEEDED" : "", limit, line_limit,
           meter_over ? ", METER OVER" : "",
           any_inverter_over ? ", INVERTER OVER" : "");

  for (size_t i = 0; i < this->inverters_.size(); i++) {
    GrowattInverter *inv = this->inverters_[i];
    if (!inv->is_enabled()) {
      ESP_LOGD(TAG, "  slot %u: disabled", (unsigned) i);
      continue;
    }
    if (!inv->is_online()) {
      ESP_LOGD(TAG, "  slot %u: %s, excluded from control", (unsigned) i,
               inv->health_text());
      continue;
    }
    char wiring[8];
    if (inv->get_phases() >= 3)
      snprintf(wiring, sizeof(wiring), "3p");
    else
      snprintf(wiring, sizeof(wiring), "1p L%u", inv->get_phase() + 1);
    // The unit's own measurement, which is what its protection acts on and is
    // higher than the meter's by whatever the AC cabling drops. Labelled with
    // the convention so it is comparable to the right threshold at a glance.
    // Three phase units print all three registers; a single phase unit only
    // ever fills the first, whatever grid phase it is actually wired to.
    // Phase and line are different registers reporting different things, and a
    // MOD populates both at once. Printed separately rather than picked
    // between, because guessing which one a unit "uses" is what hid a bug.
    char volts[72];
    {
      char *w = volts;
      uint8_t n = inv->get_phases() >= 3 ? 3 : 1;
      for (uint8_t q = 0; q < n; q++) {
        float v = inv->get_ac_voltage(q);
        // A unit in standby reports a real zero; one that has published nothing
        // reports nothing. Those are different states.
        if (std::isnan(v))
          w += snprintf(w, sizeof(volts) - (w - volts), "%s-", q ? "/" : "");
        else
          w += snprintf(w, sizeof(volts) - (w - volts), "%s%.1f", q ? "/" : "",
                        v);
      }
      w += snprintf(w, sizeof(volts) - (w - volts), " %s",
                    inv->ac_voltage_is_line() ? "Vll" : "Vph");
      float l = inv->peak_line_voltage();
      if (!std::isnan(l) && l > 0)
        snprintf(w, sizeof(volts) - (w - volts), " / %.1f Vll", l);
    }
    ESP_LOGD(TAG,
             "  slot %u: %s, rate %u%% [%u..%u], injecting %.0f W, %s, "
             "derating %u (%s), can produce more %s, %+.0f W of room to grow",
             (unsigned) i, wiring, inv->get_power_percent(),
             inv->get_min_power_rate(), inv->get_max_power_rate(),
             std::isnan(inv->get_grid_power()) ? 0.0f : inv->get_grid_power(),
             volts, inv->get_derating_mode(), inv->get_derating_text(),
             inv->can_produce_more() ? "yes" : "no",
             this->headroom_up_(inv, err));
  }

  // ---- above the contractual export cap: cut everything at once ----
  // Per phase trimming is the right shape for ordinary export, but a breach of
  // a contractual limit is a different problem: one inverter per cycle is far
  // too slow, so every unit feeding the grid comes down together.
  if (hard_export) {
    bool cut = false;
    for (size_t i = 0; i < this->inverters_.size(); i++) {
      GrowattInverter *inv = this->inverters_[i];
      if (!inv->is_enabled() || !inv->is_online())
        continue;
      if (inv->get_power_percent() <= inv->get_min_power_rate())
        continue;
      float out = inv->get_grid_power();
      if (std::isnan(out) || out < MIN_INJECTING_W)
        continue;
      float from = inv->get_power_percent();
      inv->apply_power_rate(from - this->max_step_);
      ESP_LOGW(TAG, "export %.0f W over the %.0f W cap -> slot %u %.0f%% to %u%%",
               this->export_w_, export_cap, (unsigned) i, from,
               inv->get_power_percent());
      cut = true;
    }
    if (cut) {
      this->set_ctrl_state_("over export limit, reducing");
      return;
    }
  }

  // ---- voltage check 1: the meter sees a phase over the limit ----
  // Only phase voltages are consulted. The meter's line to line block is a
  // separate transaction and an optional one, so depending on it would make a
  // safety check conditional on which sensors happen to be declared - and on a
  // three phase meter the line figures are derived from these anyway.
  //
  // Everything feeding that phase comes down, three phase units included: they
  // cannot trim one phase alone, but they are still injecting into it, and the
  // alternative is leaving the rise to the hardware protection.
  int8_t worst = -1;
  for (uint8_t i = 0; i < 3; i++) {
    if (std::isnan(volt[i]) || volt[i] <= limit)
      continue;
    if (worst < 0 || volt[i] > volt[worst])
      worst = (int8_t) i;
  }
  if (worst >= 0) {
    bool cut = false;
    for (size_t i = 0; i < this->inverters_.size(); i++) {
      GrowattInverter *inv = this->inverters_[i];
      if (!inv->is_enabled() || !inv->is_online())
        continue;
      if (inv->get_phases() < 3 && inv->get_phase() != (uint8_t) worst)
        continue;
      if (inv->get_power_percent() <= inv->get_min_power_rate())
        continue;
      float from = inv->get_power_percent();
      inv->apply_power_rate(from - this->min_step_);
      ESP_LOGW(TAG, "L%d at %.1f V, above the %.1f V limit -> slot %u %.0f%% to %u%%",
               worst + 1, volt[worst], limit, (unsigned) i, from,
               inv->get_power_percent());
      cut = true;
    }
    if (cut) {
      this->set_ctrl_state_("grid voltage high, reducing");
      return;
    }
  }

  // ---- voltage check 2: an inverter sees more at its own terminals ----
  // The drop across the AC cabling means a unit can be over its trip threshold
  // while the meter still reads normal. Only the offending unit is touched;
  // nothing else is contributing to what it measures.
  {
    bool cut = false;
    for (size_t i = 0; i < this->inverters_.size(); i++) {
      GrowattInverter *inv = this->inverters_[i];
      if (!inv->is_enabled() || !inv->is_online())
        continue;
      float own = inv->peak_ac_voltage();
      float own_line_v = inv->peak_line_voltage();
      bool ac_is_line = inv->ac_voltage_is_line();
      float ac_limit = ac_is_line ? line_limit : limit;
      bool over_ac = !std::isnan(own) && own > ac_limit;
      bool over_line = !std::isnan(own_line_v) && own_line_v > line_limit;
      if (!over_ac && !over_line)
        continue;
      bool own_line = over_ac ? ac_is_line : true;
      float own_limit = over_ac ? ac_limit : line_limit;
      if (!over_ac)
        own = own_line_v;
      if (inv->get_power_percent() <= inv->get_min_power_rate())
        continue;
      float from = inv->get_power_percent();
      inv->apply_power_rate(from - this->min_step_);
      ESP_LOGW(TAG,
               "slot %u at %.1f V on its own terminals, above the %.1f V %s "
               "limit -> %.0f%% to %u%%",
               (unsigned) i, own, own_limit, own_line ? "line" : "phase", from,
               inv->get_power_percent());
      cut = true;
    }
    if (cut) {
      this->set_ctrl_state_("inverter voltage high, reducing");
      return;
    }
  }

  // ---- priority zero: stop exporting, worst phase first ----
  // Every watt injected costs money, so this runs before anything else and
  // before any thought of raising production. The inverter list is walked
  // backwards here: the last declared unit is the least important one, so it
  // is the first to give way.
  float exported[3];
  for (uint8_t i = 0; i < 3; i++)
    exported[i] = std::isnan(err[i]) ? NAN : -err[i];

  uint8_t order[3];
  order_phases(exported, order);

  bool acted = false;
  for (uint8_t k = 0; k < 3; k++) {
    uint8_t p = order[k];
    if (std::isnan(exported[p]) || exported[p] <= this->export_threshold_)
      continue;

    // Two rounds: single phase units wired to this phase first, because
    // trimming a three phase unit would also cut phases that may still be
    // importing.
    GrowattInverter *chosen = nullptr;
    bool chosen_single = false;
    for (uint8_t round = 0; round < 2 && chosen == nullptr; round++) {
      for (int i = (int) this->inverters_.size() - 1; i >= 0; i--) {
        GrowattInverter *inv = this->inverters_[i];
        if (!inv->is_enabled() || !inv->is_online())
          continue;
        if (inv->get_power_percent() <= inv->get_min_power_rate())
          continue;
        // Only units actually feeding the grid can shrink the export.
        float out = inv->get_grid_power();
        if (std::isnan(out) || out < MIN_INJECTING_W) {
          if (round == 0)
            ESP_LOGD(TAG, "  slot %u injects %.0f W, cannot help with L%u",
                     (unsigned) i, std::isnan(out) ? 0.0f : out, p + 1);
          continue;
        }
        bool single_here = inv->get_phases() < 3 && inv->get_phase() == p;
        bool three_phase = inv->get_phases() >= 3;
        if (round == 0 && !single_here)
          continue;
        if (round == 1 && !three_phase)
          continue;
        chosen = inv;
        chosen_single = single_here;
        break;
      }
    }
    if (chosen == nullptr) {
      ESP_LOGD(TAG, "  L%u exporting %.0f W but no inverter can be reduced",
               p + 1, exported[p]);
      continue;
    }

    float power = this->excess_on_(chosen, err, p);
    float step = this->step_for_(chosen, power, this->decrease_gain_);
    float from = chosen->get_power_percent();
    chosen->apply_power_rate(from - step);
    ESP_LOGI(TAG, "L%u exporting %.0f W -> %s inverter down %.1f%%, %.0f%% to %u%%",
             p + 1, exported[p], chosen_single ? "single phase" : "three phase",
             step, from, chosen->get_power_percent());
    acted = true;
  }
  if (acted) {
    this->set_ctrl_state_("reducing to stop export");
    return;
  }

  // ---- then one increase, whichever inverter can absorb the most ----
  // Comparing usable power rather than applying a fixed preference settles the
  // single versus three phase question on its own: with the import spread
  // evenly a three phase unit covers three times the weakest phase, which beats
  // any single phase unit, while a lopsided import makes the single phase unit
  // on the heavy phase the better choice.
  GrowattInverter *best = nullptr;
  size_t best_i = 0;
  float best_power = 0;

  for (size_t i = 0; i < this->inverters_.size(); i++) {
    GrowattInverter *inv = this->inverters_[i];
    if (!inv->is_enabled() || !inv->is_online())
      continue;
    if (inv->get_power_percent() >= inv->get_max_power_rate())
      continue;
    if (!inv->can_produce_more())
      continue;

    // Raising output raises voltage, so an inverter on a phase already near
    // the limit must not be pushed further. Its own terminals count as well as
    // the meter's view, and against the threshold in its own convention.
    bool blocked = false;
    float own_p = inv->peak_ac_voltage();
    float own_l = inv->peak_line_voltage();
    float own_p_limit = inv->ac_voltage_is_line() ? line_limit : limit;
    if (!std::isnan(own_p) && own_p > own_p_limit) {
      blocked = true;
      ESP_LOGD(TAG, "  slot %u: blocked, own terminals at %.1f V (limit %.1f)",
               (unsigned) i, own_p, own_p_limit);
    }
    if (!std::isnan(own_l) && own_l > line_limit) {
      blocked = true;
      ESP_LOGD(TAG, "  slot %u: blocked, own line voltage %.1f V", (unsigned) i,
               own_l);
    }
    if (inv->get_phases() >= 3) {
      for (uint8_t q = 0; q < 3; q++)
        if (!std::isnan(volt[q]) && volt[q] > limit)
          blocked = true;
    } else {
      uint8_t p = inv->get_phase();
      if (p < 3 && !std::isnan(volt[p]) && volt[p] > limit)
        blocked = true;
    }
    if (blocked) {
      ESP_LOGD(TAG, "  slot %u: blocked, grid voltage above %.1f V",
               (unsigned) i, limit);
      continue;
    }

    float power = this->headroom_up_(inv, err);
    if (power <= this->import_threshold_)
      continue;
    // Strictly greater keeps declaration order as the tie breaker.
    if (best == nullptr || power > best_power) {
      best = inv;
      best_i = i;
      best_power = power;
    }
  }

  if (best != nullptr) {
    float step = this->step_for_(best, best_power, this->increase_gain_);
    // Close to the limit, creep. The proportional step is sized from a power
    // error, which says nothing about how much voltage headroom is left, and a
    // 20 % jump here simply lands back above the limit next cycle.
    float near_v = NAN;
    float near_limit = limit;
    if (best->get_phases() >= 3) {
      for (uint8_t q = 0; q < 3; q++)
        if (!std::isnan(volt[q]) && (std::isnan(near_v) || volt[q] > near_v))
          near_v = volt[q];
    } else {
      uint8_t p = best->get_phase();
      if (p < 3)
        near_v = volt[p];
    }
    // Compare like with like by measuring each source's distance to its own
    // limit, then keep whichever is closest.
    const float own_vs[2] = {best->peak_ac_voltage(), best->peak_line_voltage()};
    const float own_ls[2] = {best->ac_voltage_is_line() ? line_limit : limit,
                             line_limit};
    for (uint8_t k = 0; k < 2; k++) {
      if (std::isnan(own_vs[k]))
        continue;
      if (std::isnan(near_v) ||
          (own_ls[k] - own_vs[k]) < (near_limit - near_v)) {
        near_v = own_vs[k];
        near_limit = own_ls[k];
      }
    }
    if (!std::isnan(near_v) && near_v > near_limit - this->voltage_soft_margin_ &&
        step > this->min_step_) {
      ESP_LOGD(TAG, "  step capped: %.1f V is within %.1f V of the %.1f V limit",
               near_v, this->voltage_soft_margin_, near_limit);
      step = this->min_step_;
    }
    float from = best->get_power_percent();
    best->apply_power_rate(from + step);
    ESP_LOGI(TAG, "%.0f W coverable by slot %u (%s) -> up %.1f%%, %.0f%% to %u%%",
             best_power, (unsigned) best_i,
             best->get_phases() >= 3 ? "three phase" : "single phase", step,
             from, best->get_power_percent());
    this->set_ctrl_state_("raising to cover import");
    return;
  }

  // ---- last resort: rebalance the phases ----
  // Nothing could be raised, which normally means a three phase unit is pinned
  // by whichever phase has the least headroom. Single phase inverters loading
  // the quieter phases are what keeps that headroom small, so giving up some of
  // their output buys three times as much from a three phase unit. Every phase
  // meaningfully below the busiest one is treated, because two lightly loaded
  // phases block just as effectively as one.
  //
  // This runs after the increase pass on purpose: sacrificing production is
  // only justified once there is no ordinary way left to cover the import.
  if (this->rebalance_) {
    float busiest = NAN;
    for (uint8_t p = 0; p < 3; p++) {
      if (std::isnan(err[p]))
        continue;
      if (std::isnan(busiest) || err[p] > busiest)
        busiest = err[p];
    }

    bool taker = false;
    for (auto *inv : this->inverters_) {
      if (inv->is_enabled() && inv->is_online() && inv->get_phases() >= 3 &&
          inv->get_power_percent() < inv->get_max_power_rate() &&
          inv->can_produce_more()) {
        taker = true;
        break;
      }
    }

    if (!std::isnan(busiest) && busiest > this->import_threshold_ && taker) {
      bool traded = false;
      for (uint8_t p = 0; p < 3; p++) {
        if (std::isnan(err[p]))
          continue;
        float deficit = busiest - err[p];
        if (deficit <= this->rebalance_threshold_)
          continue;

        for (int i = (int) this->inverters_.size() - 1; i >= 0; i--) {
          GrowattInverter *inv = this->inverters_[i];
          if (!inv->is_enabled() || !inv->is_online())
            continue;
          if (inv->get_phases() >= 3)
            continue;
          if (inv->get_phase() != p)
            continue;
          if (inv->get_power_percent() <= inv->get_min_power_rate())
            continue;
          float out = inv->get_grid_power();
          if (std::isnan(out) || out < MIN_INJECTING_W)
            continue;

          float step = this->step_for_(inv, deficit, this->decrease_gain_);
          float from = inv->get_power_percent();
          inv->apply_power_rate(from - step);
          ESP_LOGI(TAG,
                   "L%u sits %.0f W below the busiest phase; trading slot %u "
                   "down %.1f%% (%.0f%% to %u%%) to free three phase headroom",
                   p + 1, deficit, (unsigned) i, step, from,
                   inv->get_power_percent());
          traded = true;
          break;
        }
      }
      if (traded) {
        this->set_ctrl_state_("rebalancing phases");
        return;
      }
    }
  }

  this->set_ctrl_state_("balanced");
}

void GrowattHub::update() {
  this->update_meter_health_();
  this->update_aggregates_();
  this->update_conditions_();

  // Push the widened trip thresholds down to every inverter. They apply them
  // once they know whether they speak in phase or line voltages.
  float m = 1.0f + this->protection_margin_ / 100.0f;
  for (auto *inv : this->inverters_) {
    inv->set_protection_targets(this->values_[HUB_PHASE_V_LOW] / m,
                                this->values_[HUB_PHASE_V_HIGH] * m,
                                this->values_[HUB_LINE_V_LOW] / m,
                                this->values_[HUB_LINE_V_HIGH] * m,
                                this->restart_delay_s_);
  }

  // Everything starts at the configured startup rate, so a reboot never leaves
  // the inverters running unsupervised at whatever they had before.
  if (!this->started_) {
    this->started_ = true;
    this->last_step_ = millis();
    this->last_refresh_ = millis();
    this->set_all_(this->startup_rate_, "starting up");
    return;
  }
  this->control_power_();
}

void GrowattHub::dump_config() {
  ESP_LOGCONFIG(TAG, "Growatt Master: %u inverter slot(s) of %u, %u meter(s)",
                (unsigned) this->inverters_.size(), this->max_inverters_,
                (unsigned) this->meters_.size());
  ESP_LOGCONFIG(TAG, "  grid phase voltage: %.1f .. %.1f V",
                this->values_[HUB_PHASE_V_LOW],
                this->values_[HUB_PHASE_V_HIGH]);
  ESP_LOGCONFIG(TAG, "  grid line voltage: %.1f .. %.1f V",
                this->values_[HUB_LINE_V_LOW], this->values_[HUB_LINE_V_HIGH]);
  ESP_LOGCONFIG(TAG, "  inverter trip margin: %.0f %%, restart delay %u s",
                this->protection_margin_, this->restart_delay_s_);
  ESP_LOGCONFIG(TAG, "  UPS load max: %.0f %% (avg %.0f %%)",
                this->values_[HUB_UPS_MAX_LOAD],
                this->values_[HUB_UPS_MAX_LOAD_AVG]);
  ESP_LOGCONFIG(TAG, "  battery SOC: %.0f .. %.0f %%",
                this->values_[HUB_BATTERY_SOC_MIN],
                this->values_[HUB_BATTERY_SOC_MAX]);
  ESP_LOGCONFIG(TAG, "  meter stalled after %u ms, offline after %u ms",
                (unsigned) this->stalled_ms_, (unsigned) this->offline_ms_);
  // The controller only acts on a hub update, so the interval it really runs
  // at is step_interval rounded up to the next multiple of update_interval.
  ESP_LOGCONFIG(TAG,
                "  polled every %u ms, control step %u ms (effective %u ms), "
                "refresh %u ms",
                (unsigned) this->get_update_interval(),
                (unsigned) this->step_interval_,
                (unsigned) (this->get_update_interval() == 0
                                ? this->step_interval_
                                : ((this->step_interval_ +
                                    this->get_update_interval() - 1) /
                                   this->get_update_interval()) *
                                      this->get_update_interval()),
                (unsigned) this->refresh_interval_);
  ESP_LOGCONFIG(TAG, "  gain %.2f up / %.2f down, step %.1f..%.1f %%",
                this->increase_gain_, this->decrease_gain_, this->min_step_,
                this->max_step_);
}

}  // namespace growatt_master
}  // namespace esphome
