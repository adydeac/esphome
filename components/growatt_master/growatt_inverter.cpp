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
// Not a response timeout - the hub's send_wait_time is the response timeout, and
// a second, shorter one here is what this replaces. Only a terminal callback
// that never arrives at all gets this far, so the value only has to be above any
// send_wait_time anyone would configure.
static const uint32_t STALL_BACKSTOP_MS = 15000;
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

// Registers arrive in host byte order, so the wire-order assembly these helpers
// used to do is already done. The call sites were always register indexed, which
// is why only the helpers change shape and not the hundreds of uses.
static inline uint16_t reg16(std::span<const uint16_t> d, size_t reg) {
  return d[reg];
}

static inline uint32_t reg32(std::span<const uint16_t> d, size_t reg) {
  return (((uint32_t) reg16(d, reg)) << 16) | reg16(d, reg + 1);
}

// Almost every 32-bit quantity in this protocol is a magnitude: each direction
// gets its own register pair - power to user and power to grid, charge and
// discharge - so none of them ever has to go below zero. Total AC power is the
// exception. It is real power at the grid connection, and it goes negative
// whenever the unit draws from the grid instead of feeding it, which is what a
// PV inverter does all night.
static inline int32_t sreg32(std::span<const uint16_t> d, size_t reg) {
  return (int32_t) reg32(d, reg);
}

static inline void pub2s(sensor::Sensor *s, std::span<const uint16_t> d,
                         size_t reg, float unit) {
  if (s != nullptr)
    s->publish_state(sreg32(d, reg) * unit);
}

static inline void pub1(sensor::Sensor *s, std::span<const uint16_t> d,
                        size_t reg, float unit) {
  if (s != nullptr)
    s->publish_state(reg16(d, reg) * unit);
}

static inline void pub2(sensor::Sensor *s, std::span<const uint16_t> d,
                        size_t reg, float unit) {
  if (s != nullptr)
    s->publish_state(reg32(d, reg) * unit);
}

static inline void pub_val(sensor::Sensor *s, float v) {
  if (s != nullptr)
    s->publish_state(v);
}

// Extract printable ASCII from a register range, trailing spaces removed.
// Each register carries two characters, the first in the high byte - the wire
// order, which survives the conversion to host order as the word's own layout.
static std::string ascii_from(std::span<const uint16_t> d, size_t reg,
                              size_t count) {
  std::string s;
  for (size_t i = 0; i < count; i++) {
    const uint16_t w = d[reg + i];
    const char pair[2] = {(char) (w >> 8), (char) (w & 0xFF)};
    for (char c : pair) {
      if (c >= 32 && c <= 126)
        s += c;
    }
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
// The four storage settings the TL-XH keeps in its own block. Everything else
// in SETTING_ADDR is in the first holding group and is the same address on
// every family. Zero means "this family does not have it".
static const uint16_t XH_SETTING_ADDR[SET_COUNT] = {
    REG_ACTIVE_POWER_RATE, XH_GF_DISCHARGE_RATE, XH_GF_STOP_SOC,
    XH_BF_CHARGE_RATE,     XH_BF_STOP_SOC,       HO_PV_START_VOLT,
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

// A window block per mode, three registers to a period. Zero for a mode this
// family does not have, which is not a case today - all three blocks exist on
// an SPH - but is one the moment a family without load first appears, and the
// callers have to be able to say so either way.
uint16_t sph_window_base(uint8_t mode) {
  switch (mode) {
    case MODE_GRID_FIRST: return HO_GF_WINDOW_BASE;
    case MODE_BATTERY_FIRST: return HO_BF_WINDOW_BASE;
    case MODE_LOAD_FIRST: return HO_LF_WINDOW_BASE;
    default: return 0;
  }
}

// Declaration order is the convention, not the hardware's: any of the nine may
// carry any priority.
static const uint16_t XH_WINDOWS[MODE_COUNT * PERIOD_COUNT] = {
    3038, 3040, 3042,   // periods 1..3
    3044, 3050, 3052,   // periods 4..6
    3054, 3056, 3058,   // periods 7..9
};

uint16_t xh_window_base(uint8_t mode, uint8_t period) {
  if (mode >= MODE_COUNT || period >= PERIOD_COUNT)
    return 0;
  return XH_WINDOWS[mode * PERIOD_COUNT + period];
}

// Our mode numbering and the register's are not the same, and neither is worth
// changing to match: ours is the order the periods appear in, the register's is
// Growatt's.
uint8_t xh_priority_for_mode(uint8_t mode) {
  switch (mode) {
    case MODE_GRID_FIRST: return XH_PRIO_GRID_FIRST;
    case MODE_BATTERY_FIRST: return XH_PRIO_BATTERY_FIRST;
    default: return XH_PRIO_LOAD_FIRST;
  }
}

const char *xh_priority_text(uint8_t prio) {
  switch (prio) {
    case XH_PRIO_LOAD_FIRST: return "load first";
    case XH_PRIO_BATTERY_FIRST: return "battery first";
    case XH_PRIO_GRID_FIRST: return "grid first";
    default: return "unknown";
  }
}

const char *window_mode_text(uint8_t mode) {
  switch (mode) {
    case MODE_GRID_FIRST: return "grid first";
    case MODE_BATTERY_FIRST: return "battery first";
    case MODE_LOAD_FIRST: return "load first";
    default: return "unknown";
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

void GrowattInverter::publish_control_summary(const char *s) {
  if (this->control_ts_ == nullptr || this->control_ts_->state == s)
    return;
  this->control_ts_->publish_state(s);
}

void GrowattInverter::update_health_() {
  uint32_t now = millis();
  uint8_t h;
  if (this->address_ == 0) {
    // An unpointed slot. Address zero is the Modbus broadcast address, so a
    // read issued here would be answered by nobody and heard by everybody;
    // there is nothing to wait a timeout for. Declaring it offline at once
    // keeps it off the bus and off the dispatch.
    h = INV_OFFLINE;
  } else if (!this->ever_asked_) {
    // Never asked anything yet, so there is nothing to conclude. try_send_()
    // only queues a frame once the bus will take one, and a modbus_tcp hub will
    // not take one until its socket is up - so this state covers the whole
    // period in which the transport is still finding its way to the inverter.
    // A slot cannot be blamed for silence it was never given a chance to break.
    h = INV_ONLINE;
  } else if (this->last_update_ == 0) {
    // Asked, never answered. The window is measured from the first request that
    // actually left this node rather than from boot, because boot is much
    // earlier than that: WiFi association, DHCP, the transport's connect - which
    // a modbus_tcp hub retries no sooner than reconnect_interval - and the other
    // slots taking their turn all land in between. Measured from uptime, a slot
    // is written off before it has been asked twice, and then serves a full
    // offline_probe_interval of enforced silence for a fault that was never its
    // own. Its own window too, not offline_ms_: coming up cold is a different
    // question from having gone quiet, it is slower, and it happens once.
    h = (now - this->first_send_ms_ > this->startup_grace_ms_) ? INV_OFFLINE
                                                               : INV_ONLINE;
  } else {
    uint32_t age = (micros() - this->last_update_) / 1000;
    if (age < this->stalled_ms_)
      h = INV_ONLINE;
    else if (age < this->offline_ms_)
      h = INV_STALLED;
    else
      h = INV_OFFLINE;
  }
  if (h == this->health_ && this->health_published_)
    return;

  uint8_t was = this->health_;
  this->health_ = h;
  this->health_published_ = true;
  if (this->state_ts_ != nullptr)
    this->state_ts_->publish_state(this->health_text());

  if (h == INV_OFFLINE) {
    ESP_LOGW(TAG, "slot %u went offline, backing off to a probe every %u s",
             this->slot_index_, (unsigned) (this->offline_probe_ms_ / 1000));
    this->zero_instantaneous_();
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

// Every flow figure this slot publishes, driven to zero once the unit is
// declared offline. A power reading left frozen at its last value is worse than
// no reading at all: it keeps a dead unit contributing to graphs and to any
// template that sums across the plant, and it does so silently.
//
// What is deliberately absent is as much of the design as what is here.
// Voltages, frequency, temperatures and state of charge stay on their last
// reading - zero there would assert something this node cannot know, and a
// grid voltage of 0 V reads as a blackout rather than as a lost slot. The
// energy counters stay too: they are cumulative, and a counter that dips to
// zero and climbs back destroys the long term statistics in Home Assistant.
// Those all go stale honestly and the health entity is what says so.
void GrowattInverter::zero_instantaneous_() {
  for (uint8_t i = 0; i < 3; i++) {
    pub_val(this->phases_[i].current, 0.0f);
    pub_val(this->phases_[i].power, 0.0f);
    pub_val(this->ups_[i].current, 0.0f);
    pub_val(this->ups_[i].power, 0.0f);
  }
  for (uint8_t i = 0; i < MAX_STRINGS; i++) {
    pub_val(this->pvs_[i].current, 0.0f);
    pub_val(this->pvs_[i].power, 0.0f);
  }
  pub_val(this->pv_active_power_, 0.0f);
  pub_val(this->grid_active_power_, 0.0f);
  pub_val(this->ac_charge_power_, 0.0f);
  pub_val(this->bat_charge_power_, 0.0f);
  pub_val(this->bat_discharge_power_, 0.0f);
  pub_val(this->bms_current_, 0.0f);
  pub_val(this->power_to_user_, 0.0f);
  pub_val(this->power_to_grid_, 0.0f);
  pub_val(this->local_load_power_, 0.0f);
  pub_val(this->ups_total_power_, 0.0f);
  pub_val(this->ups_load_, 0.0f);
  pub_val(this->ups_load_avg_, 0.0f);
}

// A refused request produces no callback at all, so starting the wait would
// stall this slot for the full timeout with nothing coming. Every send path
// reports its outcome here. The recovery deliberately mirrors loop()'s timeout
// handling case for case, including the order - a write is only abandoned after
// the retry budget, exactly as a timed-out one is - so a refusal and a silence
// leave the state machine in the same place.
bool GrowattInverter::queued_(bool ok) {
  if (ok) {
    this->last_send_ = millis();
    // The moment this slot was first actually asked something, which is what
    // update_health_() measures its startup window from. Only a frame the bus
    // accepted counts: a refusal means the question was never put.
    if (!this->ever_asked_) {
      this->ever_asked_ = true;
      this->first_send_ms_ = this->last_send_;
    }
    this->waiting_ = true;
    return true;
  }
  this->waiting_ = false;
  this->retries_++;
  this->bus_release_ = millis();

  if (this->probing_) {
    this->probing_ = false;
    this->retries_ = 0;
    return false;
  }

  if (this->retries_ <= IDENT_MAX_RETRIES) {
    ESP_LOGD(TAG, "slot %u: bus refused the request, retrying (%u/%u)",
             this->slot_index_, this->retries_, IDENT_MAX_RETRIES);
    this->want_send_ = true;
    return false;
  }

  ESP_LOGW(TAG, "slot %u: bus kept refusing the request, abandoning it",
           this->slot_index_);

  if (this->writing_) {
    this->writing_ = false;
    this->write_head_ = (this->write_head_ + 1) % WRITE_QUEUE_SIZE;
    this->write_count_--;
    this->retries_ = 0;
    this->want_send_ = this->write_count_ > 0;
    return false;
  }
  if (this->dump_active_) {
    this->dump_skip_range_();
    return false;
  }
  if (this->poll_ != POLL_IDLE) {
    this->advance_poll_();
    return false;
  }
  this->advance_(false);
  return false;
}

void GrowattInverter::send_probe_() {
  if (!this->queued_(this->read_input_registers(PROBE_BASE, PROBE_CNT)))
    return;
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

// The hub is the one that decides a request has gone unanswered, and it says so
// through on_no_response(). This is that decision arriving; the recovery below
// is unchanged, only its trigger moved.
bool GrowattInverter::on_no_response(std::span<const uint8_t> request_pdu) {
  if (this->waiting_)
    this->no_answer_("no response");
  // The hub would re-queue the same frame for us. Declining keeps the choice
  // here, where the retry budget, the write queue and the identification step
  // all live, and where a retry has always been decided.
  return false;
}

// Accepted into the machine and then discarded before it reached the wire -
// a queue cleared out from under it. Same hole in the state machine as a
// silence, so the same recovery: this is a terminal callback like any other,
// and nothing else is coming for that request.
void GrowattInverter::on_not_sent(std::span<const uint8_t> request_pdu) {
  if (this->waiting_)
    this->no_answer_("never sent");
}

void GrowattInverter::loop() {
  if (this->want_send_ && !this->waiting_)
    this->try_send_();

  if (!this->waiting_)
    return;

  // Not the timeout - the hub owns that. This is a watchdog of last resort for
  // a terminal callback that never arrived at all, which would otherwise leave
  // this slot waiting forever with no timer left to rescue it. It is set well
  // above any send_wait_time worth configuring, so reaching it means the hub
  // broke its own contract: log it as the fault it is rather than papering over
  // it at DEBUG among the ordinary retries.
  if (millis() - this->last_send_ < STALL_BACKSTOP_MS)
    return;
  ESP_LOGE(TAG, "slot %u: no terminal callback %u ms after send, recovering",
           this->slot_index_, (unsigned) STALL_BACKSTOP_MS);
  this->no_answer_("lost callback");
}

// Everything that follows a request resolving with nothing usable in it. Split
// out of loop() when the hub's callback became the trigger, so that a silence,
// a discarded frame and the backstop all leave the state machine in the same
// place - which is the property the whole retry path depends on.
void GrowattInverter::no_answer_(const char *why) {
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
    ESP_LOGD(TAG, "slot %u: %s (step %u, poll %u), retrying (%u/%u)",
             this->slot_index_, why, this->step_, this->poll_, this->retries_,
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
  bool ok;
  switch (this->step_) {
    case IDENT_START:
      this->step_ = IDENT_LIVE;
      // fallthrough
    case IDENT_LIVE:
      ok = this->read_input_registers(IN_BASE, FIRST_GROUP_CNT);
      break;
    case IDENT_INFO:
      ok = this->read_holding_registers(HOLD_BASE, FIRST_GROUP_CNT);
      break;
    case IDENT_TYPE:
      ok = this->read_holding_registers(REG_TYPE_BASE, REG_TYPE_CNT);
      break;
    case IDENT_CAPS:
      ok = this->read_holding_registers(REG_PVSTRSCAN, REG_CAPS_CNT);
      break;
    case IDENT_STORAGE:
      ok = this->read_holding_registers(REG_STORAGE_BASE, REG_STORAGE_CNT);
      break;
    case IDENT_BDC:
      ok = this->read_input_registers(REG_BDC_STATE, REG_BDC_STATE_CNT);
      break;
    case IDENT_BATTERY:
      if (this->caps_.storage_family == STORAGE_TLXH)
        ok = this->read_input_registers(XH_BAT_BASE, XH_BAT_CNT);
      else
        ok = this->read_input_registers(REG_BAT_BASE, REG_BAT_CNT);
      break;
    case IDENT_SETTINGS:
      ok = this->caps_.storage_family == STORAGE_TLXH
               ? this->read_holding_registers(XH_SETTINGS_BASE, XH_SETTINGS_CNT)
               : this->read_holding_registers(HO_SETTINGS_BASE,
                                              HO_SETTINGS_CNT);
      break;
    default:
      return;
  }
  if (!this->queued_(ok))
    return;
  ESP_LOGV(TAG, "slot %u: sent step %u", this->slot_index_, this->step_);
}

void GrowattInverter::send_poll_() {
  bool ok;
  switch (this->poll_) {
    case POLL_FAST_MAIN:
      ok = this->read_input_registers(POLL_FAST_MAIN_BASE, POLL_FAST_MAIN_CNT);
      break;
    case POLL_FAST_STATUS:
      ok = this->read_input_registers(POLL_FAST_STATUS_BASE, POLL_FAST_STATUS_CNT);
      break;
    case POLL_FAST_BAT:
      ok = this->caps_.storage_family == STORAGE_TLXH
               ? this->read_input_registers(XH_FAST_BAT_BASE, XH_FAST_BAT_CNT)
               : this->read_input_registers(POLL_FAST_BAT_BASE,
                                            POLL_FAST_BAT_CNT);
      break;
    case POLL_FAST_UPS:
      ok = this->read_input_registers(POLL_FAST_UPS_BASE, POLL_FAST_UPS_CNT);
      break;
    case POLL_SLOW_MAIN:
      ok = this->read_input_registers(POLL_SLOW_MAIN_BASE, POLL_SLOW_MAIN_CNT);
      break;
    case POLL_SLOW_STOR:
      ok = this->caps_.storage_family == STORAGE_TLXH
               ? this->read_input_registers(XH_SLOW_STOR_BASE,
                                            XH_SLOW_STOR_CNT)
               : this->read_input_registers(POLL_SLOW_STOR_BASE,
                                            POLL_SLOW_STOR_CNT);
      break;
    default:
      return;
  }
  if (!this->queued_(ok))
    return;
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
    case IDENT_STORAGE:
      // Only ask about a BDC when the 1000 block turned up nothing. A unit
      // that answered there is an SPH and has no 3000 storage block to probe.
      this->step_ = this->caps_.has_storage ? IDENT_BATTERY : IDENT_BDC;
      break;
    case IDENT_BDC:     this->step_ = IDENT_BATTERY; break;
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
      // A TL-XH has no EPS terminal, so the block after this one is a read
      // that can only ever return zeros on a bus that is the constraint.
      this->poll_ = this->caps_.has_ups_block()
                        ? POLL_FAST_UPS
                        : (this->slow_due_ ? POLL_SLOW_MAIN : POLL_IDLE);
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

void GrowattInverter::detect_from_live_(std::span<const uint16_t> data) {
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

void GrowattInverter::parse_device_info_(std::span<const uint16_t> data) {
  this->caps_.dtc = reg16(data, HO_DTC);
  this->caps_.serial = ascii_from(data, HO_SERIAL, HO_SERIAL_CNT);

  // Nameplate power is documented as 0.1 VA and reads that way on MID and MIN
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
    // The flat table on purpose, not setting_addr_(): this runs before the
    // storage family is known, and the first holding group is the one place
    // where both tables hold the same address anyway.
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
  // and the phase count in the low byte. It is accurate on MID and MIN units
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
void GrowattInverter::parse_fast_main_(std::span<const uint16_t> data) {
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

  pub2s(this->grid_active_power_, data, IN_AC_POWER, ONE_DEC);
  this->grid_power_w_ = sreg32(data, IN_AC_POWER) * ONE_DEC;
  this->revise_nameplate_();
  this->update_capability_();
  pub1(this->frequency_, data, IN_FREQUENCY, TWO_DEC);

  float iac[3], pac[3];
  uint8_t i_present = 0, p_present = 0;
  for (uint8_t i = 0; i < 3; i++) {
    size_t v = IN_VAC[i];
    this->ac_voltage_[i] = reg16(data, v) * ONE_DEC;
    iac[i] = reg16(data, v + 1) * ONE_DEC;
    // Deliberately unsigned, unlike the total above: the per phase registers
    // carry Vac x Iac, a magnitude, and stay positive while the total is
    // negative. Measured at night on a MIN: total -31.0 W, Pac1 +48.5 W with
    // Vac1 228.7 V and Iac1 0.2 A.
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
  // Detected rather than keyed off a model, because a MID 40K populates all
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
void GrowattInverter::parse_fast_status_(std::span<const uint16_t> data) {
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
void GrowattInverter::parse_slow_main_(std::span<const uint16_t> data) {
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
void GrowattInverter::parse_fast_bat_(std::span<const uint16_t> data) {
  pub2(this->bat_discharge_power_, data, 0, ONE_DEC);  // 1009
  pub2(this->bat_charge_power_, data, 2, ONE_DEC);     // 1011
  this->battery_voltage_v_ = reg16(data, 4) * ONE_DEC;  // 1013
  pub_val(this->battery_voltage_, this->battery_voltage_v_);
  this->battery_soc_pct_ = reg16(data, 5);              // 1014
  pub_val(this->battery_soc_sens_, this->battery_soc_pct_);
}

// Fast BDC block, input 3167..3181. Same entities as the SPH block above, with
// the pack voltage at 0.01 V rather than 0.1 V and the charge and discharge
// power the other way round in the register order.
void GrowattInverter::parse_fast_bat_xh_(std::span<const uint16_t> data) {
  pub2(this->bat_discharge_power_, data, XB_P_DISCHARGE, ONE_DEC);
  pub2(this->bat_charge_power_, data, XB_P_CHARGE, ONE_DEC);
  this->battery_voltage_v_ = reg16(data, XB_VBAT) * TWO_DEC;
  pub_val(this->battery_voltage_, this->battery_voltage_v_);
  this->battery_soc_pct_ = reg16(data, XB_SOC);
  pub_val(this->battery_soc_sens_, this->battery_soc_pct_);
  // On an SPH the pack temperature arrives with the slow block; here it is in
  // the fast one, so the same entity simply updates more often.
  pub1(this->battery_temperature_, data, XB_TEMP_A, ONE_DEC);
  pub1(this->fault_word_, data, XB_FAULT, 1.0f);
}

// Fast UPS block, input 1067..1081. Also feeds the load average window.
void GrowattInverter::parse_fast_ups_(std::span<const uint16_t> data) {
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

void GrowattInverter::parse_storage_(std::span<const uint16_t> data) {
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

// Slow BDC/BMS block, input 3125..3231. Only the values the SPH slow block also
// publishes are mapped, so the entity set is the same on both families. The
// registers this family has and the other does not - the EPS totals, the BMS
// cell extremes, the derate reason - are deliberately left for their own
// change rather than smuggled in under a battery patch.
void GrowattInverter::parse_storage_xh_(std::span<const uint16_t> data) {
  pub2(this->discharge_energy_today_, data, XS_E_DISCHARGE_TODAY, ONE_DEC);
  pub2(this->discharge_energy_total_, data, XS_E_DISCHARGE_TOTAL, ONE_DEC);
  pub2(this->charge_energy_today_, data, XS_E_CHARGE_TODAY, ONE_DEC);
  pub2(this->charge_energy_total_, data, XS_E_CHARGE_TOTAL, ONE_DEC);

  pub1(this->bms_soc_, data, XS_BMS_SOC, 1.0f);
  pub1(this->bms_voltage_, data, XS_BMS_VOLT, TWO_DEC);
  pub1(this->bms_current_, data, XS_BMS_CURR, TWO_DEC);
  pub1(this->bms_temperature_, data, XS_BMS_TEMP, ONE_DEC);
  pub1(this->battery_cycles_, data, XS_BAT_CYCLES, 1.0f);
  pub1(this->battery_health_, data, XS_BAT_HEALTH, 1.0f);
}

// Values that are computed rather than read. Kept in the component so the
// YAML side only has to declare the sensor it wants to see.
void GrowattInverter::publish_derived_() {
  if (!this->caps_.has_storage)
    return;

  if (this->ups_total_power_ != nullptr && this->caps_.has_ups_block()) {
    float sum = this->ups_phase_power_[0] + this->ups_phase_power_[1] +
                this->ups_phase_power_[2];
    this->ups_total_power_->publish_state(sum);
  }

  if (this->ups_load_avg_ != nullptr && this->caps_.has_ups_block() &&
      this->ups_avg_count_ > 0)
    this->ups_load_avg_->publish_state(this->ups_load_avg_pct_);

  // Module count from pack voltage. module_voltage_ is configurable because it
  // depends on the battery model, not on the inverter.
  float modules = 0;
  if (this->module_voltage_ > 0 && this->battery_voltage_v_ > 0)
    modules = roundf(this->battery_voltage_v_ / this->module_voltage_);
  pub_val(this->battery_modules_, modules);

  // Maximum sustainable discharge expressed as a percentage of the inverter
  // rating: usable pack energy divided by the discharge window.
  if (this->ups_max_power_ != nullptr && this->caps_.has_ups_block() &&
      this->normal_power_va_ > 0 && this->discharge_hours_ > 0) {
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
  // Set before queueing, not after: on a refusal queued_() has to know a write
  // is what it is unwinding, so it drops the right queue entry.
  this->writing_ = true;
  bool ok;
  if (w.function == CMD_WRITE_MULTI) {
    // The helper takes host-order words and does the big-endian framing itself,
    // which is what the hand-built payload byte pairs used to do here.
    ok = this->write_multiple_registers(
        w.address, std::span<const uint16_t>(w.values, w.count));
  } else {
    ok = this->write_single_register(w.address, w.values[0]);
  }
  if (!this->queued_(ok))
    return;
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
  // Recorded here rather than at each call site so that every path that moves
  // the rate - controller, safety cut, operator - leaves the same trace, and
  // the watchdog rewrite of an unchanged value leaves none.
  if (v != this->power_percent_) {
    this->ctrl_dir_ = v > this->power_percent_ ? 1 : -1;
    this->ctrl_move_ms_ = millis();
  }
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
// within 10 W of their 6000 W nameplate. A MID that was PV limited throughout
// gave 451, 647, 672, 1292 and 1325 W - which is why the ratio test has to gate
// this, or the rolling maximum would keep the worst overestimate.
void GrowattInverter::update_capability_() {
  if (!this->normal_power_valid_ || std::isnan(this->grid_power_w_) ||
      this->power_percent_ == 0)
    return;

  // A reading taken while the unit is still moving to a new setpoint is not
  // evidence about that setpoint. Output lags the command by fifteen to thirty
  // seconds, which is what control_settle_time exists to describe, and this was
  // the one consumer of output that ignored it: a rate cut extrapolates from an
  // output that has not fallen yet, and the ratchet below then keeps that
  // inflated figure for a whole window.
  //
  // The whole function waits, expiry included. Holding a settled estimate
  // through a ramp is exactly right: it was taken when the reading meant
  // something, and the ramp it is being held through is the one it authorised.
  //
  // This is necessary and not sufficient. A settled reading still cannot tell
  // "the setpoint is holding it back" from "the panels are, and the setpoint
  // happens to be just above what they give", because both put output near
  // cap_ratio times the implied limit. Only moving the setpoint and watching
  // whether output follows separates them, and that is a two point test this
  // does not attempt.
  uint32_t now = millis();
  if (this->since_last_move(now) < this->settle_ms_)
    return;

  float limit = this->normal_power_va_ * this->power_percent_ / 100.0f;
  this->rate_binding_ = limit > 0 && this->grid_power_w_ >= this->cap_ratio_ * limit;

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

uint16_t GrowattInverter::setting_addr_(uint8_t field) const {
  if (field >= SET_COUNT)
    return 0;
  return this->caps_.storage_family == STORAGE_TLXH ? XH_SETTING_ADDR[field]
                                                    : SETTING_ADDR[field];
}

uint16_t GrowattInverter::ac_charge_addr_() const {
  return this->caps_.storage_family == STORAGE_TLXH ? XH_BF_AC_CHARGE
                                                    : HO_BF_AC_CHARGE;
}

void GrowattInverter::set_setting(uint8_t field, float value) {
  if (field >= SET_COUNT)
    return;
  uint16_t addr = this->setting_addr_(field);
  if (addr == 0)
    return;
  uint16_t raw = (uint16_t) lroundf(value / SETTING_SCALE[field]);
  this->settings_[field] = raw;
  this->queue_write_(CMD_WRITE_SINGLE, addr, &raw, 1);
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

void GrowattInverter::publish_reg_entities_(std::span<const uint16_t> data,
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
  this->queue_write_(CMD_WRITE_SINGLE, this->ac_charge_addr_(), &v, 1);
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

// The firmware rejects an enabled period that overlaps an enabled period of a
// different mode. Catching it here gives a readable message instead of a bare
// Modbus exception. Two periods of the same mode overlapping is the operator's
// business, not ours: the result is one longer period.
bool GrowattInverter::windows_overlap(std::string *reason) const {
  for (uint8_t ma = 0; ma < MODE_COUNT; ma++) {
    for (uint8_t pa = 0; pa < PERIOD_COUNT; pa++) {
      const TimeWindow &a = this->windows_[ma][pa];
      if (!a.enabled)
        continue;
      for (uint8_t mb = ma + 1; mb < MODE_COUNT; mb++) {
        for (uint8_t pb = 0; pb < PERIOD_COUNT; pb++) {
          const TimeWindow &b = this->windows_[mb][pb];
          if (!b.enabled || !window_pair_overlaps(a, b))
            continue;
          if (reason != nullptr) {
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "%s period %u (%02u:%02u-%02u:%02u) overlaps "
                     "%s period %u (%02u:%02u-%02u:%02u)",
                     window_mode_text(ma), pa + 1, a.start_h, a.start_m,
                     a.stop_h, a.stop_m, window_mode_text(mb), pb + 1,
                     b.start_h, b.start_m, b.stop_h, b.stop_m);
            *reason = buf;
          }
          return true;
        }
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

  if (this->caps_.storage_family == STORAGE_TLXH)
    return this->apply_windows_xh_(mode);

  uint16_t regs[WINDOW_REGS];
  for (uint8_t p = 0; p < PERIOD_COUNT; p++) {
    const TimeWindow &w = this->windows_[mode][p];
    regs[p * 3 + 0] = ((uint16_t) w.start_h << 8) | w.start_m;
    regs[p * 3 + 1] = ((uint16_t) w.stop_h << 8) | w.stop_m;
    regs[p * 3 + 2] = w.enabled ? 1 : 0;
  }
  uint16_t base = sph_window_base(mode);
  if (base == 0) {
    ESP_LOGW(TAG, "slot %u: no %s window block on this model", this->slot_index_,
             window_mode_text(mode));
    return false;
  }
  ESP_LOGI(TAG, "slot %u: applying %s windows to %u", this->slot_index_,
           window_mode_text(mode), base);
  return this->queue_write_(CMD_WRITE_MULTI, base, regs, WINDOW_REGS);
}

// Three periods of one mode, three separate writes: the nine windows are not
// contiguous - 3038..3045, then 3050..3059 with the battery first parameters
// between them - so there is no block to send in one frame the way the SPH
// blocks are sent. Two registers each, which fits the queue three times over.
//
// Each pair goes as one 0x10 rather than two 0x06. Start and stop have to
// change together: sending them separately leaves the window as a new start
// against an old stop for as long as the second frame takes, and a schedule
// nobody asked for is a schedule the inverter will act on.
bool GrowattInverter::apply_windows_xh_(uint8_t mode) {
  uint8_t prio = xh_priority_for_mode(mode);
  bool all = true;
  for (uint8_t p = 0; p < PERIOD_COUNT; p++) {
    TimeWindow &w = this->windows_[mode][p];
    uint16_t base = xh_window_base(mode, p);
    uint16_t regs[2];
    // The flags share the word with the start time, so they are composed here
    // rather than preserved: this is the one moment the convention is allowed
    // to correct what the register says, because the operator is writing this
    // window. Anything else about it is left as it was found.
    regs[0] = (uint16_t) (((w.enabled ? XH_WIN_ENABLED : 0) |
                           ((prio & XH_WIN_PRIORITY_MASK)
                            << XH_WIN_PRIORITY_SHIFT) |
                           ((w.start_h & XH_WIN_HOUR_MASK)
                            << XH_WIN_HOUR_SHIFT) |
                           (w.start_m & XH_WIN_MINUTE_MASK)));
    // Bits 13..15 of the stop word are reserved and go out clear.
    regs[1] = (uint16_t) (((w.stop_h & XH_WIN_HOUR_MASK) << XH_WIN_HOUR_SHIFT) |
                          (w.stop_m & XH_WIN_MINUTE_MASK));
    if (w.priority != prio)
      ESP_LOGI(TAG, "slot %u: window %u was %s, writing it as %s",
               this->slot_index_, mode * PERIOD_COUNT + p + 1,
               xh_priority_text(w.priority), xh_priority_text(prio));
    ESP_LOGI(TAG, "slot %u: %s period %u -> %u: %02u:%02u-%02u:%02u %s",
             this->slot_index_, window_mode_text(mode), p + 1, base, w.start_h,
             w.start_m, w.stop_h, w.stop_m, w.enabled ? "enabled" : "off");
    if (!this->queue_write_(CMD_WRITE_MULTI, base, regs, 2)) {
      all = false;
      continue;
    }
    // Only once the frame is queued: a dropped write leaves the register as it
    // was, and our copy has to say the same or the next read back looks like
    // the inverter changed its mind.
    w.priority = prio;
  }
  return all;
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
void GrowattInverter::parse_settings_(std::span<const uint16_t> data) {
  // offsets relative to 1070
  for (uint8_t f = 0; f < SET_COUNT; f++) {
    uint16_t addr = this->setting_addr_(f);
    if (addr >= HO_SETTINGS_BASE && addr < HO_SETTINGS_BASE + HO_SETTINGS_CNT)
      this->settings_[f] = reg16(data, addr - HO_SETTINGS_BASE);
  }
  this->ac_charge_ = reg16(data, HO_BF_AC_CHARGE - HO_SETTINGS_BASE) != 0;
  this->publish_reg_entities_(data, HO_SETTINGS_BASE, HO_SETTINGS_CNT);

  for (uint8_t m = 0; m < MODE_COUNT; m++) {
    uint16_t base = sph_window_base(m);
    if (base == 0)
      continue;
    size_t off = base - HO_SETTINGS_BASE;
    for (uint8_t p = 0; p < PERIOD_COUNT; p++) {
      uint16_t start = reg16(data, off + p * 3 + 0);
      uint16_t stop = reg16(data, off + p * 3 + 1);
      TimeWindow &w = this->windows_[m][p];
      // The hour is five bits, not eight. It reads the same on this family,
      // whose top three bits are clear, and on a TL-XH, which keeps the enable
      // and the priority up there - so the mask belongs in the shared parser
      // rather than in whichever one is written second.
      w.start_h = (start >> 8) & 0x1F;
      w.start_m = start & 0xFF;
      w.stop_h = (stop >> 8) & 0x1F;
      w.stop_m = stop & 0xFF;
      w.enabled = reg16(data, off + p * 3 + 2) != 0;
      // On this family the block is the priority, so there is nothing to read
      // and nothing that can disagree.
      w.priority = xh_priority_for_mode(m);
    }
  }
  this->publish_settings_();
  ESP_LOGI(TAG, "slot %u: settings read back (GF %u%%/%u%%, BF %u%%/%u%%, AC %s)",
           this->slot_index_, this->settings_[SET_GF_DISCHARGE_RATE],
           this->settings_[SET_GF_STOP_SOC], this->settings_[SET_BF_CHARGE_RATE],
           this->settings_[SET_BF_STOP_SOC], this->ac_charge_ ? "on" : "off");
}

// The same job for the other family: holding 3036..3059. The rates and stop
// SOCs are plain values at different addresses, so they land in settings_ the
// same way. The windows are the part that is genuinely different - nine of
// them, two registers each, with the enable and the priority riding in the
// start word.
void GrowattInverter::parse_settings_xh_(std::span<const uint16_t> data) {
  for (uint8_t f = 0; f < SET_COUNT; f++) {
    uint16_t addr = this->setting_addr_(f);
    if (addr >= XH_SETTINGS_BASE && addr < XH_SETTINGS_BASE + XH_SETTINGS_CNT)
      this->settings_[f] = reg16(data, addr - XH_SETTINGS_BASE);
  }
  this->ac_charge_ = reg16(data, XH_BF_AC_CHARGE - XH_SETTINGS_BASE) != 0;
  this->publish_reg_entities_(data, XH_SETTINGS_BASE, XH_SETTINGS_CNT);

  uint8_t mismatched = 0;
  for (uint8_t m = 0; m < MODE_COUNT; m++) {
    for (uint8_t p = 0; p < PERIOD_COUNT; p++) {
      uint16_t base = xh_window_base(m, p);
      size_t off = base - XH_SETTINGS_BASE;
      uint16_t start = reg16(data, off);
      uint16_t stop = reg16(data, off + 1);
      TimeWindow &w = this->windows_[m][p];
      w.start_h = (start >> XH_WIN_HOUR_SHIFT) & XH_WIN_HOUR_MASK;
      w.start_m = start & XH_WIN_MINUTE_MASK;
      w.stop_h = (stop >> XH_WIN_HOUR_SHIFT) & XH_WIN_HOUR_MASK;
      w.stop_m = stop & XH_WIN_MINUTE_MASK;
      w.enabled = (start & XH_WIN_ENABLED) != 0;
      w.priority = (start >> XH_WIN_PRIORITY_SHIFT) & XH_WIN_PRIORITY_MASK;
      // Which third of the nine a window sits in is our convention; the bits
      // are the inverter's. Where they disagree the register wins, because
      // whatever put it there meant it - ShinePhone can write any priority
      // into any window. Correcting it belongs to the next write of that
      // window, not to boot.
      if (w.enabled && w.priority != xh_priority_for_mode(m)) {
        mismatched++;
        ESP_LOGW(TAG,
                 "slot %u: window %u (%u) is %02u:%02u-%02u:%02u and says %s, "
                 "not the %s this period stands for - left as it is",
                 this->slot_index_, m * PERIOD_COUNT + p + 1, base, w.start_h,
                 w.start_m, w.stop_h, w.stop_m, xh_priority_text(w.priority),
                 window_mode_text(m));
      }
    }
  }

  this->publish_settings_();
  ESP_LOGI(TAG,
           "slot %u: settings read back (GF %u%%/%u%%, BF %u%%/%u%%, AC %s), "
           "%u window(s) not in their conventional priority",
           this->slot_index_, this->settings_[SET_GF_DISCHARGE_RATE],
           this->settings_[SET_GF_STOP_SOC], this->settings_[SET_BF_CHARGE_RATE],
           this->settings_[SET_BF_STOP_SOC], this->ac_charge_ ? "on" : "off",
           mismatched);
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

  const modbus::EntityType table = r.function == CMD_READ_HOLDING
                                       ? modbus::EntityType::HOLDING
                                       : modbus::EntityType::INPUT_REGISTER;
  this->queued_(this->read_entities(table, r.start + this->dump_offset_, n));
}

void GrowattInverter::handle_dump_(std::span<const uint16_t> data) {
  const DumpRange &r = DUMP_RANGES[this->dump_range_];
  uint16_t base = r.start + this->dump_offset_;
  size_t regs = data.size();

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

// Every terminal callback starts here: an answer of any shape, from data to an
// exception, proves the inverter is alive and talking. Returns false when the
// frame is not one we are waiting for and should be ignored.
bool GrowattInverter::answered_() {
  if (!this->is_enabled())
    return false;
  this->last_update_ = micros();
  if (!this->waiting_)
    return false;
  this->waiting_ = false;
  this->bus_release_ = millis();
  return true;
}

// Write acknowledgements now arrive in callbacks of their own, so a rejected
// write can no longer fall through into the identification state machine the
// way it once did - the routing does not depend on remembering to check
// writing_ first, because a write response has nowhere else to go.
void GrowattInverter::on_write_single_register(uint16_t address, uint16_t value,
                                               modbus::ResponseStatus status) {
  if (!this->answered_())
    return;
  this->finish_write_(status);
}

void GrowattInverter::on_write_multiple_registers(
    uint16_t start_address, std::span<const uint16_t> registers,
    modbus::ResponseStatus status) {
  if (!this->answered_())
    return;
  this->finish_write_(status);
}

void GrowattInverter::finish_write_(modbus::ResponseStatus status) {
  // A write that was never sent as a write cannot be retired as one. This only
  // fires if an echo outlives the transaction it belongs to.
  if (!this->writing_) {
    ESP_LOGD(TAG, "slot %u: write acknowledgement with no write pending",
             this->slot_index_);
    return;
  }
  const PendingWrite &w = this->write_queue_[this->write_head_];
  if (modbus::succeeded(status)) {
    ESP_LOGI(TAG, "slot %u: write to %u acknowledged", this->slot_index_,
             w.address);
  } else {
    ESP_LOGW(TAG,
             "slot %u: register %u rejected the write (exception %u); this "
             "model does not accept it, not trying again until the next "
             "identification",
             this->slot_index_, w.address,
             (unsigned) static_cast<uint8_t>(status.value()));
    this->mark_rejected_(w.address);
  }
  this->writing_ = false;
  this->write_head_ = (this->write_head_ + 1) % WRITE_QUEUE_SIZE;
  this->write_count_--;
  this->retries_ = 0;
  this->want_send_ = this->write_count_ > 0;
}

void GrowattInverter::on_read_registers(modbus::EntityType entity_type,
                                        uint16_t start_address,
                                        std::span<const uint16_t> data,
                                        modbus::ResponseStatus status) {
  if (!this->answered_())
    return;

  // A probe answering is all we needed; update_health_() notices the fresh
  // timestamp on the next cycle and starts identification. An exception counts
  // just as well: something replied.
  if (this->probing_) {
    this->probing_ = false;
    this->retries_ = 0;
    return;
  }

  if (!modbus::succeeded(status)) {
    ESP_LOGD(TAG, "slot %u: exception %u at register %u", this->slot_index_,
             (unsigned) static_cast<uint8_t>(status.value()), start_address);
    if (this->dump_active_) {
      ESP_LOGI(TAG, "DUMP slot %u: range %u not implemented, skipping",
               this->slot_index_, this->dump_range_);
      this->dump_skip_range_();
    } else if (this->poll_ != POLL_IDLE) {
      this->advance_poll_();
    } else if (this->step_ == IDENT_BDC) {
      // The probe asks a question a model without the 3000 block cannot even
      // parse, so an exception here is the answer "no BDC" and not a failed
      // identification. Marking the run incomplete would cost this slot three
      // full identification passes for having been asked politely.
      ESP_LOGD(TAG, "slot %u: no BDC register on this model -> grid-tie only",
               this->slot_index_);
      this->advance_(true);
    } else {
      this->advance_(false);
    }
    return;
  }

  if (this->dump_active_) {
    this->handle_dump_(data);
    return;
  }

  if (this->poll_ != POLL_IDLE) {
    const bool xh = this->caps_.storage_family == STORAGE_TLXH;
    switch (this->poll_) {
      case POLL_FAST_MAIN:
        if (data.size() >= POLL_FAST_MAIN_CNT)
          this->parse_fast_main_(data);
        break;
      case POLL_FAST_STATUS:
        if (data.size() >= POLL_FAST_STATUS_CNT)
          this->parse_fast_status_(data);
        break;
      case POLL_FAST_BAT:
        if (xh) {
          if (data.size() >= XH_FAST_BAT_CNT)
            this->parse_fast_bat_xh_(data);
        } else if (data.size() >= POLL_FAST_BAT_CNT) {
          this->parse_fast_bat_(data);
        }
        break;
      case POLL_FAST_UPS:
        if (data.size() >= POLL_FAST_UPS_CNT)
          this->parse_fast_ups_(data);
        break;
      case POLL_SLOW_MAIN:
        if (data.size() >= POLL_SLOW_MAIN_CNT)
          this->parse_slow_main_(data);
        break;
      case POLL_SLOW_STOR:
        if (xh) {
          if (data.size() >= XH_SLOW_STOR_CNT)
            this->parse_storage_xh_(data);
        } else if (data.size() >= POLL_SLOW_STOR_CNT) {
          this->parse_storage_(data);
        }
        break;
      default:
        break;
    }
    this->advance_poll_();
    return;
  }

  switch (this->step_) {
    case IDENT_LIVE: {
      if (data.size() < FIRST_GROUP_CNT) { this->advance_(false); return; }
      this->detect_from_live_(data);
      break;
    }
    case IDENT_INFO: {
      if (data.size() < FIRST_GROUP_CNT) { this->advance_(false); return; }
      this->parse_device_info_(data);
      break;
    }
    case IDENT_TYPE: {
      if (data.size() < REG_TYPE_CNT) { this->advance_(false); return; }
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
      if (data.size() < REG_CAPS_CNT) { this->advance_(false); return; }
      this->caps_.bdc_count = reg16(data, 1) & 0xFF;
      this->caps_.battery_packs = reg16(data, 2) & 0xFF;
      ESP_LOGI(TAG, "slot %u: PvStrScan=%u, BDC=%u, PackNum=%u",
               this->slot_index_, reg16(data, 0), this->caps_.bdc_count,
               this->caps_.battery_packs);
      break;
    }
    case IDENT_STORAGE: {
      if (data.size() < REG_STORAGE_CNT) { this->advance_(false); return; }
      uint16_t acc = 0;
      for (uint8_t i = 0; i < REG_STORAGE_CHECK; i++)
        acc |= reg16(data, i);
      this->caps_.has_storage = (acc != 0);
      if (this->caps_.has_storage) {
        this->caps_.storage_family = STORAGE_SPH;
        this->caps_.has_ups = (reg16(data, REG_UPS_OFFSET) & 0x01) != 0;
        ESP_LOGI(TAG, "slot %u: storage YES, UPS %s", this->slot_index_,
                 this->caps_.has_ups ? "enabled" : "disabled");
        // battery type (1048) and the UPS enable switch (1060) live here
        this->publish_reg_entities_(data, REG_STORAGE_BASE, REG_STORAGE_CNT);
      } else {
        this->caps_.has_ups = false;
        ESP_LOGD(TAG, "slot %u: no storage config at 1000, trying the BDC "
                 "register", this->slot_index_);
      }
      break;
    }
    case IDENT_BDC: {
      if (data.size() < REG_BDC_STATE_CNT) { this->advance_(false); return; }
      uint16_t state = reg16(data, 0);
      this->caps_.bdc_count = 0;
      for (uint8_t bit = 0; bit < 2; bit++)
        if (state & (1u << bit))
          this->caps_.bdc_count++;
      if (state != 0) {
        this->caps_.has_storage = true;
        this->caps_.storage_family = STORAGE_TLXH;
        // A MIN TL-XH has no EPS terminal at all, so there is nothing for a
        // 1060 style enable register to enable. The EPS block at 3145 exists
        // in the protocol for the family but reads dead on this hardware.
        this->caps_.has_ups = false;
        ESP_LOGI(TAG, "slot %u: BDC connected (state 0x%02X, %u BDC) -> "
                 "TL-XH storage", this->slot_index_, state,
                 this->caps_.bdc_count);
      } else {
        ESP_LOGI(TAG, "slot %u: no storage on either register family -> "
                 "grid-tie only", this->slot_index_);
      }
      break;
    }
    case IDENT_BATTERY: {
      bool xh = this->caps_.storage_family == STORAGE_TLXH;
      uint8_t need = xh ? XH_BAT_CNT : REG_BAT_CNT;
      if (data.size() < need) { this->advance_(false); return; }
      // 1013 is 0.1 V, 3169 is 0.01 V, and the SOC sits one register further
      // along on the TL-XH because Ibat comes between. Reporting a tenth of
      // the real pack voltage would sail straight through any sanity check,
      // which is why the two layouts get separate reads rather than an offset.
      uint16_t vbat = xh ? reg16(data, 0) / 10 : reg16(data, 0);
      uint16_t soc = xh ? reg16(data, 2) : reg16(data, 1);
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
      if (this->caps_.storage_family == STORAGE_TLXH) {
        if (data.size() < XH_SETTINGS_CNT) { this->advance_(false); return; }
        this->parse_settings_xh_(data);
      } else {
        if (data.size() < HO_SETTINGS_CNT) { this->advance_(false); return; }
        this->parse_settings_(data);
      }
      break;
    }
    default:
      return;
  }
  this->advance_(true);
}

// A reply whose length does not match the request never reaches the typed
// callbacks - the dispatcher diverts it here rather than handing over a short
// block dressed as a complete one. The old code caught the same case with its
// size guards and treated the block as unusable, so that is what happens here.
// Overriding this matters beyond tidiness: the default implementation only
// logs, which would leave this slot in waiting_ until its own timeout.
void GrowattInverter::on_custom_response(std::span<const uint8_t> request_pdu,
                                         std::span<const uint8_t> response_pdu,
                                         modbus::ResponseStatus status) {
  if (!this->answered_())
    return;

  ESP_LOGD(TAG, "slot %u: unusable reply (%u bytes), function 0x%02X",
           this->slot_index_, (unsigned) response_pdu.size(),
           request_pdu.empty() ? 0 : request_pdu[0]);

  if (this->probing_) {
    this->probing_ = false;
    this->retries_ = 0;
    return;
  }
  if (this->writing_) {
    // Nothing was confirmed, so this is not an acknowledgement. Retired as a
    // failure rather than marked rejected: a malformed frame says nothing about
    // whether the model accepts the register, and blacklisting it on that
    // evidence would suppress a write that might well work.
    ESP_LOGW(TAG, "slot %u: write to %u answered with a malformed frame, dropping",
             this->slot_index_, this->write_queue_[this->write_head_].address);
    this->writing_ = false;
    this->write_head_ = (this->write_head_ + 1) % WRITE_QUEUE_SIZE;
    this->write_count_--;
    this->retries_ = 0;
    this->want_send_ = this->write_count_ > 0;
    return;
  }
  if (this->dump_active_) {
    this->dump_skip_range_();
    return;
  }
  if (this->poll_ != POLL_IDLE) {
    this->advance_poll_();
    return;
  }
  this->advance_(false);
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
  if (this->caps_.storage_family != STORAGE_NONE)
    ESP_LOGCONFIG(TAG, "  storage registers: %s",
                  this->caps_.storage_family == STORAGE_TLXH
                      ? "TL-XH (3125+)" : "SPH (1000+)");
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
