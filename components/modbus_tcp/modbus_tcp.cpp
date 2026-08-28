#include "modbus_tcp.h"

#include <cerrno>
#include <cinttypes>
#include <cstring>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace modbus_tcp {

static const char *const TAG = "modbus_tcp";

// MBAP: [txn hi][txn lo][proto hi][proto lo][len hi][len lo][unit]
static constexpr uint8_t MBAP_HEADER_SIZE = 7;
// The length field counts the unit identifier plus the PDU.
static constexpr uint8_t MBAP_LENGTH_OFFSET = 4;
static constexpr uint16_t MAX_PDU_SIZE = 253;

void ModbusTcpClientHub::setup() {
  // Deliberately not calling Modbus::setup(): it derives the RTU inter-frame
  // delay from the UART baud rate, and there is no UART. Both delays are a
  // property of a silent line between characters, which TCP does not have -
  // frames arrive whole or not at all - so zero is the honest value. It also
  // keeps timeout_() from ever declaring a partial response stale.
  this->frame_delay_ms_ = 0;
  this->long_rx_buffer_delay_ms_ = 0;
  // Turnaround is an RS-485 courtesy to slaves sharing one pair. Nothing here
  // shares anything, and leaving it set would only slow the link down.
  this->turnaround_delay_ms_ = 0;
  this->staging_.reserve(modbus::MAX_FRAME_SIZE);
}

void ModbusTcpClientHub::loop() {
  switch (this->state_) {
    case State::IDLE:
      if (millis() - this->last_connect_attempt_ >= this->reconnect_interval_)
        this->start_connect_();
      break;
    case State::CONNECTING:
      this->check_connect_();
      break;
    case State::CONNECTED:
      break;
  }

  // Runs the sweep, receive_bytes_(), the parser and the send-wait watchdog. It
  // is called even while disconnected on purpose: a request that was in flight
  // when the link dropped still has to resolve through the hub's own timeout,
  // exactly as a mute slave would, so no caller is left without its callback.
  modbus::ModbusClientHub::loop();
}

void ModbusTcpClientHub::start_connect_() {
  this->last_connect_attempt_ = millis();

  this->sock_ = socket::socket_ip(SOCK_STREAM, IPPROTO_TCP);
  if (this->sock_ == nullptr) {
    ESP_LOGW(TAG, "Could not create socket");
    return;
  }
  if (this->sock_->setblocking(false) != 0) {
    ESP_LOGW(TAG, "Could not set socket non-blocking: %s", strerror(errno));
    this->sock_ = nullptr;
    return;
  }
  // Nagle would coalesce a request with whatever follows it. Modbus request
  // frames are tiny and latency-sensitive, which is the case the option exists
  // to disable.
  int flag = 1;
  this->sock_->setsockopt(IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

  struct sockaddr_storage addr {};
  socklen_t addr_len = socket::set_sockaddr(reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr), this->host_,
                                            this->port_);
  if (addr_len == 0) {
    ESP_LOGW(TAG, "Could not resolve '%s'", this->host_.c_str());
    this->sock_ = nullptr;
    return;
  }

  int err = this->sock_->connect(reinterpret_cast<struct sockaddr *>(&addr), addr_len);
  if (err != 0 && errno != EINPROGRESS && errno != EALREADY) {
    ESP_LOGW(TAG, "Connect to %s:%u failed: %s", this->host_.c_str(), this->port_, strerror(errno));
    this->sock_ = nullptr;
    return;
  }

  this->connect_started_ = millis();
  this->state_ = State::CONNECTING;
  ESP_LOGD(TAG, "Connecting to %s:%u", this->host_.c_str(), this->port_);
}

void ModbusTcpClientHub::check_connect_() {
  if (this->sock_ == nullptr) {
    this->disconnect_(nullptr);
    return;
  }

  // A non-blocking connect signals completion by making the socket writable.
  // This is the only test that actually works on lwIP: SO_ERROR stays 0 while
  // the handshake is in flight, and getpeername() returns the remote address as
  // soon as connect() is called, because lwIP stores it on the pcb before SYN is
  // even answered. Both report success against a host that is switched off.
  fd_set write_fds;
  FD_ZERO(&write_fds);
  const int fd = this->sock_->get_fd();
  FD_SET(fd, &write_fds);
  struct timeval tv {};  // poll, never block the main loop

  int ready = ::select(fd + 1, nullptr, &write_fds, nullptr, &tv);
  if (ready < 0) {
    this->disconnect_("select failed");
    return;
  }
  if (ready == 0 || !FD_ISSET(fd, &write_fds)) {
    // Bounded by the same reconnect interval rather than a separate knob: a
    // connect that has not completed in that window is not going to.
    if (millis() - this->connect_started_ > this->reconnect_interval_)
      this->disconnect_("connect timed out");
    return;
  }

  // Writable means the connect resolved - but resolved either way. SO_ERROR now
  // carries the verdict, and a refused connection also arrives as writable.
  int soerr = 0;
  socklen_t len = sizeof(soerr);
  if (this->sock_->getsockopt(SOL_SOCKET, SO_ERROR, &soerr, &len) != 0) {
    this->disconnect_("getsockopt failed");
    return;
  }
  if (soerr != 0) {
    ESP_LOGW(TAG, "Connect to %s:%u failed: %s", this->host_.c_str(), this->port_, strerror(soerr));
    this->disconnect_(nullptr);
    return;
  }

  this->state_ = State::CONNECTED;
  this->staging_.clear();
  ESP_LOGI(TAG, "Connected to %s:%u after %" PRIu32 "ms", this->host_.c_str(), this->port_,
           millis() - this->connect_started_);
}

void ModbusTcpClientHub::disconnect_(const char *reason) {
  if (reason != nullptr && this->state_ != State::IDLE)
    ESP_LOGW(TAG, "Disconnected from %s:%u: %s", this->host_.c_str(), this->port_, reason);
  if (this->sock_ != nullptr) {
    this->sock_->close();
    this->sock_ = nullptr;
  }
  this->state_ = State::IDLE;
  this->last_connect_attempt_ = millis();
  this->staging_.clear();
  // Anything half-received is meaningless across a reconnect. The in-flight
  // request itself is left alone: the hub's watchdog owns it and will retire it.
  this->rx_buffer_.clear();
  this->have_pending_ = false;
}

bool ModbusTcpClientHub::tx_blocked() {
  if (this->state_ != State::CONNECTED || this->sock_ == nullptr)
    return true;
  // Upstream's conditions minus the UART one: still a single frame in flight,
  // still nothing sent while a response sits half-parsed.
  return this->waiting_for_response_ || !this->rx_buffer_.empty() ||
         this->tx_delay_remaining() > modbus::MODBUS_TX_MAX_DELAY_MS;
}

bool ModbusTcpClientHub::timeout_() {
  // Always "finished". Not a shortcut: deliver_frame_() only ever appends whole
  // frames, so a partial response cannot exist in rx_buffer_ by construction.
  // There is also nothing to measure - TCP has no inter-character silence, which
  // is the only thing upstream's version is timing.
  return true;
}

bool ModbusTcpClientHub::send_frame_(const modbus::ModbusFrame &frame) {
  // Both conditions: state_ and sock_ are set together everywhere, but a null
  // dereference here faults the whole node, so the redundant check is cheap.
  if (this->state_ != State::CONNECTED || this->sock_ == nullptr)
    return false;

  const std::span<const uint8_t> pdu = frame.pdu();
  if (pdu.empty() || pdu.size() > MAX_PDU_SIZE) {
    ESP_LOGW(TAG, "Refusing to send a %zu byte PDU", pdu.size());
    return false;
  }

  const uint16_t tid = this->next_transaction_id_++;
  const uint16_t length = static_cast<uint16_t>(pdu.size() + 1);  // unit id + PDU

  uint8_t buf[MBAP_HEADER_SIZE + MAX_PDU_SIZE];
  buf[0] = tid >> 8;
  buf[1] = tid & 0xFF;
  buf[2] = 0;  // protocol identifier, always 0 for Modbus
  buf[3] = 0;
  buf[4] = length >> 8;
  buf[5] = length & 0xFF;
  buf[6] = frame.address();
  memcpy(buf + MBAP_HEADER_SIZE, pdu.data(), pdu.size());

  const size_t total = MBAP_HEADER_SIZE + pdu.size();
  ssize_t written = this->sock_->write(buf, total);
  if (written < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      // The frame never left. Returning false is the same contract as an RTU
      // send deferred by an inbound byte: the hub keeps the entry and retries.
      ESP_LOGV(TAG, "Send would block, deferring");
      return false;
    }
    ESP_LOGW(TAG, "Write failed: %s", strerror(errno));
    this->disconnect_(nullptr);
    return false;
  }
  if (static_cast<size_t>(written) != total) {
    // A short write leaves a torn frame on the wire that no amount of retrying
    // can repair, so the connection is the thing to throw away.
    this->disconnect_("short write");
    return false;
  }

  this->pending_transaction_id_ = tid;
  this->have_pending_ = true;
  this->last_send_ = millis();
  // Consumed by tx_delay_remaining() to model how long an RTU frame occupies the
  // line. A TCP frame occupies nothing once written.
  this->last_send_tx_offset_ = 0;

  ESP_LOGV(TAG, "Write TID 0x%04X unit %u: %s", tid, frame.address(),
           format_hex_pretty(buf + MBAP_HEADER_SIZE, pdu.size()).c_str());
  return true;
}

void ModbusTcpClientHub::receive_bytes_() {
  this->last_receive_check_ = millis();
  if (this->state_ != State::CONNECTED || this->sock_ == nullptr)
    return;
  if (!this->drain_socket_())
    return;

  // Lift out every complete MBAP frame. The socket contract requires reading
  // until it would block, so more than one response can land in a single pass
  // even though only one request is ever in flight.
  size_t offset = 0;
  while (this->staging_.size() - offset >= MBAP_HEADER_SIZE) {
    const uint8_t *p = this->staging_.data() + offset;
    const uint16_t proto = (static_cast<uint16_t>(p[2]) << 8) | p[3];
    const uint16_t length = (static_cast<uint16_t>(p[MBAP_LENGTH_OFFSET]) << 8) | p[MBAP_LENGTH_OFFSET + 1];

    if (proto != 0 || length < 2 || length > MAX_PDU_SIZE + 1) {
      // The stream is no longer frame-aligned and there is no silence to
      // resynchronise on, unlike RTU. Only the connection can be resynchronised.
      this->disconnect_("malformed MBAP header");
      return;
    }
    const size_t frame_len = MBAP_HEADER_SIZE + length - 1;
    if (this->staging_.size() - offset < frame_len)
      break;  // frame still arriving

    this->deliver_frame_(p, static_cast<uint16_t>(frame_len));
    offset += frame_len;
  }

  if (offset > 0)
    this->staging_.erase(this->staging_.begin(), this->staging_.begin() + offset);
}

bool ModbusTcpClientHub::drain_socket_() {
  uint8_t chunk[256];
  for (;;) {
    ssize_t n = this->sock_->read(chunk, sizeof(chunk));
    if (n > 0) {
      this->staging_.insert(this->staging_.end(), chunk, chunk + n);
      this->last_modbus_byte_ = this->last_receive_check_;
      continue;
    }
    if (n == 0) {
      this->disconnect_("peer closed the connection");
      return false;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return true;  // drained
    ESP_LOGW(TAG, "Read failed: %s", strerror(errno));
    this->disconnect_(nullptr);
    return false;
  }
}

void ModbusTcpClientHub::deliver_frame_(const uint8_t *mbap, uint16_t total_len) {
  const uint16_t tid = (static_cast<uint16_t>(mbap[0]) << 8) | mbap[1];
  const uint8_t unit = mbap[6];
  const uint8_t *pdu = mbap + MBAP_HEADER_SIZE;
  const uint16_t pdu_len = total_len - MBAP_HEADER_SIZE;

  if (!this->have_pending_ || tid != this->pending_transaction_id_) {
    // The whole reason for tracking Transaction IDs at all. On a link with one
    // client, a reply we are not waiting for means the peer's own response
    // timeout is longer than ours: we gave up, it did not, and the answer turned
    // up late. Naming the likely cause here is what stops this from becoming an
    // unexplained counter.
    this->late_responses_++;
    ESP_LOGW(TAG,
             "Late or unexpected response, TID 0x%04X (expected 0x%04X), unit %u. The peer answered a request this "
             "hub had already timed out - its response timeout is likely longer than send_wait_time.",
             tid, this->have_pending_ ? this->pending_transaction_id_ : 0, unit);
    return;
  }
  this->have_pending_ = false;

  // Reassemble the RTU frame the parser above is written against: address, PDU,
  // CRC. The CRC cannot fail here - we just computed it - which is precisely why
  // the parser's validation of it costs nothing and is left in place.
  const size_t at = this->rx_buffer_.size();
  this->rx_buffer_.resize(at + pdu_len + 3);
  uint8_t *out = this->rx_buffer_.data() + at;
  out[0] = unit;
  memcpy(out + 1, pdu, pdu_len);
  const uint16_t crc = crc16(out, pdu_len + 1);
  out[pdu_len + 1] = crc & 0xFF;
  out[pdu_len + 2] = crc >> 8;

  ESP_LOGV(TAG, "Read TID 0x%04X unit %u, %" PRIu32 "ms after send: %s", tid, unit, millis() - this->last_send_,
           format_hex_pretty(pdu, pdu_len).c_str());
}

void ModbusTcpClientHub::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Modbus TCP:\n"
                "  Host: %s:%u\n"
                "  Send Wait Time: %ums\n"
                "  Reconnect Interval: %" PRIu32 "ms",
                this->host_.c_str(), this->port_, this->send_wait_time_, this->reconnect_interval_);
}

}  // namespace modbus_tcp
}  // namespace esphome
