#pragma once

#include <memory>
#include <string>
#include <vector>

#include "esphome/components/modbus/modbus.h"
#include "esphome/components/socket/socket.h"
#include "esphome/core/component.h"

namespace esphome {
namespace modbus_tcp {

// A Modbus TCP transport for the stock client hub.
//
// Everything above the wire - the transmit queue, the selection function, the
// send-wait watchdog, response validation and the whole ModbusClientDevice
// callback surface - is upstream's and is not touched here. This class replaces
// exactly two things: how a frame leaves, and how bytes arrive.
//
// The trick that keeps it that small is translating at the boundary rather than
// at the parser. The hub hands down an RTU frame and expects RTU frames back in
// rx_buffer_; on the way out the CRC is dropped and an MBAP header prepended, on
// the way in the header is stripped and a CRC recomputed. parse_modbus_frames()
// above never learns that the transport changed. The recomputed CRC is redundant
// work on a link that already guarantees integrity, but it buys an unmodified
// parser, which is the better trade for a first version.
class ModbusTcpClientHub : public modbus::ModbusClientHub {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_host(const std::string &host) { this->host_ = host; }
  void set_port(uint16_t port) { this->port_ = port; }
  void set_reconnect_interval(uint32_t ms) { this->reconnect_interval_ = ms; }

  bool is_connected() const { return this->state_ == State::CONNECTED; }
  // Responses that arrived carrying a Transaction ID we are no longer waiting
  // for. On a single-client link there is no innocent explanation: the peer
  // answered a request we had already given up on, which means its own timeout
  // sits above ours. Exposed so the condition is visible in Home Assistant
  // rather than only in a log nobody reads eight months from now.
  uint32_t get_late_response_count() const { return this->late_responses_; }

 protected:
  // The two seams. Both are no-ops while disconnected, which is what keeps the
  // hub's own watchdog in charge of resolving whatever was already in flight.
  bool send_frame_(const modbus::ModbusFrame &frame) override;
  void receive_bytes_() override;

  // Upstream's version consults the UART, which does not exist here. The rest of
  // the conditions carry over unchanged: one frame in flight at a time, nothing
  // half-parsed in the buffer.
  bool tx_blocked() override;

  enum class State : uint8_t { IDLE, CONNECTING, CONNECTED };

  void start_connect_();
  void check_connect_();
  void disconnect_(const char *reason);
  // Pulls whatever the socket has into staging_ and lifts every complete MBAP
  // frame out of it. Returns false if the peer closed.
  bool drain_socket_();
  // Converts one MBAP frame into the RTU frame the parser above expects.
  void deliver_frame_(const uint8_t *mbap, uint16_t total_len);

  std::string host_;
  uint16_t port_{502};
  uint32_t reconnect_interval_{5000};

  std::unique_ptr<socket::Socket> sock_;
  State state_{State::IDLE};
  uint32_t last_connect_attempt_{0};
  uint32_t connect_started_{0};

  // Bytes read from the socket that do not yet form a whole MBAP frame. Distinct
  // from the hub's rx_buffer_, which only ever holds complete, RTU-shaped frames.
  std::vector<uint8_t> staging_;

  uint16_t next_transaction_id_{0};
  uint16_t pending_transaction_id_{0};
  bool have_pending_{false};
  uint32_t late_responses_{0};
};

}  // namespace modbus_tcp
}  // namespace esphome
