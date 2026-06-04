#pragma once

#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <map>

#include "esphome/core/component.h"
#include "esphome/core/log.h"


namespace esphome::modbus_tcp_server {

// EM24 Ethernet register map (dbus-modbus-client carlo_gavazzi.py, models 1648-1653)
// All multi-register values are Reg_s32l: little-endian word order (low word at lower address)
static constexpr uint8_t MAX_CLIENTS = 5;
static constexpr uint16_t MAX_BUF = 260;
static constexpr uint32_t CLIENT_TIMEOUT_MS = 10000;

struct Client {
  int fd{-1};
  uint8_t buf[MAX_BUF];
  uint16_t buf_len{0};
  uint32_t last_recv_ms{0};
};

// 4.3 MODBUS Data model
enum class ModbusPrimaryTypes : uint8_t {
  CUSTOM = 0x01,
  COIL = 0x01,
  DISCRETE_INPUT = 0x02,
  HOLDING = 0x03,
  READ =0x04,
};

class ModbusTcpServerComponent : public Component {
 public:

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void DvcEM24_Propagate(uint8_t uid, const char *serial_number);
  void DvcEM24_Upd_Power(uint8_t uid, uint8_t phase, float value, bool calc_current = true );
  void DvcEM24_Upd_Current(uint8_t uid, uint8_t phase, float value);
  void DvcEM24_Upd_Voltage(uint8_t uid, uint8_t phase, float value );
  void DvcEM24_Upd_Energy_Import_Phases(uint8_t uid, uint8_t phase, float value );
  void DvcEM24_Upd_Energy_Import_Total(uint8_t uid, float value );
  void DvcEM24_Upd_Energy_Export_Total(uint8_t uid, float value );

  void set_port(uint16_t port) { port_ = port; }

 protected:
  // TCP Server Port
  uint16_t port_;


  // Dense register data
  std::map<uint8_t, std::map<ModbusPrimaryTypes, std::map<uint16_t, uint16_t>>> regdata_;

  // TCP server
  int server_fd_{-1};
  Client clients_[MAX_CLIENTS];

  // Helpers
  void accept_clients_();
  void process_client_(Client &c);
  void handle_frame_(Client &c, uint16_t frame_len);
  void send_response_(int fd, uint16_t txid, uint8_t uid, const uint8_t *pdu, uint8_t pdu_len);
  void send_exception_(int fd, uint16_t txid, uint8_t uid, uint8_t fc, uint8_t code);
  void close_client_(Client &c);

  // Sparse register lookup: returns value for any EM24 address, including out-of-dense-range
  uint16_t get_register_(uint8_t uid, ModbusPrimaryTypes mbpt, uint16_t addr);
  
  // Write signed int32 as two little-endian uint16 registers (Reg_s32l: low word at idx)
  void write_int32_(uint8_t uid, ModbusPrimaryTypes mbpt, uint16_t idx, float value);
  // Read signed int32 as two little-endian uint16 registers (Reg_s32l: low word at idx)
  float read_int32_(uint8_t uid, ModbusPrimaryTypes mbpt, uint16_t idx);

};

}  // namespace esphome::grid_meter
