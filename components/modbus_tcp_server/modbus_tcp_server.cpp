#include "modbus_tcp_server.h"

#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>

#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome::modbus_tcp_server {

static const char *const TAG = "modbus_tcp_server";


// ---------- helpers ----------

// Read signed int32 as two little-endian uint16 registers (Reg_s32l: low word at idx)
float ModbusTcpServerComponent::read_int32_(uint8_t uid,ModbusPrimaryTypes mbpt, uint16_t idx) {
  uint32_t value_combined = ((uint32_t)get_register_(uid,mbpt,idx + 1) << 16) | get_register_(uid,mbpt,idx);
  int32_t value_signed = (int32_t)value_combined;
  return (float) value_signed;
}

// Sparse lookup for registers from dictionary.
uint16_t ModbusTcpServerComponent::get_register_(uint8_t uid, ModbusPrimaryTypes mbpt, uint16_t addr) {
  if ( regdata_.contains(uid) and regdata_[uid].contains(mbpt) and regdata_[uid][mbpt].contains(addr) ) {
    return regdata_[uid][mbpt][addr];
  }
  else {
    return 0;
  }
}
// Generic public Modbus setter
void ModbusTcpServerComponent::MBR_Upd(uint8_t uid, ModbusPrimaryTypes mbpt, uint16_t idx, ModbusValueType mbvt, float value_f) {
  int64_t value = static_cast<int64_t>(value_f + (value_f >= 0 ? 0.5f : -0.5f));
  switch (mbvt) {
    case ModbusValueType::U_WORD:
    case ModbusValueType::S_WORD:
      regdata_[uid][mbpt][idx] = (value & 0xFFFF);
      break;
    case ModbusValueType::U_DWORD:
    case ModbusValueType::S_DWORD:
    case ModbusValueType::FP32:
      regdata_[uid][mbpt][idx] = ((value & 0xFFFF0000) >> 16);
      regdata_[uid][mbpt][idx + 1] = (value & 0xFFFF);
      break;
    case ModbusValueType::U_DWORD_R:
    case ModbusValueType::S_DWORD_R:
    case ModbusValueType::FP32_R:
      regdata_[uid][mbpt][idx] = (value & 0xFFFF);
      regdata_[uid][mbpt][idx + 1] = ((value & 0xFFFF0000) >> 16);
      break;
    case ModbusValueType::U_QWORD:
    case ModbusValueType::S_QWORD:
      regdata_[uid][mbpt][idx] = ((value & 0xFFFF000000000000) >> 48);
      regdata_[uid][mbpt][idx + 1] = ((value & 0xFFFF00000000) >> 32);
      regdata_[uid][mbpt][idx + 2] = ((value & 0xFFFF0000) >> 16);
      regdata_[uid][mbpt][idx + 2] = (value & 0xFFFF);
      break;
    case ModbusValueType::U_QWORD_R:
    case ModbusValueType::S_QWORD_R:
      regdata_[uid][mbpt][idx] = (value & 0xFFFF);
      regdata_[uid][mbpt][idx + 1] = ((value & 0xFFFF0000) >> 16);
      regdata_[uid][mbpt][idx + 2] = ((value & 0xFFFF00000000) >> 32);
      regdata_[uid][mbpt][idx + 3] = ((value & 0xFFFF000000000000) >> 48);
      break;
    default:
      ESP_LOGE(TAG, "Invalid data type for modbus number to payload conversion: %d", static_cast<uint16_t>(mbvt));
      break;
  } 
}

// Victron EM24
void ModbusTcpServerComponent::DvcEM24_Propagate(uint8_t uid, const char *serial_number) {
  regdata_[uid][ModbusPrimaryTypes::HOLDING][0x000B ] = 1651;    // 0x000B Model ID register (probed by carlo_gavazzi.py)
  regdata_[uid][ModbusPrimaryTypes::HOLDING][0x0033] = 500 ;    // 0x0033 Frequency: 50.0 Hz (Reg_u16, /10 Hz)


  DvcEM24_Upd_Voltage(uid, 1, 240 );
  DvcEM24_Upd_Voltage(uid, 2, 240 );
  DvcEM24_Upd_Voltage(uid, 3, 240 );

  regdata_[uid][ModbusPrimaryTypes::HOLDING][0x0302] = 0x0100;  // 0x0302 HW version 1.0.0
  regdata_[uid][ModbusPrimaryTypes::HOLDING][0x0304] = 0x0100;  // 0x0304 FW version 1.0.0
  regdata_[uid][ModbusPrimaryTypes::HOLDING][0x1002] = 0;       // 0x1002 PhaseConfig = 3P --> 0, 1P --> 3
  regdata_[uid][ModbusPrimaryTypes::HOLDING][0xa000] = 7;       // 0xa000 Application = H mode
  regdata_[uid][ModbusPrimaryTypes::HOLDING][0xa100] = 3;       // 0xa100 SwitchPos = '1' (active kWh, both directions)

  for (uint8_t i = 0; i < 7; i++) {
    uint8_t hi = (serial_number[i * 2]     != '\0') ? (uint8_t)serial_number[i * 2]     : '0';
    uint8_t lo = (serial_number[i * 2 + 1] != '\0') ? (uint8_t)serial_number[i * 2 + 1] : '0';
    regdata_[uid][ModbusPrimaryTypes::HOLDING][0x5000 + i] = ((uint16_t)hi << 8) | lo;
  }
  ESP_LOGD(TAG, "EM24 Victron Populated: s%", serial_number);
}

void ModbusTcpServerComponent::DvcEM24_Upd_Power(uint8_t uid, uint8_t phase, float value, bool calc_current ) {
  if ( phase >=1 and phase <=3 ) {
    MBR_Upd(uid, ModbusPrimaryTypes::HOLDING, 0x12 + ( phase - 1 ) * 2 , ModbusValueType::S_DWORD_R, value * 10.0f);
    
    MBR_Upd(uid, ModbusPrimaryTypes::HOLDING, 0x28, ModbusValueType::S_DWORD_R,
        read_int32_(uid, ModbusPrimaryTypes::HOLDING, 0x12 + 0) 
      + read_int32_(uid, ModbusPrimaryTypes::HOLDING, 0x12 + 2) 
      + read_int32_(uid, ModbusPrimaryTypes::HOLDING, 0x12 + 4));
    if ( calc_current and read_int32_(uid, ModbusPrimaryTypes::HOLDING, 0x00 + ( phase - 1 ) * 2) != 0 ) {
      DvcEM24_Upd_Current(uid, phase, value / read_int32_(uid, ModbusPrimaryTypes::HOLDING, 0x00 + ( phase - 1 ) * 2) * 10.0f );
    }
  }
}

void ModbusTcpServerComponent::DvcEM24_Upd_Current(uint8_t uid, uint8_t phase, float value ) {
  if ( phase >=1 and phase <=3 ) {
    MBR_Upd(uid, ModbusPrimaryTypes::HOLDING, 0x0C + ( phase - 1 ) * 2 , ModbusValueType::S_DWORD_R, value * 1000.0f);
  }
}

void ModbusTcpServerComponent::DvcEM24_Upd_Voltage(uint8_t uid, uint8_t phase, float value ) {
  if ( phase >=1 and phase <=3 ) {
    MBR_Upd(uid, ModbusPrimaryTypes::HOLDING, 0x00 + ( phase - 1 ) * 2 , ModbusValueType::S_DWORD_R, value * 10.0f);
  }
}

void ModbusTcpServerComponent::DvcEM24_Upd_Energy_Import_Phases(uint8_t uid, uint8_t phase, float value ) {
  if ( phase >=1 and phase <=3 ) {
    MBR_Upd(uid, ModbusPrimaryTypes::HOLDING, 0x40 + ( phase - 1 ) * 2 , ModbusValueType::S_DWORD_R, value * 10.0f);
  }
}

void ModbusTcpServerComponent::DvcEM24_Upd_Energy_Import_Total(uint8_t uid, float value ) {
  MBR_Upd(uid, ModbusPrimaryTypes::HOLDING, 0x34 , ModbusValueType::S_DWORD_R, value * 10.0f);
}

void ModbusTcpServerComponent::DvcEM24_Upd_Energy_Export_Total(uint8_t uid, float value ) {
  MBR_Upd(uid, ModbusPrimaryTypes::HOLDING, 0x4E , ModbusValueType::S_DWORD_R, value * 10.0f);
}


// ---------- lifecycle ----------

void ModbusTcpServerComponent::setup() {

  // Open non-blocking TCP socket on port given port
  this->server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (this->server_fd_ < 0) {
    ESP_LOGE(TAG, "socket() failed: %d", errno);
    this->mark_failed();
    return;
  }

  int opt = 1;
  ::setsockopt(this->server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port_);

  if (::bind(this->server_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    ESP_LOGE(TAG, "bind() failed on port: %d", errno);
    ::close(this->server_fd_);
    this->server_fd_ = -1;
    this->mark_failed();
    return;
  }

  if (::listen(this->server_fd_, 2) < 0) {
    ESP_LOGE(TAG, "listen() failed: %d", errno);
    ::close(this->server_fd_);
    this->server_fd_ = -1;
    this->mark_failed();
    return;
  }

  int flags = ::fcntl(this->server_fd_, F_GETFL, 0);
  ::fcntl(this->server_fd_, F_SETFL, flags | O_NONBLOCK);

  ESP_LOGI(TAG, "Modbus TCP server listening on port %i ", port_);
}

void ModbusTcpServerComponent::loop() {
  if (this->server_fd_ < 0)
    return;

  this->accept_clients_();

  for (auto &c : this->clients_) {
    if (c.fd >= 0)
      this->process_client_(c);
  }
}

void ModbusTcpServerComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus TCP Server:");
}


// ---------- TCP server ----------

void ModbusTcpServerComponent::accept_clients_() {
  struct sockaddr_in client_addr{};
  socklen_t addr_len = sizeof(client_addr);
  int fd;
  while ((fd = ::accept(this->server_fd_, reinterpret_cast<struct sockaddr *>(&client_addr), &addr_len)) >= 0) {
    bool accepted = false;
    for (auto &c : this->clients_) {
      if (c.fd < 0) {
        int flags = ::fcntl(fd, F_GETFL, 0);
        ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        c.fd = fd;
        c.buf_len = 0;
        c.last_recv_ms = millis();
        ESP_LOGD(TAG, "Client connected (fd=%d)", fd);
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      ESP_LOGW(TAG, "Max clients reached, rejecting connection");
      ::close(fd);
    }
    addr_len = sizeof(client_addr);
  }
}

void ModbusTcpServerComponent::close_client_(Client &c) {
  if (c.fd >= 0) {
    ESP_LOGD(TAG, "Closing client (fd=%d)", c.fd);
    ::close(c.fd);
    c.fd = -1;
    c.buf_len = 0;
  }
}

void ModbusTcpServerComponent::process_client_(Client &c) {
  // Timeout check
  if (millis() - c.last_recv_ms > CLIENT_TIMEOUT_MS) {
    ESP_LOGD(TAG, "Client timeout (fd=%d)", c.fd);
    this->close_client_(c);
    return;
  }

  // Read into buffer
  int n = ::recv(c.fd, c.buf + c.buf_len, MAX_BUF - c.buf_len, 0);
  if (n == 0) {
    this->close_client_(c);
    return;
  }
  if (n < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK)
      this->close_client_(c);
    return;
  }
  c.buf_len += static_cast<uint16_t>(n);
  c.last_recv_ms = millis();

  // Buffer overflow: no complete frame in 260 bytes
  if (c.buf_len >= MAX_BUF) {
    ESP_LOGD(TAG, "Client buffer overflow (fd=%d), closing", c.fd);
    this->close_client_(c);
    return;
  }

  // Process all complete frames in buffer
  while (c.buf_len >= 6) {
    uint16_t proto_id = (c.buf[2] << 8) | c.buf[3];
    uint16_t pdu_length = (c.buf[4] << 8) | c.buf[5];

    if (proto_id != 0x0000) {
      ESP_LOGD(TAG, "Invalid protocol ID %04X, closing", proto_id);
      this->close_client_(c);
      return;
    }

    // pdu_length must include at least unit-id + function-code bytes
    if (pdu_length < 2) {
      ESP_LOGD(TAG, "PDU length %u too short, closing", pdu_length);
      this->close_client_(c);
      return;
    }

    uint16_t frame_len = 6 + pdu_length;
    if (frame_len > MAX_BUF) {
      ESP_LOGD(TAG, "Frame too large (%u bytes), closing", frame_len);
      this->close_client_(c);
      return;
    }

    if (c.buf_len < frame_len)
      break;  // incomplete frame

    this->handle_frame_(c, frame_len);

    c.buf_len -= frame_len;
    if (c.buf_len > 0)
      memmove(c.buf, c.buf + frame_len, c.buf_len);
  }
}

void ModbusTcpServerComponent::handle_frame_(Client &c, uint16_t frame_len) {
  uint16_t txid = (c.buf[0] << 8) | c.buf[1];
  uint8_t uid = c.buf[6];
  uint8_t fc = c.buf[7];

  if (fc == 0x03 || fc == 0x04) {
    // FC03 / FC04: Read Holding/Input Registers
    if (frame_len < 12) {
      this->send_exception_(c.fd, txid, uid, fc, 0x03);
      return;
    }
    uint16_t start = (c.buf[8] << 8) | c.buf[9];
    uint16_t count = (c.buf[10] << 8) | c.buf[11];
    ESP_LOGI(TAG, "FC%02X start=0x%04X count=%u", fc, start, count);

    if (count == 0 || count > 125) {
      this->send_exception_(c.fd, txid, uid, fc, 0x03);  // Illegal Data Value
      return;
    }

    // Build response using sparse register lookup -- returns 0 for any unknown address
    ModbusPrimaryTypes mbpt = ( fc == 0x03 ? ModbusPrimaryTypes::HOLDING : ModbusPrimaryTypes::READ );
    uint8_t pdu[2 + 125 * 2];
    pdu[0] = fc;
    pdu[1] = static_cast<uint8_t>(count * 2);
    for (uint16_t i = 0; i < count; i++) {
      uint16_t addr = static_cast<uint16_t>(static_cast<uint32_t>(start) + i);
      uint16_t val = get_register_(uid,mbpt, addr);
      pdu[2 + i * 2]     = val >> 8;
      pdu[2 + i * 2 + 1] = val & 0xFF;
    }
    this->send_response_(c.fd, txid, uid, pdu, static_cast<uint8_t>(2 + count * 2));

  } else if (fc == 0x06) {
    // FC06: Write Single Register -- accept as no-op (echo request back)
    if (frame_len < 12) {
      this->send_exception_(c.fd, txid, uid, fc, 0x03);
      return;
    }
    uint16_t addr = (c.buf[8] << 8) | c.buf[9];
    uint16_t val = (c.buf[10] << 8) | c.buf[11];
    ESP_LOGI(TAG, "FC06 write addr=0x%04X val=0x%04X (ignored)", addr, val);
    uint8_t pdu[5] = {fc, c.buf[8], c.buf[9], c.buf[10], c.buf[11]};
    this->send_response_(c.fd, txid, uid, pdu, 5);

  } else if (fc == 0x10) {
    // FC16: Write Multiple Registers -- accept as no-op (echo address + count)
    if (frame_len < 13) {
      this->send_exception_(c.fd, txid, uid, fc, 0x03);
      return;
    }
    uint16_t addr = (c.buf[8] << 8) | c.buf[9];
    uint16_t count = (c.buf[10] << 8) | c.buf[11];
    ESP_LOGI(TAG, "FC16 write addr=0x%04X count=%u (ignored)", addr, count);
    uint8_t pdu[5] = {fc, c.buf[8], c.buf[9], c.buf[10], c.buf[11]};
    this->send_response_(c.fd, txid, uid, pdu, 5);

  } else {
    ESP_LOGW(TAG, "FC%02X unsupported", fc);
    this->send_exception_(c.fd, txid, uid, fc, 0x01);  // Illegal Function
  }
}

void ModbusTcpServerComponent::send_response_(int fd, uint16_t txid, uint8_t uid, const uint8_t *pdu, uint8_t pdu_len) {
  uint8_t frame[7 + 125 * 2];
  frame[0] = txid >> 8;
  frame[1] = txid & 0xFF;
  frame[2] = 0x00;
  frame[3] = 0x00;
  frame[4] = static_cast<uint8_t>((1 + pdu_len) >> 8);
  frame[5] = static_cast<uint8_t>((1 + pdu_len) & 0xFF);
  frame[6] = uid;
  memcpy(frame + 7, pdu, pdu_len);
  int total = 7 + pdu_len;
  int sent = ::send(fd, frame, total, 0);
  if (sent != total) {
    ESP_LOGW(TAG, "send() short-write (fd=%d, expected=%d, got=%d), closing", fd, total, sent);
    ::close(fd);
  }
}

void ModbusTcpServerComponent::send_exception_(int fd, uint16_t txid, uint8_t uid, uint8_t fc, uint8_t code) {
  uint8_t pdu[2] = {static_cast<uint8_t>(fc | 0x80), code};
  this->send_response_(fd, txid, uid, pdu, 2);
}

}  // namespace esphome::grid_meter
