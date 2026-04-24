#include "ros2_modbus/modbus_bridge.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace ros2_modbus
{

namespace
{
constexpr int kConnectTimeoutSec = 3;
constexpr std::size_t kMbapHeaderSize = 7;

std::string errno_str()
{
  return std::string(std::strerror(errno));
}

// Blocking send-all; returns true only if exactly `len` bytes were sent.
bool send_all(int fd, const uint8_t * buf, std::size_t len)
{
  std::size_t sent = 0;
  while (sent < len) {
    ssize_t n = ::send(fd, buf + sent, len - sent, 0);
    if (n <= 0) {
      return false;
    }
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

// Blocking recv-all; returns true only if exactly `len` bytes were received.
bool recv_all(int fd, uint8_t * buf, std::size_t len)
{
  std::size_t got = 0;
  while (got < len) {
    ssize_t n = ::recv(fd, buf + got, len - got, 0);
    if (n <= 0) {
      return false;
    }
    got += static_cast<std::size_t>(n);
  }
  return true;
}
}  // namespace

ModbusBridge::ModbusBridge()
: rclcpp::Node("modbus_bridge")
{
  plc_ip_           = this->declare_parameter<std::string>("plc_ip", "192.168.1.88");
  plc_port_         = this->declare_parameter<int>("plc_port", 502);
  slave_id_         = this->declare_parameter<int>("slave_id", 0);
  poll_rate_hz_     = this->declare_parameter<double>("poll_rate_hz", 1.0);
  poll_reg_address_ = this->declare_parameter<int>("poll_reg_address", 0);
  poll_reg_count_   = this->declare_parameter<int>("poll_reg_count", 10);
  poll_coil_address_ = this->declare_parameter<int>("poll_coil_address", 0);
  poll_coil_count_  = this->declare_parameter<int>("poll_coil_count", 8);

  reg_pub_  = this->create_publisher<std_msgs::msg::UInt16MultiArray>("/modbus/registers", 10);
  coil_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/modbus/coils", 10);

  read_reg_srv_ = this->create_service<srv::ReadRegisters>(
    "/modbus/read_registers",
    std::bind(&ModbusBridge::handle_read_registers, this,
      std::placeholders::_1, std::placeholders::_2));
  write_reg_srv_ = this->create_service<srv::WriteRegister>(
    "/modbus/write_register",
    std::bind(&ModbusBridge::handle_write_register, this,
      std::placeholders::_1, std::placeholders::_2));
  read_coil_srv_ = this->create_service<srv::ReadCoils>(
    "/modbus/read_coils",
    std::bind(&ModbusBridge::handle_read_coils, this,
      std::placeholders::_1, std::placeholders::_2));
  write_coil_srv_ = this->create_service<srv::WriteCoil>(
    "/modbus/write_coil",
    std::bind(&ModbusBridge::handle_write_coil, this,
      std::placeholders::_1, std::placeholders::_2));

  const double hz = (poll_rate_hz_ > 0.0) ? poll_rate_hz_ : 1.0;
  const auto period = std::chrono::duration<double>(1.0 / hz);
  poll_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ModbusBridge::on_poll_timer, this));

  RCLCPP_INFO(this->get_logger(),
    "modbus_bridge started: %s:%d unit=%d poll=%.2fHz regs[%d+%d] coils[%d+%d]",
    plc_ip_.c_str(), plc_port_, slave_id_, poll_rate_hz_,
    poll_reg_address_, poll_reg_count_,
    poll_coil_address_, poll_coil_count_);
}

ModbusBridge::~ModbusBridge()
{
  std::lock_guard<std::mutex> lk(sock_mutex_);
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
}

bool ModbusBridge::connect_socket(std::string & err)
{
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }

  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    err = "socket(): " + errno_str();
    return false;
  }

  struct timeval tv;
  tv.tv_sec = kConnectTimeoutSec;
  tv.tv_usec = 0;
  if (::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0 ||
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
  {
    err = "setsockopt(SO_*TIMEO): " + errno_str();
    ::close(fd);
    return false;
  }

  struct sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(plc_port_));
  if (::inet_pton(AF_INET, plc_ip_.c_str(), &addr.sin_addr) != 1) {
    err = "inet_pton(" + plc_ip_ + "): invalid address";
    ::close(fd);
    return false;
  }

  if (::connect(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    err = "connect(" + plc_ip_ + ":" + std::to_string(plc_port_) + "): " + errno_str();
    ::close(fd);
    return false;
  }

  sock_fd_ = fd;
  return true;
}

void ModbusBridge::disconnect_socket()
{
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
}

bool ModbusBridge::ensure_connected(std::string & err)
{
  if (sock_fd_ >= 0) {
    return true;
  }
  return connect_socket(err);
}

// `pdu` is the full PDU including the function-code byte as its first byte.
// `resp_pdu` on success contains the response PDU including FC byte.
bool ModbusBridge::transact(
  uint8_t fc,
  const std::vector<uint8_t> & pdu,
  std::vector<uint8_t> & resp_pdu,
  std::string & err)
{
  std::lock_guard<std::mutex> lk(sock_mutex_);

  if (!ensure_connected(err)) {
    return false;
  }

  const uint16_t tx_id = transaction_id_++;  // uint16 wraps naturally
  const uint16_t length = static_cast<uint16_t>(1 + pdu.size());  // unit_id + pdu

  std::vector<uint8_t> frame;
  frame.reserve(kMbapHeaderSize + pdu.size());
  frame.push_back(static_cast<uint8_t>((tx_id >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(tx_id & 0xFF));
  frame.push_back(0x00);  // protocol id hi
  frame.push_back(0x00);  // protocol id lo
  frame.push_back(static_cast<uint8_t>((length >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>(length & 0xFF));
  frame.push_back(static_cast<uint8_t>(slave_id_ & 0xFF));
  frame.insert(frame.end(), pdu.begin(), pdu.end());

  if (!send_all(sock_fd_, frame.data(), frame.size())) {
    err = "send(): " + errno_str();
    disconnect_socket();
    return false;
  }

  uint8_t header[kMbapHeaderSize];
  if (!recv_all(sock_fd_, header, kMbapHeaderSize)) {
    err = "recv(mbap header): " + errno_str();
    disconnect_socket();
    return false;
  }

  const uint16_t resp_tx =
    static_cast<uint16_t>((header[0] << 8) | header[1]);
  const uint16_t resp_proto =
    static_cast<uint16_t>((header[2] << 8) | header[3]);
  const uint16_t resp_len =
    static_cast<uint16_t>((header[4] << 8) | header[5]);
  // header[6] = unit id, ignored for validation

  if (resp_tx != tx_id) {
    err = "mbap tx_id mismatch: sent " + std::to_string(tx_id) +
      " got " + std::to_string(resp_tx);
    disconnect_socket();
    return false;
  }
  if (resp_proto != 0) {
    err = "mbap protocol_id != 0";
    disconnect_socket();
    return false;
  }
  if (resp_len < 2 || resp_len > 260) {
    err = "mbap length out of range: " + std::to_string(resp_len);
    disconnect_socket();
    return false;
  }

  const std::size_t pdu_len = static_cast<std::size_t>(resp_len) - 1;  // minus unit_id
  resp_pdu.assign(pdu_len, 0);
  if (!recv_all(sock_fd_, resp_pdu.data(), pdu_len)) {
    err = "recv(pdu): " + errno_str();
    disconnect_socket();
    return false;
  }

  const uint8_t resp_fc = resp_pdu[0];
  if (resp_fc == (fc | 0x80)) {
    const uint8_t ex = (resp_pdu.size() >= 2) ? resp_pdu[1] : 0;
    err = "modbus exception fc=0x" + std::to_string(fc) +
      " code=" + std::to_string(static_cast<int>(ex));
    // exception responses do not corrupt the stream; keep socket open
    return false;
  }
  if (resp_fc != fc) {
    err = "function code echo mismatch: sent " + std::to_string(fc) +
      " got " + std::to_string(resp_fc);
    disconnect_socket();
    return false;
  }

  return true;
}

bool ModbusBridge::read_holding_registers(
  uint16_t addr, uint16_t count,
  std::vector<uint16_t> & out, std::string & err)
{
  if (count == 0 || count > 125) {
    err = "read_holding_registers: count out of range (1..125)";
    return false;
  }

  std::vector<uint8_t> pdu = {
    0x03,
    static_cast<uint8_t>((addr >> 8) & 0xFF),
    static_cast<uint8_t>(addr & 0xFF),
    static_cast<uint8_t>((count >> 8) & 0xFF),
    static_cast<uint8_t>(count & 0xFF),
  };

  std::vector<uint8_t> resp;
  if (!transact(0x03, pdu, resp, err)) {
    return false;
  }

  // resp: [fc=03, byte_count, N*2 data]
  if (resp.size() < 2) {
    err = "fc03 response truncated";
    return false;
  }
  const uint8_t byte_count = resp[1];
  if (byte_count != count * 2 || resp.size() < 2u + byte_count) {
    err = "fc03 byte_count mismatch";
    return false;
  }
  out.resize(count);
  for (uint16_t i = 0; i < count; ++i) {
    const std::size_t off = 2 + i * 2;
    out[i] = static_cast<uint16_t>((resp[off] << 8) | resp[off + 1]);
  }
  return true;
}

bool ModbusBridge::write_single_register(
  uint16_t addr, uint16_t value, std::string & err)
{
  std::vector<uint8_t> pdu = {
    0x06,
    static_cast<uint8_t>((addr >> 8) & 0xFF),
    static_cast<uint8_t>(addr & 0xFF),
    static_cast<uint8_t>((value >> 8) & 0xFF),
    static_cast<uint8_t>(value & 0xFF),
  };

  std::vector<uint8_t> resp;
  if (!transact(0x06, pdu, resp, err)) {
    return false;
  }
  if (resp.size() != 5) {
    err = "fc06 response unexpected size";
    return false;
  }
  return true;
}

bool ModbusBridge::read_coils(
  uint16_t addr, uint16_t count,
  std::vector<uint8_t> & out, std::string & err)
{
  if (count == 0 || count > 2000) {
    err = "read_coils: count out of range (1..2000)";
    return false;
  }

  std::vector<uint8_t> pdu = {
    0x01,
    static_cast<uint8_t>((addr >> 8) & 0xFF),
    static_cast<uint8_t>(addr & 0xFF),
    static_cast<uint8_t>((count >> 8) & 0xFF),
    static_cast<uint8_t>(count & 0xFF),
  };

  std::vector<uint8_t> resp;
  if (!transact(0x01, pdu, resp, err)) {
    return false;
  }

  if (resp.size() < 2) {
    err = "fc01 response truncated";
    return false;
  }
  const uint8_t byte_count = resp[1];
  const uint8_t expected_bytes =
    static_cast<uint8_t>((count + 7) / 8);
  if (byte_count != expected_bytes || resp.size() < 2u + byte_count) {
    err = "fc01 byte_count mismatch";
    return false;
  }
  out.resize(count);
  for (uint16_t i = 0; i < count; ++i) {
    const uint8_t byte = resp[2 + (i / 8)];
    out[i] = static_cast<uint8_t>((byte >> (i % 8)) & 0x01);
  }
  return true;
}

bool ModbusBridge::write_single_coil(
  uint16_t addr, bool value, std::string & err)
{
  const uint16_t v = value ? 0xFF00 : 0x0000;
  std::vector<uint8_t> pdu = {
    0x05,
    static_cast<uint8_t>((addr >> 8) & 0xFF),
    static_cast<uint8_t>(addr & 0xFF),
    static_cast<uint8_t>((v >> 8) & 0xFF),
    static_cast<uint8_t>(v & 0xFF),
  };

  std::vector<uint8_t> resp;
  if (!transact(0x05, pdu, resp, err)) {
    return false;
  }
  if (resp.size() != 5) {
    err = "fc05 response unexpected size";
    return false;
  }
  return true;
}

void ModbusBridge::on_poll_timer()
{
  std::string err;

  if (poll_reg_count_ > 0) {
    std::vector<uint16_t> regs;
    if (read_holding_registers(
        static_cast<uint16_t>(poll_reg_address_),
        static_cast<uint16_t>(poll_reg_count_),
        regs, err))
    {
      std_msgs::msg::UInt16MultiArray msg;
      msg.layout.dim.resize(1);
      msg.layout.dim[0].label = "registers";
      msg.layout.dim[0].size = static_cast<uint32_t>(regs.size());
      msg.layout.dim[0].stride = static_cast<uint32_t>(regs.size());
      msg.data = regs;
      reg_pub_->publish(msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "poll registers failed: %s", err.c_str());
    }
  }

  if (poll_coil_count_ > 0) {
    std::vector<uint8_t> coils;
    if (read_coils(
        static_cast<uint16_t>(poll_coil_address_),
        static_cast<uint16_t>(poll_coil_count_),
        coils, err))
    {
      std_msgs::msg::UInt8MultiArray msg;
      msg.layout.dim.resize(1);
      msg.layout.dim[0].label = "coils";
      msg.layout.dim[0].size = static_cast<uint32_t>(coils.size());
      msg.layout.dim[0].stride = static_cast<uint32_t>(coils.size());
      msg.data = coils;
      coil_pub_->publish(msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "poll coils failed: %s", err.c_str());
    }
  }
}

void ModbusBridge::handle_read_registers(
  const std::shared_ptr<srv::ReadRegisters::Request> req,
  std::shared_ptr<srv::ReadRegisters::Response> res)
{
  std::string err;
  std::vector<uint16_t> values;
  res->success = read_holding_registers(req->address, req->count, values, err);
  if (res->success) {
    res->values = values;
    res->message = "ok";
  } else {
    res->message = err;
  }
}

void ModbusBridge::handle_write_register(
  const std::shared_ptr<srv::WriteRegister::Request> req,
  std::shared_ptr<srv::WriteRegister::Response> res)
{
  std::string err;
  res->success = write_single_register(req->address, req->value, err);
  res->message = res->success ? "ok" : err;
}

void ModbusBridge::handle_read_coils(
  const std::shared_ptr<srv::ReadCoils::Request> req,
  std::shared_ptr<srv::ReadCoils::Response> res)
{
  std::string err;
  std::vector<uint8_t> values;
  res->success = read_coils(req->address, req->count, values, err);
  if (res->success) {
    res->values.resize(values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {
      res->values[i] = (values[i] != 0);
    }
    res->message = "ok";
  } else {
    res->message = err;
  }
}

void ModbusBridge::handle_write_coil(
  const std::shared_ptr<srv::WriteCoil::Request> req,
  std::shared_ptr<srv::WriteCoil::Response> res)
{
  std::string err;
  res->success = write_single_coil(req->address, req->value, err);
  res->message = res->success ? "ok" : err;
}

}  // namespace ros2_modbus

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_modbus::ModbusBridge>());
  rclcpp::shutdown();
  return 0;
}
