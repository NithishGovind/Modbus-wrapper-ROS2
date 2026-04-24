#ifndef ROS2_MODBUS__MODBUS_BRIDGE_HPP_
#define ROS2_MODBUS__MODBUS_BRIDGE_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "ros2_modbus/srv/read_registers.hpp"
#include "ros2_modbus/srv/write_register.hpp"
#include "ros2_modbus/srv/read_coils.hpp"
#include "ros2_modbus/srv/write_coil.hpp"

namespace ros2_modbus
{

class ModbusBridge : public rclcpp::Node
{
public:
  ModbusBridge();
  ~ModbusBridge() override;

private:
  // --- Modbus transport ---
  bool connect_socket(std::string & err);
  void disconnect_socket();
  bool ensure_connected(std::string & err);
  bool transact(
    uint8_t fc,
    const std::vector<uint8_t> & pdu,
    std::vector<uint8_t> & resp_pdu,
    std::string & err);

  // --- Modbus function codes ---
  bool read_holding_registers(
    uint16_t addr, uint16_t count,
    std::vector<uint16_t> & out, std::string & err);
  bool write_single_register(
    uint16_t addr, uint16_t value, std::string & err);
  bool read_coils(
    uint16_t addr, uint16_t count,
    std::vector<uint8_t> & out, std::string & err);
  bool write_single_coil(
    uint16_t addr, bool value, std::string & err);

  // --- ROS callbacks ---
  void on_poll_timer();

  void handle_read_registers(
    const std::shared_ptr<srv::ReadRegisters::Request> req,
    std::shared_ptr<srv::ReadRegisters::Response> res);
  void handle_write_register(
    const std::shared_ptr<srv::WriteRegister::Request> req,
    std::shared_ptr<srv::WriteRegister::Response> res);
  void handle_read_coils(
    const std::shared_ptr<srv::ReadCoils::Request> req,
    std::shared_ptr<srv::ReadCoils::Response> res);
  void handle_write_coil(
    const std::shared_ptr<srv::WriteCoil::Request> req,
    std::shared_ptr<srv::WriteCoil::Response> res);

  // --- State ---
  std::string plc_ip_;
  int plc_port_{502};
  int slave_id_{0};
  double poll_rate_hz_{1.0};
  int poll_reg_address_{0};
  int poll_reg_count_{10};
  int poll_coil_address_{0};
  int poll_coil_count_{8};

  int sock_fd_{-1};
  std::mutex sock_mutex_;
  uint16_t transaction_id_{0};

  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr reg_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr coil_pub_;

  rclcpp::Service<srv::ReadRegisters>::SharedPtr read_reg_srv_;
  rclcpp::Service<srv::WriteRegister>::SharedPtr write_reg_srv_;
  rclcpp::Service<srv::ReadCoils>::SharedPtr read_coil_srv_;
  rclcpp::Service<srv::WriteCoil>::SharedPtr write_coil_srv_;

  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace ros2_modbus

#endif  // ROS2_MODBUS__MODBUS_BRIDGE_HPP_
