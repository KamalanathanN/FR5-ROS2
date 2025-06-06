#ifndef _FR_HARDWARE_INTERFACE_
#define _FR_HARDWARE_INTERFACE_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "visibility_control.h"
#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <algorithm>
#include "libfairino/include/robot.h"


#define CONTROLLER_IP_ADDRESS "192.168.58.2"

namespace fairino_hardware
{

class FairinoHardwareInterface: public hardware_interface::SystemInterface{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  //FAIRINO_HARDWARE_PUBLIC
  //hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
  FAIRINO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;
  // hardware_interface::return_type perform_command_mode_switch(
  //   const std::vector<std::string>& start_interfaces,
  //   const std::vector<std::string>& stop_interfaces) override;

  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  FAIRINO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
private:
  // Arm joint variables
  double _jnt_position_command[6];
  double _jnt_velocity_command[6];
  double _jnt_torque_command[6];
  double _jnt_position_state[6];
  double _jnt_velocity_state[6];
  double _jnt_torque_state[6];
  int _control_mode;
  std::string _controller_ip = CONTROLLER_IP_ADDRESS;
  std::unique_ptr<FRRobot> _ptr_robot;

  // Gripper-related variables
  bool _has_gripper;
  std::vector<std::string> _gripper_joint_names;
  std::vector<double> _gripper_position_command;
  std::vector<double> _gripper_velocity_command;
  std::vector<double> _gripper_effort_command;
  std::vector<double> _gripper_position_state;
  std::vector<double> _gripper_velocity_state;
  std::vector<double> _gripper_effort_state;
  
  // Gripper configuration parameters
  int _gripper_company;
  int _gripper_device;
  int _gripper_softversion;
  int _gripper_bus;
  int _gripper_index;
  int _gripper_max_time;
  
  // Gripper control parameters
  double _gripper_position_tolerance;
  double _gripper_max_velocity;
  double _gripper_max_effort;
  
  // Gripper status
  bool _gripper_active;
  uint8_t _gripper_position;
  int8_t _gripper_current;
  int _gripper_voltage;
  int _gripper_temperature;
  int8_t _gripper_speed;
  uint16_t _gripper_fault;
  
  // Timing
  std::chrono::steady_clock::time_point _last_read_time;
  std::chrono::steady_clock::time_point _last_write_time;

  // Private gripper methods
  bool configure_gripper();
  bool activate_gripper(bool activate);
  bool move_gripper_to_position(double position, double velocity = 50.0, double effort = 50.0);
  bool read_gripper_status();
  double gripper_units_to_meters(uint8_t gripper_units);
  uint8_t meters_to_gripper_units(double meters);
  bool is_gripper_motion_done();
};

} //end namespace


#endif