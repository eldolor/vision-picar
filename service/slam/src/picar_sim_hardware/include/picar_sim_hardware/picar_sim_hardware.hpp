// picar_sim_hardware -- PLAN-ros-alignment.md R4.
//
// hardware_interface::SystemInterface is RobotInterface by another name:
// read() pulls state, write() pushes commands, and nothing requires the
// hardware to be real. This one reaches the simulator through the robot
// server's HTTP API -- GET /wheels for state, POST /wheels (x-driver: ros)
// for the command -- so the robot server's safety vet, its wheel loop and
// its watchdog all stay in the path. picar_hardware (R7) is the same class
// against the ESP32; nothing above this seam changes when it is swapped.
#pragma once

#include <string>
#include <vector>

#include <curl/curl.h>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace picar_sim_hardware
{

class PicarSimHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PicarSimHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  ~PicarSimHardware() override;

private:
  bool request(const std::string & method, const std::string & path,
               const std::string & body, std::string & out);
  bool post_wheels(double left, double right);

  std::string robot_url_;
  std::string secret_;
  CURL * curl_ = nullptr;
  // Index 0 = left_wheel_joint, 1 = right_wheel_joint, by name at init.
  double pos_[2] = {0.0, 0.0};
  double vel_[2] = {0.0, 0.0};
  double cmd_[2] = {0.0, 0.0};
  std::string names_[2];
  int consecutive_failures_ = 0;
  rclcpp::Logger logger_ = rclcpp::get_logger("PicarSimHardware");
};

}  // namespace picar_sim_hardware
