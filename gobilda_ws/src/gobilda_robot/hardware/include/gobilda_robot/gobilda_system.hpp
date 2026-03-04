#ifndef GOBILDA_ROBOT__GOBILDA_SYSTEM_HPP_
#define GOBILDA_ROBOT__GOBILDA_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "gobilda_robot/motor.hpp"
#include <stdint.h>

namespace gobilda_robot
{

// structs to match esp32 encoder packet format
typedef struct {
  int16_t dL;      // delta left
  int16_t dR;     // delta right
  uint32_t timestamp_ms;  // var for timestamp
} __attribute__((packed)) encoder_data_t;

typedef struct {
  uint8_t seq; // seq num for testing loss
  encoder_data_t encoder_data;
  uint16_t checksum; 
} __attribute__((packed)) data_packet_t;

class GobildaSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GobildaSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<std::unique_ptr<Motor>> motors_; // Change to pointers
  // Code assumes that Right motor is on pin 32
  // and Left motor is on pin 15
  std::vector<int> pwm_chip_numbers_ = {0, 3};

  // Calibrated values 
  // TODO: (consider making ROS params)
  // These values were tuned for a specific Gobilda number
  // They most likely will need to be tuned for every robot/
  // motor controller. NOTE that I used thr 312 rpm motors
  // for these values.
  const double neutral_us           = 1500.0;
  const double deadband_fwd_us      = 40.0;
  const double deadband_rev_us      = 40.0;
  const double gain_fwd_us_per_rads = 27.0;
  const double gain_rev_us_per_rads = 20.0;
  const double cmd_deadband_rad_s   = 0.05;

  #define ESP_FILE_PATH "/dev/ttyACM0"
  // std::ofstream file(ESP_FILE_PATH);
  int esp_rd_fd = -1; // File descriptor for reading from ESP32 (initialized to -1 for safety)
  
  // Hard-cap on how fast the robot can actually move
  const double top_fwd_us           = 1800.0;
  const double top_rev_us           = 1200.0;
};

}  // namespace gobilda_robot

uint16_t calculate_checksum(const void *data, size_t len);

#endif //GOBILDA_ROBOT__GOBILDA_SYSTEM_HPP
