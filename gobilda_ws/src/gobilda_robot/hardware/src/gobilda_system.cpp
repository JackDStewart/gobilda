#include "gobilda_robot/gobilda_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include <cstring>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"

extern "C" {
#include "cobs.h"
}


namespace gobilda_robot
{
hardware_interface::CallbackReturn GobildaSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  else
  {
    RCLCPP_INFO(
      rclcpp::get_logger("GobildaSystemHardware"),
      "Success on init!"
    );
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (auto i = 0u; i < info_.joints.size(); i++) {
     try {
        motors_.emplace_back(std::make_unique<Motor>(pwm_chip_numbers_[i], 0));
        RCLCPP_INFO(
          rclcpp::get_logger("GobildaSystemHardware"),
          "Set motor to pwm chip: %d", pwm_chip_numbers_[i] 
        );
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
          rclcpp::get_logger("GobildaSystemHardware"),
          "Failed to create motor on chip %d: %s", pwm_chip_numbers_[i], e.what()
        );
        return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("GobildaSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("GobildaSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("GobildaSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("GobildaSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("GobildaSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GobildaSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GobildaSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn GobildaSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GobildaSystemHardware"), "Activating ...please wait...");
  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  { 
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  bool success = true;

  for (auto i = 0u; i < hw_positions_.size(); i++) {
    // Send a neutral signal on init
    success = success & motors_[i]->trySetVelocity(1500);
    RCLCPP_DEBUG(rclcpp::get_logger("GobildaSystemHardware"),
                  "Motor activated");
  }

  if (!success)
  {
    RCLCPP_ERROR(rclcpp::get_logger("GobildaSystemHardware"),
                "Error while sending init vels. to motors");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // open fd for comms with esp32
  esp_rd_fd = open(ESP_FILE_PATH, O_RDWR | O_NOCTTY | O_NONBLOCK); // rd/wr for safety, no controlling terminal, non-blocking
  // std::ofstream file(ESP_FILE_PATH);
  if (esp_rd_fd < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("GobildaSystemHardware"),
      "Failed to open serial port /dev/ttyACM0 for reading: %s", strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(rclcpp::get_logger("GobildaSystemHardware"),
    "USB tty activated");

  RCLCPP_INFO(rclcpp::get_logger("GobildaSystemHardware"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GobildaSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GobildaSystemHardware"), "Deactivating ...please wait...");
  
  bool success = true;
  
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    // Sned neutral signal on de-activate!
    success = success && motors_[i]->trySetVelocity(1500);
  }
  // Add the gpioTerminateFunction to release the memory!!

  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("GobildaSystemHardware"),
                 "Error setting velocity on motors while deactivating!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("GobildaSystemHardware"),
              "Successfully deactivated!");

  close(esp_rd_fd); // Close the file descriptor when deactivating
  esp_rd_fd = -1;
  // file.close();
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GobildaSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // This 'read' function should receive information from encoder sensors and store in hw_velocities
  
  rclcpp::Logger logger = rclcpp::get_logger("GobildaSystemHardware");
  static rclcpp::Clock clock(RCL_STEADY_TIME);

  // check if fd is still open:
  if (esp_rd_fd < 0){
    RCLCPP_ERROR(logger, "ACM0 port not open (esp_rd_fd < 0)");
    return hardware_interface::return_type::ERROR;
  }

  char read_buf[sizeof(data_packet_t)]; 
  while (true) { //align to 0x00 COBS delimiter
    char byte;
    ssize_t result = ::read(esp_rd_fd, &byte, 1);
    if (result < 0) {
      RCLCPP_ERROR(logger, "Error reading serial while aligning: %s", strerror(errno));
      return hardware_interface::return_type::ERROR;
    }

    if (byte == 0x00) {
      break; // Found the packet delimiter
    }
  }

  size_t bytes_read = 0;
  ssize_t n;
  while (bytes_read < sizeof(read_buf)) { // read until we have a full packet
    n = ::read(esp_rd_fd, read_buf + bytes_read, sizeof(read_buf) - bytes_read);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK){
        RCLCPP_DEBUG_THROTTLE(logger, clock, 1000, "No bytes available (EAGAIN) while reading packet");
        return hardware_interface::return_type::OK;
      }
      RCLCPP_ERROR(logger, "Error reading serial: %s", strerror(errno));
      return hardware_interface::return_type::ERROR;
    }
    if (n == 0) {
      RCLCPP_DEBUG_THROTTLE(logger, clock, 1000, "No bytes read (EOF) while reading packet");
      return hardware_interface::return_type::OK;
    }
    bytes_read += n;
  }

  data_packet_t packet;
  cobs_decode_result res = cobs_decode(&packet, sizeof(data_packet_t), read_buf, sizeof(read_buf)); // decode into struct
  if (res.status != COBS_DECODE_OK) {
    RCLCPP_ERROR(logger, "COBS decode error: %d", res.status);
    return hardware_interface::return_type::ERROR;
  }
  if (res.out_len != sizeof(data_packet_t)) {
    RCLCPP_ERROR(logger, "COBS decode length mismatch: got %zu, expected %zu", res.out_len, sizeof(data_packet_t));
    return hardware_interface::return_type::ERROR;
  }

  // validate checksum
  uint16_t true_checksum = packet.checksum;
  packet.checksum = 0; // zero out checksum field for calculation
  uint16_t calc_checksum = calculate_checksum(&packet, sizeof(data_packet_t));

  if (calc_checksum != true_checksum) {
    RCLCPP_ERROR(logger, "Checksum mismatch: got 0x%04x, expected 0x%04x", calc_checksum, true_checksum);
    return hardware_interface::return_type::ERROR;
  }

  static double last_timestamp = 0;
  // change in time since last packet, default to 20ms (50Hz) if we don't have a valid last timestamp
  double dt = (last_timestamp != 0 ? ((packet.encoder_data.timestamp_ms - last_timestamp) / 1000.0) : 0.02); // convert ms to seconds
  
  //calculate velocity from deltas
  // ticks / ticks per rotation * 2pi * 50Hz = radians per second
  double l_rps = packet.encoder_data.dL / Motor::TICKS_PER_ROTATION * (2 * M_PI) / dt;
  double r_rps = packet.encoder_data.dR / Motor::TICKS_PER_ROTATION * (2 * M_PI) / dt;

  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    const bool is_left = (info_.joints[i].name == "left_wheel_joint");
    hw_velocities_[i] = (is_left ? l_rps : r_rps);
  }

  last_timestamp = packet.encoder_data.timestamp_ms;

  RCLCPP_INFO_THROTTLE(logger, clock, 100, "seq=%u dL=%d dR=%d t=%u checksum=0x%04x", packet.seq, packet.encoder_data.dL, packet.encoder_data.dR, packet.encoder_data.timestamp_ms, true_checksum);
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type gobilda_robot::GobildaSystemHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &) 
{
  bool success = true;

  // Map wheel angular velocity (rad/s) to PWM µs
  auto map_vel_to_us = [&](double u_rad_s) -> int {
    // Handle deadband - zero output within deadband
    if (std::fabs(u_rad_s) < cmd_deadband_rad_s)
        return static_cast<int>(std::llround(neutral_us));

    const bool fwd = (u_rad_s > 0.0);
    
    // Remove deadband and scale the remaining velocity
    double mag = std::fabs(u_rad_s) - cmd_deadband_rad_s;
    
    // Use your configured base and gain values (don't override!)
    double base = fwd ? deadband_fwd_us : deadband_rev_us;
    double gain = fwd ? gain_fwd_us_per_rads : gain_rev_us_per_rads;

    // Calculate pulse width using actual velocity magnitude
    double pulse = neutral_us + std::copysign(base + gain * mag, u_rad_s);

    // Directional caps to enforce your chosen "full" values'
    // Hardware guard-rails
    if (fwd) {
        pulse = std::min(pulse, top_fwd_us);
    } else {
        pulse = std::max(pulse, top_rev_us);
    }

    return static_cast<int>(std::llround(pulse));
  };

  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    // One sign flip for the left side so +u means forward robot motion
    const bool is_left = (info_.joints[i].name == "left_wheel_joint");
    const double u_rad_s = hw_commands_[i] * (is_left ? -1.0 : 1.0);

    const int pulse_i = map_vel_to_us(u_rad_s);

    // Optional: throttle log to see mapping/clamping
    RCLCPP_DEBUG(
      rclcpp::get_logger("GobildaSystemHardware"),
      "[%s] u=%.3f rad/s -> %d us",
      info_.joints[i].name.c_str(), u_rad_s, pulse_i
    );

    success = success && motors_[i]->trySetVelocity(pulse_i);
  }

  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("GobildaSystemHardware"),
                 "Error setting velocity on motors during write step!");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace gobilda_robot

//helper checksum func
uint16_t calculate_checksum(const void *data, size_t len) { // stolen from schmitt if we have any issues 
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += bytes[i];
    }
    // Fold 32-bit sum into 16 bits
    while (sum >> 16) {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return (uint16_t)~sum;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  gobilda_robot::GobildaSystemHardware,
  hardware_interface::SystemInterface
)
