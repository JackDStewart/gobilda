#include "gobilda_robot/gobilda_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <deque>

#include <cstring>
#include <iostream>
#include <fstream>
#include <unistd.h>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"

#include <termios.h>
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

    // changing this to see the output of the cobs encoding (was throwing an error before and blocking)
      try {
        motors_.emplace_back(std::make_unique<Motor>(pwm_chip_numbers_[i], 0));
        RCLCPP_INFO(
          rclcpp::get_logger("GobildaSystemHardware"),
          "Set motor to pwm chip: %d", pwm_chip_numbers_[i]
        );
    } catch (const std::exception& e) {
        RCLCPP_WARN(
          rclcpp::get_logger("GobildaSystemHardware"),
          "Motor on chip %d unavailable (skipping): %s", pwm_chip_numbers_[i], e.what()
        );
        motors_.emplace_back(nullptr);  // keep indexing valid
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

  for (size_t i = 0; i < info_.joints.size(); i++) {
    if (info_.joints[i].name == "left_wheel_joint") left_wheel_idx_ = i;
    else if (info_.joints[i].name == "right_wheel_joint") right_wheel_idx_ = i;
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

  //  -------------------- DEBUGGING ----------------------------
  // for (auto i = 0u; i < hw_positions_.size(); i++) {
  //   // Send a neutral signal on init
  //   success = success & motors_[i]->trySetVelocity(1500);
  //   RCLCPP_DEBUG(rclcpp::get_logger("GobildaSystemHardware"),
  //                 "Motor activated");
  // }
  // -------------------------------------------------------------

  // changed this as well:
  for (auto i = 0u; i < hw_positions_.size(); i++) {
      if (motors_[i] != nullptr) {
          success = success && motors_[i]->trySetVelocity(1500);
      }
  }

  if (!success)
  {
    RCLCPP_ERROR(rclcpp::get_logger("GobildaSystemHardware"),
                "Error while sending init vels. to motors");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // open fd for comms with esp32
  esp_rd_fd = open(ESP_FILE_PATH, O_RDWR | O_NOCTTY | O_NONBLOCK); // rd/wr for safety, no controlling terminal, non-blocking

  if (esp_rd_fd < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("GobildaSystemHardware"),
      "Failed to open serial port %s for reading: %s", ESP_FILE_PATH, strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }
  else {
    RCLCPP_INFO(rclcpp::get_logger("GobildaSystemHardware"),
      "esp fd opened");
  }

  struct termios tty;
  tcgetattr(esp_rd_fd, &tty); // get ptr to current settings
  cfmakeraw(&tty);           // disable all TTY processing
  cfsetspeed(&tty, B115200); // match ESP32 baud rate
  tty.c_cc[VMIN]  = 0;       // 0 min bytes
  tty.c_cc[VTIME] = 0;      // 0 min ms for nonblocking
  tcsetattr(esp_rd_fd, TCSANOW, &tty); //apply new settings

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
  RCLCPP_INFO(rclcpp::get_logger("GobildaSystemHardware"), "Lost packets: %d", lost_packets);
  
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
  
  static bool first = true; // drop first partial packet
  
  // reading from the esp connected file descriptor
  uint8_t tmp[MAX_BUF];
  ssize_t n = ::read(esp_rd_fd, tmp, sizeof(tmp));
  if (n < 0) {
    RCLCPP_ERROR(logger, "Error reading bytes: %s", strerror(errno));
    return hardware_interface::return_type::ERROR;
  }
  if (n == 0) {
    RCLCPP_INFO(logger, "nothing to read 😭");
    return hardware_interface::return_type::OK;
  }
  
  //else all good, inset to end of buffer
  read_buf.insert(read_buf.end(), tmp, tmp + n);
  

  //------------------------ debug print the buffer contents in hex and ascii -------------------------
  // RCLCPP_INFO(logger, "%zu bytes in buffer", read_buf.size());
  // for (ssize_t i = 0; i < n; i++) {
  //   // print ascii and hex of each byte read
  //   RCLCPP_INFO(logger, "  byte[%zu] = 0x%02x '%c'", i, tmp[i], (isprint(tmp[i]) ? tmp[i] : '.'));
  // }
  // --------------------------------------------------------------------------------------------------

  //only called on first invocation
  while (first && !read_buf.empty()) {
    if (!(read_buf.front() | 0x00)) {  
      first = false;
      RCLCPP_INFO(rclcpp::get_logger("GobildaSystemHardware"),
        "found first 0x00 dropped partial packet");
    }
    read_buf.erase(read_buf.begin());
    RCLCPP_INFO(rclcpp::get_logger("GobildaSystemHardware"),
    "deleted begining of buffer...%zu bytes in buffer", read_buf.size());
  }
  if (first){
    RCLCPP_INFO(rclcpp::get_logger("GobildaSystemHardware"),
    "Did not drop packets yet...%zu bytes in buffer", read_buf.size());
  }

  // vars for writing to packet array
  uint8_t encoded_frame[PACKET_ENCODED_SIZE];
  bool found_packet;
  size_t r = 0;
  
  // parsing data
  while (r < read_buf.size()) {
    if (!read_buf[r]) { // 0x00 byte
      if (r == 0){
        // if first byte is 0, want to empty the frame, skip this zero byte and continue
        read_buf.erase(read_buf.begin());
        // restart the search (don't break)
        r = 0;
        continue;
      }
      // parse packet between 0 and r
      memcpy(encoded_frame, read_buf.data() + (r - PACKET_ENCODED_SIZE), std::min(r, PACKET_ENCODED_SIZE)); // copy packet to temp buffer for decoding, cap at max packet size
      //erase frames between l and r (inclusive)
      read_buf.erase(read_buf.begin(), read_buf.begin() + r + 1);
      found_packet = true;
      // RCLCPP_INFO(logger, "copied frame of %zu bytes", r);
      break;
    }
    r++;
  }

  // we didn't find a full packet yet, wait for more data
  if (!found_packet) {
    return hardware_interface::return_type::OK;
  }

  // ------------------------------------------ debugging -------------------------------------------
  // RCLCPP_INFO(logger, "Frame size r=%zu, encoded_frame size=%zu", r, sizeof(encoded_frame));
  // for (size_t i = 0; i < r; i++) {
  //   RCLCPP_INFO(logger, "  byte[%zu] = 0x%02x", i, encoded_frame[i]);
  // }
  // --------------------------------------------------------------------------------------------------
  
  // checking the different cases for the packet (mismatch) 
  if (r != sizeof(encoded_frame)) {
    RCLCPP_ERROR(logger, "Frame size mismatch: got %zu bytes, expected %zu bytes", r, sizeof(encoded_frame));
    return hardware_interface::return_type::OK;
  }

  // decoding the result array into the data_packet_t struct
  data_packet_t packet;
  cobs_decode_result res = cobs_decode(&packet, sizeof(data_packet_t), encoded_frame, sizeof(encoded_frame)); // decode into struct
  
  // checking for errors
  if (res.status != COBS_DECODE_OK) {
    RCLCPP_ERROR(logger, "COBS decode error: %x", res.status);
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

  // compare checksums
  if (calc_checksum != true_checksum) {
    RCLCPP_INFO(logger, "Checksum mismatch: got 0x%04x, expected 0x%04x", calc_checksum, true_checksum);
    return hardware_interface::return_type::ERROR;
  }

  // change in time since last packet, default to 20ms (50Hz) if we don't have a valid last timestamp
  static double last_timestamp = 0;
  double dt = (last_timestamp != 0 ? ((packet.encoder_data.timestamp_ms - last_timestamp) / 1000.0) : 0.02); // convert ms to seconds
  
  //calculate velocity from tick deltas
  // ticks / ticks per rotation * 2pi * 50Hz = radians per second
  double l_rps = -((double) packet.encoder_data.dL / Motor::TICKS_PER_ROTATION) * (2 * M_PI) / dt;
  double r_rps = ((double) packet.encoder_data.dR / Motor::TICKS_PER_ROTATION) * (2 * M_PI) / dt;

  //add to hardware velocity variables
  hw_velocities_[left_wheel_idx_] = l_rps;
  hw_velocities_[right_wheel_idx_] = r_rps;

  // timestamp code
  last_timestamp = packet.encoder_data.timestamp_ms;
  static uint8_t prev_seq;
  if(!first && (uint8_t)(prev_seq + 1) != packet.seq) { // typecase to wraparound
    RCLCPP_INFO(logger, "skipped seq - previous %u, current%u", prev_seq, packet.seq);
    lost_packets++;
  }
  prev_seq = packet.seq;

  // ---------------------------------------------------------------------- DEBUGGING (can comment out)---------------------------------------------------------------------------
  // RCLCPP_INFO(logger, "seq=%u dL=%d dR=%d t=%u checksum=0x%04x", packet.seq, packet.encoder_data.dL, packet.encoder_data.dR, packet.encoder_data.timestamp_ms, true_checksum);
  // RCLCPP_INFO(logger, "seq=%u dL=%d dR=%d t=%u checksum=0x%04x", packet.seq, packet.encoder_data.dL, packet.encoder_data.dR, packet.encoder_data.timestamp_ms, true_checksum);
  // RCLCPP_INFO(logger, "\nLeft rads/sec: %.2lf\nRight rads/sec: %.2lf", l_rps, r_rps);
  // RCLCPP_INFO(logger, "\nLeft meter/sec: %.2lf\nRight meter/sec: %.2lf", l_rps * Motor::WHEEL_RADIUS, r_rps * Motor::WHEEL_RADIUS);
  // -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type gobilda_robot::GobildaSystemHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &) 
{
  rclcpp::Logger logger = rclcpp::get_logger("GobildaSystemHardware");

  // to keep track of sequence number
  static uint8_t seq = 0;

  // --------------------- debug print hw_commands -----------------------
  // for (size_t i = 0; i < hw_commands_.size(); i++) {
  //   RCLCPP_INFO(logger, "hw_commands_[%zu] = %f", i, hw_commands_[i]);
  // }
  // ---------------------------------------------------------------------
  
  // writing the target speeds along with other struct memebers to target speed packet
  target_speed_packet_t tsp = {};
  tsp.target_left_rads = (float) hw_commands_[left_wheel_idx_] * -1.0;
  tsp.target_right_rads = (float) hw_commands_[right_wheel_idx_];
  tsp.seq = seq++;
  tsp.checksum = 0;
  uint16_t checksum = calculate_checksum(&tsp, sizeof(target_speed_packet_t));
  tsp.checksum = checksum;

  // done building the packet, need to encode with COBS now
  uint8_t packet_buf[sizeof(target_speed_packet_t) + 2]; // +2 for COBS overhead + zero terminator
  cobs_encode_result res = cobs_encode(packet_buf, sizeof(packet_buf) - 1, &tsp, sizeof(target_speed_packet_t)); // encode 
  if (res.status != COBS_ENCODE_OK) {
    RCLCPP_ERROR(logger, "COBS encode error: %x", res.status);
    return hardware_interface::return_type::ERROR;
  }

  // manually append zero terminator
  packet_buf[res.out_len] = 0x00; 
  
  // write to fd
  ssize_t written = ::write(esp_rd_fd, packet_buf, res.out_len + 1);
  if (written < 0) {
    RCLCPP_ERROR(logger, "Write failed: %s", strerror(errno));
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