#pragma once

#ifndef GOBILDA_ROBOT__MOTOR_HPP_
#define GOBILDA_ROBOT__MOTOR_HPP_

// non_blocking --> motor.hpp
#include <atomic>
#include <thread>
#include <mutex>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>

class Motor {
private:
    std::atomic<int> target_pulse_{1500};
    std::thread pwm_thread_;
    std::atomic<bool> running_{false};
    
    std::string pwm_channel_path_;
    std::string pwm_duty_cycle_path_;
    std::string pwm_chip_path_;
    
    int pwm_chip_number_;
    int pwm_channel_;
    int period_fd_ = -1, duty_fd_ = -1, enable_fd_ = -1;
    // Values grabbed from Gobilda specs!
    int period_ns_ = 20000000, duty_cycle_ns_ = 1500000;
    int min_us_ = 1050, max_us_ = 1950;
        
public:
    static constexpr int TICKS_PER_ROTATION = 538; // Might need to be changed for different motors, check datasheet

    Motor(int pwm_chip, int pwm_channel);
    ~Motor();
    
    bool trySetVelocity(int pulse_width_us); // NON-BLOCKING
    void stop();
    
private:
    void hardwareThread(); // Runs in separate thread
    bool setVelocityHardware(int pulse_width_us); // The actual blocking call
};

#endif  // GOBILDA_ROBOT__MOTOR_HPP