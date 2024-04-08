#pragma once

#include <string>
#include <math.h>
#include <spdlog/spdlog.h>

#include "vulp/actuation/ServoLayout.h"

#define CHIP_FREQ 19200000
#define SERVO_MODE_VEL 0
#define SERVO_MODE_POS 1
#define SERVO_MODE_TOR 2

namespace vulp::actuation::wiringpi {

using namespace std;

class ServoConfig{
public:
    const int id_;
    const int mode_;
    const string name_;
    const int bus_;
    const int pin_;
    const int ctl_a_pin_;
    const int ctl_b_pin_;
    const int standby_pin_;
    const int clock_;
    const int chip_freq_hz_;
    const int servo_freq_hz_;
    const int range_;
    const float min_pw_ms_;
    const float max_pw_ms_;
    const int min_pwm_;
    const int max_pwm_;
    const int pwm_range_;
    const int pwm_centroid_;
    const int min_degree_;
    const int max_degree_;
    const int degree_range_;
    const bool hall_;
    const int hall_a_;
    const int hall_b_;
    const float hall_resolution_;

    explicit ServoConfig(int id, int mode, string name, int bus, int pin, int ctl_a_pin, int ctl_b_pin, int standby_pin, int chip_freq_hz, int clock_div, int range, float min_pw_ms, float max_pw_ms, int min_degree, int max_degree, int freq_serv_hz, bool hall, int hall_a, int hall_b, float hall_resolution):
        id_(id),
        mode_(mode),
        name_(name),
        bus_(bus),
        pin_(pin),
        ctl_a_pin_(ctl_a_pin),
        ctl_b_pin_(ctl_b_pin),
        standby_pin_(standby_pin),
        clock_(clock_div),
        chip_freq_hz_(chip_freq_hz),
        servo_freq_hz_(freq_serv_hz),
        range_((float)chip_freq_hz / (float)clock_div / (float)freq_serv_hz),
        min_pw_ms_(min_pw_ms),
        max_pw_ms_(max_pw_ms),
        min_pwm_(min_pw_ms_ * (float)chip_freq_hz_ / (float)clock_ / 1000.0f > 0.0f ? min_pw_ms_ * (float)chip_freq_hz_ / (float)clock_ / 1000.0f : 0.0f),
        max_pwm_(max_pw_ms_ * (float) chip_freq_hz_ / (float) clock_ / 1000.0f < (float) range_ ? max_pw_ms_ * (float) chip_freq_hz_ / (float) clock_ / 1000.0f : (float)range_),
        pwm_range_(max_pwm_ - min_pwm_),
        pwm_centroid_((int)((float)(max_pwm_ + min_pwm_) / 2)),
        min_degree_(min_degree),
        max_degree_(max_degree),
        degree_range_(max_degree - min_degree),
        hall_(hall),
        hall_a_(hall_a),
        hall_b_(hall_b),
        hall_resolution_(hall_resolution){
        };
    ~ServoConfig() = default;

    int getPwmValueFromDegree(int degree);
};

class WiringpiConfig{
private:
    std::map<string, ServoConfig*> servo_configs_;
    const int pwm_clock_div_;
    const int pwm_range_;
    const int servo_delay_;
    vulp::actuation::ServoLayout servo_layout_;
public:
    WiringpiConfig(int pwm_clock_div, int pwm_range, int servo_delay):
        pwm_clock_div_(pwm_clock_div), 
        pwm_range_(pwm_range), 
        servo_delay_(servo_delay){};

    ~WiringpiConfig() = default;

    void addServo(ServoConfig* servo){
        servo_configs_.emplace(std::make_pair(servo->name_, servo));
        servo_layout_.add_servo(servo->pin_, servo->bus_, servo->name_);
    };

    map<string, ServoConfig*> getServoConfigs() const{
        return servo_configs_;
    };

    int getPwmClockDiv() const{
        return pwm_clock_div_;
    }

    int getPwmRange() const{
        return pwm_range_;
    }

    int getServoDelay() const{
        return servo_delay_;
    }

    vulp::actuation::ServoLayout servoLayout() const noexcept {
        return servo_layout_;
    }

};

}