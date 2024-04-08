#pragma once

#include <string>

namespace vulp::actuation::wiringpi {

using namespace std;

struct JointState{
    string id;
    float position_rad;
    float velocity_rad_per_sec;
    float torque_n_m;
};


}