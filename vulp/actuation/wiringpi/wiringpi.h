#pragma once

#include <string>
#include <thread>
#include <functional>

#include "wiringPi.h"
#include "wiringpi_config.h"
#include "wiringpi_models.h"
#include "vulp/actuation/ImuData.h"
#include "vulp/actuation/moteus/ServoCommand.h"
#include "atk_imu/atk_ms901m.h"
#include "vulp/actuation/moteus/Span.h"
#include "hall/wiring_hall.h"

#define ATK_IMU_901_BAUD 115200
#define ATK_IMU_901_DEV "/dev/ttyS0"

namespace vulp::actuation::wiringpi {



class WiringpiAdapter{


private:
    const WiringpiConfig& config_;
    AtkImu901* atk_imu_;
    std::map<string, HallAdapter*> hall_adapter_map_;
    const bool dry_run_;
    bool running_;
    thread thread_;
    void run_hall_thread();
    int feed_forward_torque_ = -100;
public:
    WiringpiAdapter(const WiringpiConfig& config, const bool dry_run);

    ~WiringpiAdapter();

    std::vector<JointState> runServoCommands(const ServoLayout& layout, moteus::Span<moteus::ServoCommand> commands);
    
    ImuData getImu();

};

} // namespace vulp::actuation