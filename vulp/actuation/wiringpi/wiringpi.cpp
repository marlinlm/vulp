#include <spdlog/spdlog.h>

#include "wiringpi.h"
#include "atk_imu/atk_ms901m.h"
#include "hall/wiring_hall.h"


namespace vulp::actuation::wiringpi {

    using namespace spdlog;

    WiringpiAdapter::WiringpiAdapter(const WiringpiConfig& config, const bool dry_run = false):
        config_(config),
        dry_run_(dry_run),
        running_(true)
    {   
        atk_imu_ = new AtkImu901(ATK_IMU_901_DEV, ATK_IMU_901_BAUD);
        if(!dry_run_){
            wiringPiSetup();
            // pin mode must be set before pwm mode, dont mass with this!!!
            for(auto sConfig : config_.getServoConfigs()){
                pinMode(sConfig.second->pin_, PWM_OUTPUT);

                if(sConfig.second->hall_){
                    spdlog::info("hall_a_pin: {}, hall_b_pin:{}", sConfig.second->hall_a_, sConfig.second->hall_b_);
                    HallAdapter* hall_adapter = new HallAdapter(sConfig.first, sConfig.second->hall_a_, sConfig.second->hall_b_, sConfig.second->hall_resolution_);
                    hall_adapter->init();
                    hall_adapter_map_.emplace(std::make_pair(sConfig.first, hall_adapter));
                }

                if(sConfig.second->ctl_a_pin_ != -1){
                    pinMode(sConfig.second->ctl_a_pin_, OUTPUT);
                }
                if(sConfig.second->ctl_b_pin_ != -1){
                    pinMode(sConfig.second->ctl_b_pin_, OUTPUT);
                }
                if(sConfig.second->standby_pin_ != -1){
                    pinMode(sConfig.second->standby_pin_, OUTPUT);
                    digitalWrite(sConfig.second->standby_pin_, 1);
                }
            }
            pwmSetMode(PWM_MODE_MS);
            pwmSetClock(config_.getPwmClockDiv());
            pwmSetRange(config_.getPwmRange());
        
            thread_ = thread(std::bind(&WiringpiAdapter::run_hall_thread, this));
        }
    }

    WiringpiAdapter::~WiringpiAdapter(){
        running_ = false;
        thread_.join();
        for(auto sConfig : config_.getServoConfigs()){
            pinMode(sConfig.second->pin_, PWM_OUTPUT);
            if(sConfig.second->hall_){
                spdlog::info("hall_a_pin: {}, hall_b_pin:{}", sConfig.second->hall_a_, sConfig.second->hall_b_);
                HallAdapter* hall_adapter = hall_adapter_map_.at(sConfig.first);
                delete hall_adapter;
            }
        }
    }

    std::vector<JointState> WiringpiAdapter::runServoCommands(const ServoLayout& layout, moteus::Span<moteus::ServoCommand> commands){

        std::vector<JointState> joint_states;
        for (size_t i = 0; i < commands.size(); ++i) {
            const auto& command = commands[i];
            const auto servo_id = command.id;
            const std::string& joint_name = layout.joint_name(servo_id);
            
            std::map<std::string, ServoConfig*> config = config_.getServoConfigs();
            ServoConfig* servo = config.at(joint_name);

            atk_ms901m_attitude_data_t* attitude_dat = atk_imu_->atk_ms901m_get_attitude_data();;

            int torque = command.position.feedforward_torque;
            if(attitude_dat->pitch > 60 || attitude_dat->pitch < -60){
                torque = 0;
            }
            int degree = command.position.position * 360;
            int vel = command.position.velocity * 360;
            int ctl_a = 1;
            int ctl_b = 0;
            switch(servo->mode_){
                case SERVO_MODE_POS:
                    if (!dry_run_){
                        pwmWrite(servo->pin_, servo->getPwmValueFromDegree(degree));
                        spdlog::debug("[test running] pin:" + std::to_string(servo->pin_) + ", pwm:" + std::to_string(servo->getPwmValueFromDegree(degree)));
                    }else{
                        spdlog::debug("[Dry running] pin:" + std::to_string(servo->pin_) + ", pwm:" + std::to_string(servo->getPwmValueFromDegree(degree)));
                    }
                    break;
                case SERVO_MODE_VEL:
                    if (!dry_run_){
                        pwmWrite(servo->pin_, servo->getPwmValueFromDegree(vel));
                    }else{
                        spdlog::info("[Dry running] pin:" + std::to_string(servo->pin_) + ", pwm:" + std::to_string(servo->getPwmValueFromDegree(vel)));
                    }
                    break;
                case SERVO_MODE_TOR:

                    if(torque < 0){
                        torque = - torque;
                        ctl_a = 0;
                        ctl_b = 1;
                    }

                    if(servo->ctl_a_pin_ == -1 || servo->ctl_b_pin_ == -1){
                        spdlog::error("Can not control torque servo without ctl pins.");
                        throw std::logic_error("Can not control torque servo without ctl pins.");
                    }

                    if (!dry_run_){
                        
                        digitalWrite(servo->standby_pin_, 1);
                        pwmWrite(servo->pin_, servo->getPwmValueFromDegree(torque));
                        digitalWrite(servo->ctl_a_pin_, ctl_a);
                        digitalWrite(servo->ctl_b_pin_, ctl_b);
                        spdlog::debug("[test running] {} - torque={}: pwm({}={}), ctl_a({}={}), ctl_b({}={})",
                                        servo->name_,
                                        std::to_string(feed_forward_torque_),
                                        std::to_string(servo->pin_), 
                                        std::to_string(servo->getPwmValueFromDegree(torque)),
                                        std::to_string(servo->ctl_a_pin_), std::to_string(ctl_a),
                                        std::to_string(servo->ctl_b_pin_), std::to_string(ctl_b)
                                        );
                    }else{
                        spdlog::info("[Dry running] pin=value: pwm({}={}), ctl_a({}={}), ctl_b({}={})",
                                        std::to_string(servo->pin_), std::to_string(servo->getPwmValueFromDegree(torque)),
                                        std::to_string(servo->ctl_a_pin_), std::to_string(ctl_a),
                                        std::to_string(servo->ctl_b_pin_), std::to_string(ctl_b)
                                        );
                    }
                    break;
                default:
                    spdlog::error("Unsupported servo mode: {}", servo->mode_);
                    throw std::logic_error("Unsupported servo mode.");
            }


            JointState js = {};
            js.id = joint_name;
            js.position_rad = 0;
            js.torque_n_m = 0;
            js.velocity_rad_per_sec = 0;
            if(servo->hall_){
                HallAdapter* hall = hall_adapter_map_.at(joint_name);
                js.position_rad = hall->getHallPositionRad();
                js.torque_n_m = 0;
                js.velocity_rad_per_sec = hall->getHallSpeedRadPerSec();
            }

            joint_states.push_back(js);

        }
        
        return joint_states;
    };

    
    ImuData WiringpiAdapter::getImu() {
        atk_ms901m_gyro_data_t* gyro_dat = atk_imu_->atk_ms901m_get_gyrometer_data();
        atk_ms901m_accelerometer_data_t* accelerometer_dat = atk_imu_->atk_ms901m_get_accelerometer_data();
        atk_ms901m_quaternion_data_t* quaternion_dat = atk_imu_->atk_ms901m_get_quaternion_data();

        ImuData imu = {};
        imu.angular_velocity_imu_in_imu = {gyro_dat->x, gyro_dat->y, gyro_dat->z};
        imu.linear_acceleration_imu_in_imu = {accelerometer_dat->x, accelerometer_dat->y, accelerometer_dat->z};
        imu.orientation_imu_in_ars = {quaternion_dat->q0, quaternion_dat->q1, quaternion_dat->q2, quaternion_dat->q3};

        return imu;
    }

    void WiringpiAdapter::run_hall_thread(){
        while (running_){
            for(auto p : hall_adapter_map_){
                p.second->updateHallValue();
            }
            delay(10);
        }
    }



} // namespace vulp::actuation::wiringpi