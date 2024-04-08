#include "vulp/actuation/LessieInterface.h"
#include "vulp/actuation/wiringpi/wiringpi_config.h"

namespace vulp::actuation {

using namespace vulp::actuation::wiringpi;

LessieInterface::LessieInterface(const ServoLayout& layout, const int can_cpu,
                                 const WiringpiConfig& config, const bool dry_run = false):
      Interface(layout),
      can_cpu_(can_cpu),
      can_thread_(std::bind(&LessieInterface::run_can_thread, this)),
      config_(config),
      wiringpi_adapter_(new WiringpiAdapter(config, dry_run)){}

LessieInterface::~LessieInterface() {
  done_ = true;  
  if (ongoing_can_cycle_) {
    // thread will exit at the end of its loop because done_ is set
    spdlog::info("Waiting for CAN thread to finish active cycle...");
    can_thread_.join();
    spdlog::info("CAN thread finished last cycle cleanly");
  } else /* thread is waiting for notification */ {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      can_wait_condition_.notify_one();
    }
    can_thread_.join();
  }
}

void LessieInterface::reset(const palimpsest::Dictionary& config) {}

void LessieInterface::cycle(
    const moteus::Data& data,
    std::function<void(const moteus::Output&)> callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (ongoing_can_cycle_) {
    throw std::logic_error(
        "Cycle cannot be called before the previous one has completed.");
  }

  callback_ = std::move(callback);
  ongoing_can_cycle_ = true;
  data_ = data;

  can_wait_condition_.notify_all();
}

void LessieInterface::run_can_thread() {
  spdlog::info("running can thread");
  vulp::utils::configure_cpu(can_cpu_);
  vulp::utils::configure_scheduler(10);
  pthread_setname_np(pthread_self(), "can_thread");
  while (!done_) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      if (!ongoing_can_cycle_) {
        can_wait_condition_.wait(lock);
        if (done_) {
          return;
        }
        if (!ongoing_can_cycle_) {
          continue;
        }
      }
    }
    
    auto output = cycle_can_thread();
    std::function<void(const moteus::Output&)> callback_copy;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ongoing_can_cycle_ = false;
      std::swap(callback_copy, callback_);
    }
    callback_copy(output);
  }
}

moteus::Output LessieInterface::cycle_can_thread() {

  int result_size = run_joint_commands();
  read_imu();

  moteus::Output output;
  output.query_result_size = result_size;

  return output;
}

int LessieInterface::run_joint_commands(){
  int result = 0;
  const auto joint_states = wiringpi_adapter_->runServoCommands(servo_layout(), data_.commands);
  for (size_t i = 0; i < data_.commands.size(); ++i) {
    result++;

    // update local joint status
    data_.replies[i].id = data_.commands[i].id;
    data_.replies[i].result.position = joint_states[i].position_rad;
    data_.replies[i].result.torque = joint_states[i].torque_n_m;
    data_.replies[i].result.velocity = joint_states[i].velocity_rad_per_sec;
  }

  return result;

}


void LessieInterface::read_imu(){
  imu_ = wiringpi_adapter_->getImu();
}


}  // namespace vulp::actuation
