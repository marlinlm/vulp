#pragma once

#include <pthread.h>
#include <spdlog/spdlog.h>
#include <Eigen/Geometry>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "vulp/actuation/ImuData.h"
#include "vulp/actuation/Interface.h"
#include "vulp/actuation/moteus/protocol.h"
#include "vulp/utils/realtime.h"
#include "vulp/actuation/wiringpi/wiringpi.h"
#include "vulp/actuation/wiringpi/wiringpi_config.h"
#include "vulp/actuation/moteus/ServoCommand.h"

namespace vulp::actuation {

using namespace vulp::actuation::wiringpi;

/*! Interface to moteus controllers.
 *
 * Internally it uses a background thread to operate the pi3hat, enabling the
 * main thread to perform work while servo communication is taking place.
 */
class LessieInterface : public Interface {


 public:
  /*! Configure interface and spawn CAN thread.
   *
   * \param[in] layout Servo layout.
   * \param[in] can_cpu CPUID of the core to run the CAN thread on.
   * \param[in] config Configuration for the wiringpi.
   */
  LessieInterface(const ServoLayout& layout, const int can_cpu,
                  const WiringpiConfig& config, const bool dry_run);

  //! Stop CAN thread
  ~LessieInterface();

  /*! Reset interface.
   *
   * \param[in] config Additional configuration dictionary.
   */
  void reset(const palimpsest::Dictionary& config) override;

  /*! Spin a new communication cycle.
   *
   * \param[in] data Buffer to read commands from and write replies to.
   * \param[in] callback Function to call when the cycle is over.
   *
   * The callback will be invoked from an arbitrary thread when the
   * communication cycle has completed. All memory pointed to by \p data must
   * remain valid until the callback is invoked.
   */
  void cycle(const moteus::Data& data,
             std::function<void(const moteus::Output&)> callback) final;

  ImuData imu_data() const noexcept final {
    return imu_;
  }

 private:
  /*! Main loop of the CAN thread.
   *
   * Synchronizes with \ref cycle via the internal condition variable.
   */
  void run_can_thread();

  /*! Execute one communication cycle on the CAN bus.
   *
   * Also, request the latest filtered attitude from the pi3hat.
   */
  moteus::Output cycle_can_thread();

 private:

  ImuData imu_;

  //! CPUID of the core to run the CAN thread on.
  const int can_cpu_;

  //! Mutex associated with \ref can_wait_condition_
  std::mutex mutex_;

  //! Condition variable to notify the CAN thread to spin a new cycle.
  std::condition_variable can_wait_condition_;

  //! True if and only if a CAN communication cycle is under way.
  bool ongoing_can_cycle_ = false;

  //! CAN thread exits when it is notified and this boolean is true.
  bool done_ = false;

  //! Callback function called upon completion of a CAN cycle
  std::function<void(const moteus::Output&)> callback_;

  //! Buffer to read commands from and write replies to.
  moteus::Data data_;

  //! Thread for CAN communication cycles
  std::thread can_thread_;

  const WiringpiConfig config_;

  WiringpiAdapter* wiringpi_adapter_;
  
  //! Servo replies, laid out as a joint id -> servo_reply map
  std::map<std::string, moteus::ServoReply> servo_reply_;
  
  int run_joint_commands();

  void read_imu();

};

}  // namespace vulp::actuation
