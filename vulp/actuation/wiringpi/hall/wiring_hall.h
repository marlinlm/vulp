#pragma once

#include <string>
#include <spdlog/spdlog.h>
#include <thread>
#include <functional>
#include <time.h>

#include "vulp/actuation/wiringpi/wiringPi.h"

#define FIFO_SAMPLE_SIZE 16

// #ifndef HALL_ADAPTER
// #define HALL_ADAPTER

namespace vulp::actuation::wiringpi {

using namespace std;

struct History
{
    long buf[FIFO_SAMPLE_SIZE];  
    long ts[FIFO_SAMPLE_SIZE];
    uint16_t size;
    uint16_t current;                                
};

class HallAdapter{

private:

  const string name_;
  const int hall_a_pin_;
  const int hall_b_pin_;
  const float resolution_rad_;
  const float resolution_;

  bool a_level_;
  bool b_level_;
  bool running_;
  long value_;
  long value_ts_;
  History history_;

public:
  HallAdapter():
   name_("illegal"),
   hall_a_pin_(0),
   hall_b_pin_(0),
   resolution_rad_(0),
   resolution_(0.0f)
  {
    spdlog::error("Illegal initialization of hall adapter.");
    throw std::logic_error("Illegal initialization of hall adapter.");
  };
  explicit HallAdapter(const string name, const int hall_a_pin, const int hall_b_pin,const float resolution);
  ~HallAdapter();

  void init();
  void handleAEvent();
  void handleBEvent();
  void updateHallValue();
  long getHallValue() const;
  float getHallSpeedRadPerSec() const;
  float getHallPositionRad() const;


};


// #endif //HALL_ADAPTER
}