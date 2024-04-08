#pragma once

#include <string>
#include <spdlog/spdlog.h>
#include <thread>
#include <functional>

#include "vulp/actuation/wiringpi/wiringPi.h"
#include "vulp/actuation/wiringpi/wiringSerial.h"
#include "vulp/actuation/moteus/ServoCommand.h"

namespace vulp::actuation::wiringpi {

using namespace std;

class UartAdapter{


private:
  bool running_;
  bool initialized_;
  int serial_port_;
  string device_;
  const int baud_;
  std::thread thread_;
  std::function<void(uint8_t*, uint16_t)> callback_;

public:
  UartAdapter(string device, int baud, std::function<void(uint8_t*, uint16_t)> callback):
      running_(true),
      initialized_(false),
      device_(device),
      baud_(baud),
      thread_(std::bind(&UartAdapter::run_listening_thread, this)),
      callback_(callback)
      {}

  ~UartAdapter(){
    spdlog::info("destorying UartAdapter.");
    running_ = false;
    thread_.join();
    serialClose(serial_port_);
  }

  bool reset(){
    spdlog::info("opening serial port with name{}", device_);
    try{

      if ((serial_port_ = serialOpen (device_.c_str(), baud_)) < 0)	/* open serial port */
      {
        spdlog::error("Unable to open serial device: {}\n", errno);
        return false;
      }
    }catch(exception& e){
      spdlog::error(e.what());
      return false;
    }

    spdlog::info("open serial device success, fd:{}\n", serial_port_);
    initialized_ = true;
    return true;
  }

  int run_listening_thread (){
    while(!initialized_){
      spdlog::info("[listening thread] serial port not initialized, waiting for initialization.");
      delayMicroseconds(1000);
    }

    uint8_t dat;
    
    while(running_){
      if(serialDataAvail (serial_port_) )
      { 
          int data = serialGetchar (serial_port_);
          dat = data;		/* receive character serially*/	
          callback_(&dat, 1);
      }
    }

    return 0;
  }

  int send_char(char data){
    while(!initialized_){
      spdlog::info("[send char]serial port not initialized, waiting for initialization.");
      delayMicroseconds(1000);
    }
    serialPutchar(serial_port_, data);
    return 0;
  }


};


}