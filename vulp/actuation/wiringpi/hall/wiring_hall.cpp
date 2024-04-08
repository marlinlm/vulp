#include "wiring_hall.h"
#include <ctime>


namespace vulp::actuation::wiringpi {

using namespace std;
using namespace std::chrono;

HallAdapter* _me[8];

int _fun_size = 0;

void _handleAEvent_1(){_me[0]->handleAEvent();}
void _handleAEvent_2(){_me[1]->handleAEvent();}
void _handleAEvent_3(){_me[2]->handleAEvent();}
void _handleAEvent_4(){_me[3]->handleAEvent();}
void _handleAEvent_5(){_me[4]->handleAEvent();}
void _handleAEvent_6(){_me[5]->handleAEvent();}
void _handleAEvent_7(){_me[6]->handleAEvent();}
void _handleAEvent_8(){_me[7]->handleAEvent();}

void _handleBEvent_1(){_me[0]->handleBEvent();}
void _handleBEvent_2(){_me[1]->handleBEvent();}
void _handleBEvent_3(){_me[2]->handleBEvent();}
void _handleBEvent_4(){_me[3]->handleBEvent();}
void _handleBEvent_5(){_me[4]->handleBEvent();}
void _handleBEvent_6(){_me[5]->handleBEvent();}
void _handleBEvent_7(){_me[6]->handleBEvent();}
void _handleBEvent_8(){_me[7]->handleBEvent();}


void(*_a_fun[])(void) = { _handleAEvent_1, 
                        _handleAEvent_2,
                        _handleAEvent_3,
                        _handleAEvent_5,
                        _handleAEvent_5,
                        _handleAEvent_6,
                        _handleAEvent_7,
                        _handleAEvent_8
                        };

void(*_b_fun[])(void) = { _handleBEvent_1, 
                          _handleBEvent_2,
                          _handleBEvent_3,
                          _handleBEvent_5,
                          _handleBEvent_5,
                          _handleBEvent_6,
                          _handleBEvent_7,
                          _handleBEvent_8
                        };

void register_hall_adapter(HallAdapter* me){
  if(_fun_size >= 8){
    throw std::logic_error("Hall adapter size overflow.");
  }
  _me[_fun_size] = me;
  _fun_size ++;
}

void(*get_a_fun())(){
  return _a_fun[_fun_size - 1];
}

void(*get_b_fun())(){
  return _b_fun[_fun_size - 1];
}

HallAdapter::HallAdapter(const string name, const int hall_a_pin, const int hall_b_pin,const float resolution):
  name_(name),
  hall_a_pin_(hall_a_pin),
  hall_b_pin_(hall_b_pin),
  resolution_rad_(2.0f * M_PI / (float)resolution / 4.0f),
  resolution_((float)resolution * 4.0f),
  running_(true),
  value_(0L),
  value_ts_(time(NULL))
  {
  
  a_level_ = 1;
  b_level_ = 0;

  history_.size = FIFO_SAMPLE_SIZE;
  for(int i=0; i<history_.size; i++){
    history_.buf[i] = 0;
    history_.ts[i] = 0;
  }
  history_.current = 0;      
}

void HallAdapter::init(){
  spdlog::info("hall motor name: {}", name_);
  spdlog::info("registing pin event {}", hall_a_pin_);
  register_hall_adapter(this);
  spdlog::info("fun size {}", _fun_size);
  wiringPiISR(hall_a_pin_, INT_EDGE_BOTH, get_a_fun());
  
  spdlog::info("setting pin mode {}", hall_a_pin_);
  pinMode(hall_a_pin_, INPUT);
  pullUpDnControl(hall_a_pin_, PUD_UP);
  
  spdlog::info("setting pin mode {}", hall_b_pin_);
  pinMode(hall_b_pin_, INPUT);
  pullUpDnControl(hall_b_pin_, PUD_DOWN);
  spdlog::info("fun size {}", _fun_size);
  wiringPiISR(hall_b_pin_, INT_EDGE_BOTH, get_b_fun());

  spdlog::info("registed pin event {}", hall_a_pin_);
}

HallAdapter::~HallAdapter(){
  spdlog::info("clearing hall adapter: {}", name_.c_str());
  running_ = false;
  pullUpDnControl(hall_a_pin_, PUD_OFF);
  pullUpDnControl(hall_b_pin_, PUD_OFF);
}

void HallAdapter::handleAEvent(){
  int a = digitalRead(hall_a_pin_);
  int b = digitalRead(hall_b_pin_);
  if(a == b){
    value_ --;
  }else{
    value_ ++;
  }
}

void HallAdapter::handleBEvent(){
  int a = digitalRead(hall_a_pin_);
  int b = digitalRead(hall_b_pin_);
  if(a == b){
    value_ ++;
  }else{
    value_ --;
  }
}


void HallAdapter::updateHallValue(){
  auto ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  value_ts_ = ms.count();
  history_.buf[history_.current] = value_;
  history_.ts[history_.current] = value_ts_;
  history_.current = (history_.current + 1) % history_.size;
}

long HallAdapter::getHallValue() const{
  return value_;
} ;

float HallAdapter::getHallSpeedRadPerSec() const{
  int cur_ptr = (history_.current - 1 + history_.size) % history_.size;

  long cur_val = history_.buf[cur_ptr];
  long cur_ts = history_.ts[cur_ptr];

  int last_val_ptr = (cur_ptr + 1) % history_.size;
  if(history_.ts[last_val_ptr] == 0){
    //at the begining, the buffer has not enough samples, use the first element in buffer as the last sample.
    last_val_ptr = 0;
  } 

  long last_val = history_.buf[last_val_ptr];
  long last_ts = history_.ts[last_val_ptr];
  float dt_sec = ((float)(cur_ts - last_ts)) / 1000.0f;

  return ((float)(cur_val - last_val)) * resolution_rad_ / dt_sec;
}

float HallAdapter::getHallPositionRad() const{
  return ((float)(((value_ * 100) % (long) (resolution_ * 100))) / 100) * resolution_rad_ ;
}

}