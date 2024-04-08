#include <string>
#include <vector>

#include "wiringpi_config.h"

namespace vulp::actuation::wiringpi {

using namespace std;

int ServoConfig::getPwmValueFromDegree(int degree_in){
    int degree = degree_in > max_degree_ ? max_degree_ : degree_in;
    degree = degree_in < min_degree_ ? min_degree_ : degree;
    return (int)(((float)(degree - min_degree_)) / degree_range_ * pwm_range_ + min_pwm_);
}

}