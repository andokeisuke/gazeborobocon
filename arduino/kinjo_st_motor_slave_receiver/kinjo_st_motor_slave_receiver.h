#ifndef KinjoStMotorSlave_h
#define KinjoStMotorSlave_h
#include "ti2c_slave.h"
#include "Arduino.h"
#include <Stepper.h>

class KinjoStMotorSlave {
  public:
    KinjoStMotorSlave(uint8_t i2caddr,float,int);
    int receive();
    void rotate();
    long encorder();
    float degree_per_step;
    bool rotation_state;
    int degree;

  private:
    uint8_t addr;
    Ti2c ti2c;
    Stepper stepper;
    int pre_degree;
    int tar_degree;
};
#endif
