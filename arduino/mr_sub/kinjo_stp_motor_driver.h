#ifndef KinjoStMotorDriver_h
#define KinjoStMotorDriver_h
#include "ti2c.h"
#include "Arduino.h"

class KinjoStMotorDriver {
  public:
    KinjoStMotorDriver(uint8_t i2caddr);
    void rotate_degree(int wheel_degree);
  private:
    uint8_t addr;
    Ti2c ti2c;
};
#endif
