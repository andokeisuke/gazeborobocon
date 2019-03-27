#include "Arduino.h"
#include "kinjo_stp_motor_driver.h"
#include "ti2c.h"

// IseMotorDriver::IseMotorDriver(uint8_t i2caddr) {}
// void IseMotorDriver::setSpeed(int power){}
// long IseMotorDriver::encorder(){}

KinjoStMotorDriver::KinjoStMotorDriver(uint8_t i2caddr) :
ti2c(i2caddr){
  // @param uint8_t i2caddr: i2c addr of motor driver
  //initializer
  this->addr = i2caddr;
}

void KinjoStMotorDriver::rotate_degree(int degree){
  char buf[10];
  sprintf(buf, "%d", degree);
//  Serial.println(buf);
  this->ti2c.sendStr(buf); 
}
