#include "Arduino.h"
#include "kinjo_st_motor_slave_receiver.h"
#include "ti2c_slave.h"
#include <Stepper.h>
#include <Wire.h>


// IseMotorDriver::IseMotorDriver(uint8_t i2caddr) {}
// void IseMotorDriver::setSpeed(int power){}
// long IseMotorDriver::encorder(){}

KinjoStMotorSlave::KinjoStMotorSlave(uint8_t i2caddr,float dps,int rs) 
:ti2c(i2caddr)
,stepper( (int) (360/dps), 8, 9, 10, 11) // create an instance of the stepper class, specifying
{
  // @param uint8_t i2caddr: i2c addr of motor driver
  //initializer
  this->addr = i2caddr;
  this->degree_per_step   = dps;
  
  this->stepper.setSpeed(rs);
  Wire.begin(this->addr);       // join i2c bus with address
}

int KinjoStMotorSlave::receive()
{
  this->tar_degree = this->ti2c.receiveStr(); 
}

void KinjoStMotorSlave::rotate() {
  int s = 0;
  int send_step = 0;
  //Serial.println(tar_degree);
  this->pre_degree = this->degree;
  this->degree = this->tar_degree;
  if (this->degree != this->pre_degree) 
  {
    s = this->degree - this->pre_degree;
    send_step =  s / this->degree_per_step;
/*
    Serial.println("位置");
    Serial.println(degree);
    Serial.println("step数");
    Serial.println(steep);
*/
  }
  this->stepper.step(send_step);
  this->rotation_state = false;
}
