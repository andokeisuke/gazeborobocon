//23000 pulse / spin (maxon)
//#define USE_USBCON
#include <ros.h>
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"

#include "custom_msg/RL.h"

/*
class Wheel
{
  public:
  uint8_t addr;
  IseMotorDraver motor_driver;
  int pw; 
  long enc; 
  void setup();
};
*/

// main for testing.
uint8_t sterring_left_front_addr = 0x33;
uint8_t sterring_right_front_addr = 0x23;
uint8_t sterring_left_rear_addr = 0x39;
uint8_t sterring_right_rear_addr = 0x22;

uint8_t wheel_left_front_addr = 0x21;
uint8_t wheel_right_front_addr = 0x35;
uint8_t whell_left_rear_addr = 0x25;
uint8_t wheel_right_rear_addr = 0x24;




IseMotorDriver sterring_left_front_motor_driver = IseMotorDriver(sterring_left_front_addr);
IseMotorDriver sterring_right_front_motor_driver = IseMotorDriver(sterring_right_front_addr);
IseMotorDriver sterring_left_rear_motor_driver = IseMotorDriver(sterring_left_rear_addr);
IseMotorDriver sterring_right_rear_motor_driver = IseMotorDriver(sterring_right_rear_addr);

IseMotorDriver wheel_left_front_motor_driver = IseMotorDriver(wheel_left_front_addr);
IseMotorDriver wheel_right_front_motor_driver = IseMotorDriver(wheel_right_front_addr);
IseMotorDriver wheel_left_rear_motor_driver = IseMotorDriver(wheel_left_rear_addr);
IseMotorDriver wheel_right_rear_motor_driver = IseMotorDriver(wheel_right_rear_addr);


ros::NodeHandle nh;

custom_msg::RL frontpulse;
custom_msg::RL rearpulse;

custom_msg::RL frontpower;
custom_msg::RL rearpower;

ros::Publisher front_pulse_chatter("front_pulse",&frontpulse);
ros::Publisher rear_pulse_chatter("rear_pulse",&rearpulse);


int left_front_motor_pw = 0;
int right_front_motor_pw = 0;
int left_rear_motor_pw = 0;
int right_rear_motor_pw = 0;


long left_rear_enc = 0;
long right_rear_enc = 0;
long left_front_enc = 0;
long right_front_enc = 0;


int sterring_left_front_pw = 0;
int sterring_right_front_pw = 0;
int sterring_left_rear_pw =  0;
int sterring_right_rear_pw = 0;

long wheel_left_front_enc = 0;
long wheel_right_front_enc = 0;
long whell_left_rear_enc = 0;
long wheel_right_rear_enc = 0;

//////////////////////////////////////////
void frontpower_Callback(custom_msg::RL & frontpower)
{
  //ROS_INFO("receive right_pw: [%d]", right_pw.data);
  right_front_motor_pw = frontpower.right;
  left_front_motor_pw = frontpower.left; 
}

void rearpower_Callback(custom_msg::RL & rearpower)
{
  //ROS_INFO("receive right_pw: [%d]", right_pw.data);
  right_rear_motor_pw = rearpower.right;
  left_rear_motor_pw = rearpower.left; 
}

ros::Subscriber<custom_msg::RL>frontpower_subscriber("frontpower",frontpower_Callback);
ros::Subscriber<custom_msg::RL>rearpower_subscriber("rearpower",rearpower_Callback);

void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();

  delay(300);

  nh.initNode();
  nh.advertise(rearpulse_chatter);
  nh.advertise(frontpulse_chatter);

  nh.subscribe(frontpower_subscriber);
  nh.subscribe(rearpower_subscriber);

}

void loop() {

  left_front_motor_driver.setSpeed(left_front_motor_pw);
  left_front_enc = left_front_motor_driver.encorder();

  right_front_motor_driver.setSpeed(right_front_motor_pw);
  right_front_enc = right_front_motor_driver.encorder();

  left_rear_motor_driver.setSpeed(left_rear_motor_pw);
  left_rear_enc = left_rear_motor_driver.encorder();

  right_rear_motor_driver.setSpeed(right_rear_motor_pw);
  right_rear_enc = right_rear_motor_driver.encorder();

  
  frontpulse.right = right_front_enc;
  frontpulse.left =  left_front_enc;

  rearpulse.right = right_rear_enc;
  rearpulse.left =  left_rear_enc;

  rearpulse_chatter.publish( &rearpulse );
  frontpulse_chatter.publish( &frontpulse );


  nh.spinOnce();
}
