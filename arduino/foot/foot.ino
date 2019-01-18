//23000 pulse / spin (maxon)
//#define USE_USBCON
#include <ros.h>
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"

#include "custom_msg/RL.h"



// main for testing.
uint8_t left_addr = 0x35;
uint8_t right_addr = 0x21;


IseMotorDriver left_motor_driver = IseMotorDriver(left_addr);
IseMotorDriver right_motor_driver = IseMotorDriver(right_addr);


ros::NodeHandle nh;

custom_msg::RL pulse;
custom_msg::RL power;

ros::Publisher pulse_chatter("pulse",&pulse);


int left_motor_pw = 0;
int right_motor_pw = 0;

long left_enc = 0;
long right_enc = 0;

void power_Callback(custom_msg::RL & power)
{
  //ROS_INFO("receive right_pw: [%d]", right_pw.data);
  right_motor_pw = power.right;
  left_motor_pw = power.left; 
}

ros::Subscriber<custom_msg::RL>power_subscriber("power",power_Callback);

void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();

  delay(300);

  nh.initNode();
  nh.advertise(pulse_chatter);
  nh.subscribe(power_subscriber);
}

void loop() {

  left_motor_driver.setSpeed(left_motor_pw);
  left_enc = left_motor_driver.encorder();

  right_motor_driver.setSpeed(right_motor_pw);
  right_enc = right_motor_driver.encorder();

  pulse.right = right_enc;
  pulse.left = -1*left_enc;


  pulse_chatter.publish( &pulse );

  nh.spinOnce();
}
