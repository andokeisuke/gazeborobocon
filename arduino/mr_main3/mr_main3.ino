#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <Wire.h>

#include <ros.h>
#include <custom_msg/wh_msg.h>


#include "ti2c.h"
#include "ise_motor_driver.h"
#define ENC_PER_DEG 16//１度のエンコーダの値

ros::NodeHandle  nh;

double dt = 0.1;


struct MotorHandler
{
  public:
  double target_vel;

};


// ============================== arguments ==============================

IseMotorDriver right_front = IseMotorDriver(0x0);
IseMotorDriver right_rear = IseMotorDriver(0x0);
IseMotorDriver left_front = IseMotorDriver(0x0);
IseMotorDriver left_rear = IseMotorDriver(0x0);
IseMotorDriver right_front_st = IseMotorDriver(0x0);
IseMotorDriver right_rear_st = IseMotorDriver(0x0);
IseMotorDriver left_front_st = IseMotorDriver(0x00);
IseMotorDriver left_rear_st = IseMotorDriver(0x0);                                                             

MotorHandler right_front_handler;
MotorHandler right_rear_handler;
MotorHandler left_front_handler;
MotorHandler left_rear_handler;
MotorHandler right_front_st_handler;
MotorHandler right_rear_st_handler;
MotorHandler left_front_st_handler;
MotorHandler left_rear_st_handler;

// ============================== callback ==============================

void set_target(MotorHandler *mh, MotorHandler *mh_st, double target_vel, double target_deg_st)
{
  mh -> target_vel = target_vel;
  mh_st -> target_vel = (target_deg_st) * ENC_PER_DEG;
}

void wh_cb_rfront(const custom_msg::wh_msg& msg)
{
  set_target(&right_front_handler, &right_front_st_handler, msg.wh_target_vel, msg.st_target_deg);
}

void wh_cb_rrear(const custom_msg::wh_msg& msg)
{
  set_target(&right_rear_handler, &right_rear_st_handler, msg.wh_target_vel, msg.st_target_deg);
}

void wh_cb_lfront(const custom_msg::wh_msg& msg)
{
  set_target(&left_front_handler, &left_front_st_handler, msg.wh_target_vel, msg.st_target_deg);
}

void wh_cb_lrear(const custom_msg::wh_msg& msg)
{
  set_target(&left_rear_handler, &left_rear_st_handler, msg.wh_target_vel, msg.st_target_deg);
}

ros::Subscriber<custom_msg::wh_msg>right_front_sub("right_front",wh_cb_rfront);
ros::Subscriber<custom_msg::wh_msg>right_rear_sub("right_rear",wh_cb_rrear);
ros::Subscriber<custom_msg::wh_msg>left_front_sub("left_front",wh_cb_lfront);
ros::Subscriber<custom_msg::wh_msg>left_rear_sub("left_rear",wh_cb_lrear);


// ==================== functions ==================== //


void setup() {
  nh.getHardware()->setBaud(57600);
  Wire.begin();
  nh.initNode();
  
  nh.subscribe(right_front_sub);
  nh.subscribe(right_rear_sub);
  nh.subscribe(left_front_sub);
  nh.subscribe(left_rear_sub);

/*
   servos[0].write(open_angle);
   servos[3].write(open_angle);
   delay(1000);

   servos[4].write(open_angle);
   delay(1000);

   servos[4].write(close_angle);
   delay(1000);

   servos[4].write(close_angle);
   servos[5].write(open_angle);
   delay(1000);
*/

}

void loop() {
  
/*
  right_front.setSpeed(right_front_handler.target_vel);
  right_rear.setSpeed(right_rear_handler.target_vel);
  left_front.setSpeed(left_front_handler.target_vel);
  left_rear.setSpeed(left_rear_handler.target_vel);
//ROS_INFO(" [%f]",left_front_handler.pid_control());
  right_front_st.setSpeed(right_front_st_handler.target_vel);
  right_rear_st.setSpeed(right_rear_st_handler.target_vel);
  left_front_st.setSpeed(left_front_st_handler.target_vel);
  left_rear_st.setSpeed(left_rear_st_handler.target_vel);
  */
 delay(5);
  nh.spinOnce();
  
  
}
