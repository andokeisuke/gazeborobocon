#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <Wire.h>
#include <ros.h>
#include <custom_msg/wh_msg.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#include "kinjo_stp_motor_driver.h"
#define ENC_PER_DEG 11//１度のエンコーダの値

ros::NodeHandle  nh;

double dt = 0.1;


struct MotorHandler
{
  public:
  double target_vel;

};




// ============================== arguments ==============================

IseMotorDriver right_front = IseMotorDriver(0x62);
IseMotorDriver right_rear = IseMotorDriver(0x61);
IseMotorDriver left_front = IseMotorDriver(0x21);
IseMotorDriver left_rear = IseMotorDriver(0x63);
KinjoStMotorDriver right_front_st = KinjoStMotorDriver(0x35);
KinjoStMotorDriver right_rear_st = KinjoStMotorDriver(0x24);
KinjoStMotorDriver left_front_st = KinjoStMotorDriver(0x33);
KinjoStMotorDriver left_rear_st =  KinjoStMotorDriver(0x34);                                                             

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
  mh_st -> target_vel = target_deg_st;
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

int vel_time = 0;


void setup() {
  Wire.begin();
  nh.initNode();

  nh.subscribe(right_front_sub);
  nh.subscribe(right_rear_sub);
  nh.subscribe(left_front_sub);
  nh.subscribe(left_rear_sub);


 
}

void loop() {
  

  right_front.setSpeed(right_front_handler.target_vel);
  right_rear.setSpeed(right_rear_handler.target_vel);
  left_front.setSpeed(left_front_handler.target_vel);
  left_rear.setSpeed(left_rear_handler.target_vel);
//ROS_INFO(" [%f]",left_front_handler.pid_control());
  right_front_st.rotate_degree(right_front_st_handler.target_vel);
  right_rear_st.rotate_degree(right_rear_st_handler.target_vel);
  left_front_st.rotate_degree(left_front_st_handler.target_vel);
  //left_front_st.setSpeed(90);
  left_rear_st.rotate_degree(left_rear_st_handler.target_vel);
 delay(5);
  nh.spinOnce();
  
}
