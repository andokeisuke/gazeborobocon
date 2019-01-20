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
  double now_vel;
  double KP;
  double KI;
  double KD;
  double enc;
  int pid_control();
  int pid_control_st();
  private :
  double p , i , d ;
  double integral ;
  double now_diff ;
  double pre_diff ;
  double power;

};

int MotorHandler::pid_control()
{

  pre_diff = now_diff;
  now_diff = this->target_vel - this->now_vel;
  integral += now_diff * dt;

  p = KP * now_diff;
  i = KI * integral ;
  d = KD * (now_diff - pre_diff) /dt;
/*
  ROS_INFO(" p gain: [%f]",p);
  ROS_INFO(" i gain: [%f]",i);
  ROS_INFO(" d gain: [%f]",d);


  ROS_INFO("target_vel: aaa[%f]",this->target_vel);
  ROS_INFO("now_vel: aaa[%f]",this->now_vel);
/*
  ROS_INFO("target - now: [%f]",this->target_vel - this->now_vel);
*/
  power = power+ p + i + d;
  if(fabs(target_vel)<0.05 && fabs(now_vel)<0.01 /*&&fabs(power) < 10*/)
  {
    integral = 0;
    power = 0;

  }
  else if(12 < power )
    power = 12;
  else if(power < -12)
    power = -12;
 return power;
}


int MotorHandler::pid_control_st()
{

  pre_diff = now_diff;
  now_diff = this->target_vel - this->now_vel;
  if(now_diff>320)power=12;
  else if(now_diff<-320)power=-12;
  else{
  integral += now_diff * dt;

  p = KP * now_diff;
  i = KI * integral ;
  d = KD * (now_diff - pre_diff) /dt;
/*
  ROS_INFO(" p gain: [%f]",p);
  ROS_INFO(" i gain: [%f]",i);
  ROS_INFO(" d gain: [%f]",d);


  ROS_INFO("target_vel: aaa[%f]",this->target_vel);
  ROS_INFO("now_vel: aaa[%f]",this->now_vel);
/
  ROS_INFO("target - now: [%f]",this->target_vel - this->now_vel);
*/
  power = power+ p + i + d;
  if(fabs(target_vel)<0.05 && fabs(now_vel)<0.01 /*&&fabs(power) < 10*/)
  {
    integral = 0;
    power = 0;

  }
  else if(12 < power )
    power = 12;
  else if(power < -12)
    power = -12;
  }
 return power;
}



// ============================== arguments ==============================

IseMotorDriver right_front = IseMotorDriver(0x0);
IseMotorDriver right_rear = IseMotorDriver(0x0);
IseMotorDriver left_front = IseMotorDriver(0x0);
IseMotorDriver left_rear = IseMotorDriver(0x0);
IseMotorDriver right_front_st = IseMotorDriver(0x0);
IseMotorDriver right_rear_st = IseMotorDriver(0x0);
IseMotorDriver left_front_st = IseMotorDriver(0x33);
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
void setMotorHandlerWh(MotorHandler *mh) {
  mh->KP = 0.01;
  mh->KI = 0;
  mh->KD = 0;
}

void setMotorHandlerSt(MotorHandler *mh) {
  mh->KP = 0.0001;//0.00032
  mh->KI = 0;//0.00012
  mh->KD = 0.0001;//0.00032
}

int vel_time = 0;
void calc_now_vel() {

  double rf_enc = right_front.encorder();
  double rr_enc = right_rear.encorder();
  double lf_enc = left_front.encorder();
  double lr_enc = left_rear.encorder();
  double rf_st_enc = 0;//right_front_st.encorder();
  double rr_st_enc =0;// right_rear_st.encorder();
  double lf_st_enc = 0;//left_front_st.encorder();
  double lr_st_enc = 0;//left_rear_st.encorder();
  if (vel_time == 0) {
    vel_time = millis();
  } else {
    int time = millis();
    int diff = time - vel_time;
    vel_time = time;
    
    right_front_handler.now_vel = -(right_front_handler.enc - rf_enc) / diff;
    right_rear_handler.now_vel = -(right_rear_handler.enc - rr_enc) / diff;
    left_front_handler.now_vel = -(left_front_handler.enc - lf_enc) / diff;
    left_rear_handler.now_vel = -(left_rear_handler.enc - lr_enc) / diff;

    right_front_st_handler.now_vel = rf_st_enc;
    right_rear_st_handler.now_vel = rr_st_enc;
    left_front_st_handler.now_vel = lf_st_enc;
    left_rear_st_handler.now_vel = lr_st_enc;

  }

  right_front_handler.enc = rf_enc;
  right_rear_handler.enc = rr_enc;
  left_front_handler.enc = lf_enc;
  left_rear_handler.enc = lr_enc;

  right_front_st_handler.enc = rf_st_enc;
  right_rear_st_handler.enc = rr_st_enc;
  left_front_st_handler.enc = lf_st_enc;
  left_rear_st_handler.enc = lr_st_enc;

}

void setup() {
  Wire.begin();
  nh.initNode();

  nh.subscribe(right_front_sub);
  nh.subscribe(right_rear_sub);
  nh.subscribe(left_front_sub);
  nh.subscribe(left_rear_sub);

  setMotorHandlerWh(&right_front_handler);
  setMotorHandlerWh(&right_rear_handler);
  setMotorHandlerWh(&left_front_handler);
  setMotorHandlerWh(&left_rear_handler);
  setMotorHandlerSt(&right_front_st_handler);
  setMotorHandlerSt(&right_rear_st_handler);
  setMotorHandlerSt(&left_front_st_handler);
  setMotorHandlerSt(&left_rear_st_handler);

 
}

int flag = 0;
void loop() {
  
  if(flag==10){
    calc_now_vel();
    flag=0;
  }
  flag++;
  right_front.setSpeed(right_front_handler.pid_control());
  right_rear.setSpeed(right_rear_handler.pid_control());
  left_front.setSpeed(left_front_handler.pid_control());
  //left_rear.setSpeed(left_rear_handler.pid_control());
//ROS_INFO(" [%f]",left_front_handler.pid_control());
  //right_front_st.setSpeed(right_front_st_handler.pid_control_st());
  //right_rear_st.setSpeed(right_rear_st_handler.pid_control_st());
  //left_front_st.setSpeed(left_front_st_handler.pid_control_st());
  left_front_st.setSpeed(20);
  //left_rear_st.setSpeed(left_rear_st_handler.pid_control_st());
 delay(5);
  nh.spinOnce();
  
}
