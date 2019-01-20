#define ENC_PER_DEG 46//１度のエンコーダの値
#include <stdlib.h>
#include <stdio.h>

#include <FlexiTimer2.h>
#include <Wire.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <gerege_stepping/gerege_stepping_msg.h>
#include "ti2c.h"
#include "ise_motor_driver.h"


double dt = 0.1;
double output_limit = 30;
int tar_arm_deg = 0;
int power = 0;


int pin; //ピン番号
int cw;  //CW
float pw; //パルス幅
bool dir; //回転方向
int tim; //動かす時間
bool check;

ros::NodeHandle nh;
gerege_stepping::gerege_stepping_msg msg;

class MotorHandler
{
  public:
    double target_vel;
    double now_vel;
    double KP;
    double KI;
    double KD;
    double enc;
    double power;

    int pid_control();
  private :
    double p , i , d ;
    double integral ;
    double now_diff ;
    double pre_diff ;
};

int MotorHandler::pid_control()
{

  pre_diff = now_diff;
  now_diff = this->target_vel - this->now_vel;
  integral += now_diff * dt;

  p = KP * now_diff;

  i = KI * integral ;
  d = KD * (now_diff - pre_diff) / dt;
  /*
    ROS_INFO(" p gain: [%f]",p);
    ROS_INFO(" i gain: [%f]",i);
    ROS_INFO(" d gain: [%f]",d);


    ROS_INFO("target_vel: aaa[%f]",this->target_vel);
    ROS_INFO("now_vel: aaa[%f]",this->now_vel);
    /*
    ROS_INFO("target - now: [%f]",this->target_vel - this->now_vel);
  */
  power = power + p + i + d;
  if (fabs(now_diff) < ENC_PER_DEG * 5 /*&&fabs(power) < 10*/)
  {
    integral = 0;
    power = 0;

  }
  else if (output_limit <= power )
    power = output_limit;
  else if (power <= -output_limit)
    power = -output_limit;


  return power;
}

// ============================== arguments ==============================

IseMotorDriver right_arm = IseMotorDriver(0x37);//36
IseMotorDriver left_arm = IseMotorDriver(0x36);//37

MotorHandler right_arm_handler;
MotorHandler left_arm_handler;

// ============================== callback ==============================

void set_target(struct MotorHandler *mh_arm, double target_deg)
{
  mh_arm -> target_vel = (target_deg) * ENC_PER_DEG;
}

void messageCallback(const gerege_stepping::gerege_stepping_msg& msg) {
  dir = msg.dir;
  if (check == dir) {
    pin = msg.pin;
    cw = msg.cw;
    pw = msg.pw;
    tim = msg.tim;

    pinMode(pin, OUTPUT);
    pinMode(cw, OUTPUT);
    digitalWrite(cw, dir);
    FlexiTimer2::set(pw, 1 / 10, stepp);
    FlexiTimer2::start();

    delay(tim);
    FlexiTimer2::stop();
    check = !dir;

  }
}

void arm_deg_Callback(const std_msgs::Int16& deg){

  tar_arm_deg = deg.data;
}

ros::Subscriber<gerege_stepping::gerege_stepping_msg> stepp_sub("stepping_var", messageCallback);
ros::Subscriber<std_msgs::Int16> arm_deg_sub("tar_arm_deg", arm_deg_Callback);


// ==================== functions ==================== //
 void setMotorHandlerArm(struct MotorHandler *mh) {
  mh->KP = 0.00001;
  mh->KI = 0.0000;
  mh->KD = 0.0000;
}

int vel_time = 0;
void calc_now_vel() {
  if (vel_time == 0) {
    vel_time = millis();
  } else {
    int time = millis();
    int diff = time - vel_time;
    vel_time = time;

    right_arm_handler.now_vel = right_arm.encorder();
    left_arm_handler.now_vel = left_arm.encorder();
  }
  right_arm_handler.enc = right_arm.encorder();
  left_arm_handler.enc = left_arm.encorder();

}

void stepp() {
  static  boolean output = HIGH;
  digitalWrite(pin, output);
  output = !output;
}


void setup() {

  nh.initNode();

  nh.subscribe(stepp_sub);
  nh.subscribe(arm_deg_sub);

  check = dir;


  Wire.begin();
  Serial.begin(115200);
  Serial.println("start");

//  set_target(&right_arm_handler, tar_arm_deg);
//  set_target(&left_arm_handler, -tar_arm_deg);

//  setMotorHandlerArm(&right_arm_handler);
//  setMotorHandlerArm(&left_arm_handler);

}

void loop() {
/*
  calc_now_vel();
  Serial.print(right_arm_handler.target_vel);

  right_arm.setSpeed(right_arm_handler.pid_control());
  left_arm.setSpeed((-right_arm_handler.power));

  Serial.print("power");
  //     Serial.print(right_arm_handler.power);
  Serial.print(right_arm_handler.power);


  Serial.print("enc");
  //     Serial.println(right_arm_handler.enc);
  Serial.println(right_arm_handler.enc);
*/
  if(tar_arm_deg > 0)
  {
    power = 30;
  }
  else if(tar_arm_deg < 0)
  {
    power = -30;
  }
  else
  {
    power = 0;
  }

  right_arm.setSpeed(power);
  left_arm.setSpeed(-power);

  delay(5);
  nh.spinOnce();

}
