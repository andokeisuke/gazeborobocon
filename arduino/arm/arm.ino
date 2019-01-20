//23000 pulse / spin (maxon)
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"
#define ENC_PER_DEG 46//１度のエンコーダの値

double dt = 0.1;
double output_limit = 50;

struct MotorHandler
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
  if(fabs(now_diff)<ENC_PER_DEG*5 /*&&fabs(power) < 10*/)
  {
    integral = 0;
    power = 0;

  }
  else if(output_limit <= power )
    power = output_limit;
  else if(power <= -output_limit)
    power = -output_limit;


 return power;
}

// ============================== arguments ==============================

IseMotorDriver right_arm = IseMotorDriver(0x37);//36
IseMotorDriver left_arm = IseMotorDriver(0x36);//37

MotorHandler right_arm_handler;
MotorHandler left_arm_handler;

// ============================== callback ==============================

void set_target(MotorHandler *mh_arm, double target_deg)
{
  mh_arm -> target_vel = (target_deg) * ENC_PER_DEG;
}

// ==================== functions ==================== //
void setMotorHandlerArm(MotorHandler *mh) {
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

void setup(){
  double tar_arm_deg = 50;
  Wire.begin();
  Serial.begin(115200);
  Serial.println("start");
  set_target(&right_arm_handler,tar_arm_deg);
  set_target(&left_arm_handler,-tar_arm_deg);

  setMotorHandlerArm(&right_arm_handler);
  setMotorHandlerArm(&left_arm_handler);

}

void loop(){

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

     delay(5);
}


/*

void loop(){

  calc_now_vel();
     left_arm.setSpeed(-10);

     delay(100);
}
*/
