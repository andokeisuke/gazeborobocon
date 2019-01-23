#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <gerege_stepping/gerege_stepping_msg.h>
#include <FlexiTimer2.h>

ros::NodeHandle nh;
gerege_stepping::gerege_stepping_msg msg;   

int pin; //ピン番号
int cw;  //CW
float pw; //パルス幅
bool dir; //回転方向
int tim; //動かす時間
bool check;

void messageCallback(const gerege_stepping::gerege_stepping_msg& msg){
  dir = msg.dir;
  if(check==dir){
    pin = msg.pin;
    cw = msg.cw;
    pw = msg.pw;
    tim = msg.tim;

    pinMode(pin,OUTPUT);
    pinMode(cw,OUTPUT);
    digitalWrite(cw,dir);
    FlexiTimer2::set(pw,1/10,stepp);
    FlexiTimer2::start();

    delay(tim);
    FlexiTimer2::stop();
    check = !dir;
  
//    pub.publish(&msg);
  }
}

void stepp(){
  static  boolean output = HIGH;
  digitalWrite(pin,output);
  output = !output;
}

ros::Subscriber<gerege_stepping::gerege_stepping_msg> sub("stepping_var", messageCallback);

void setup(){
  nh.initNode();
  nh.subscribe(sub); 

  check=dir;
}

void loop(){
  nh.spinOnce();
}
