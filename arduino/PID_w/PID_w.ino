#include<ros.h>
#include<std_msgs/Float64.h>
#include <Wire.h>
#include <stdlib.h>
#include "ti2c.h"
#include "ise_motor_driver.h"

const float Kp=0.001;
const float Ki=0;
const float Kd=0.1;
int pw=0;

float targetenc=0;
float P,preP,dt;
float I=0;
float D=0;
float duty=0;
float preTime=0;
float v=0;
uint8_t addr = 0x33;
IseMotorDriver m1 = IseMotorDriver(addr);
float enc = 0;
float preenc;

std_msgs::Float64 p;
ros::NodeHandle  nh;
ros::Publisher pub("PIDpub", &p);

void messageCb( const std_msgs::Float64& comand){
   targetenc=comand.data*11;
   p.data=targetenc;
   pub.publish( &p);
}
ros::Subscriber<std_msgs::Float64> sub("PIDwsub", messageCb );


void setup() {
  
  Wire.begin();
  Serial.begin(9600);
  /*nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  preTime=micros()/1000000;
  preenc = m1.encorder();*/
}

void loop() {
    
    enc = m1.encorder();
    /*P = targetenc - (enc-preenc);
    if(P>320)duty=20;
    else if(P<-320)duty=-20;
    else if(P<20&&P>-20)duty=0;
    else
   { dt = 0.1;
    preTime = micros();
    P = targetenc - (enc-preenc);
    I += P*dt;
    D = (P - preP)/dt;
    
    duty += Kp * P + Ki * I + Kd * D;

    
    if(duty>12){duty=12;}
    else if(duty<-12){duty=-12;}}
    pw=duty;
    preP = P;
    m1.setSpeed(pw);

     //Serial.print(micros());
     //Serial.print(" ");
    // Serial.println(enc);
     //Serial.println(P);
     //Serial.println(duty);
     nh.spinOnce();*/
     
    Serial.println(enc);
     delay(10);
}
