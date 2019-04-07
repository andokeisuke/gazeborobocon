#include <Wire.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <custom_msg/valve_msg.h>
#include <std_msgs/Bool.h>

#include "ise_motor_driver.h"

#define TOUCH_SENSOR_PIN A2
#define ENC_PER_DEG_arm 11//アームを上げるための１度のエンコーダの値


const int servoSum = 4;
const int close_angle = 40;
const int open_angle = 140;
const int valveSum = 6;
const int valvePin[valveSum] = {30, 32,34,36,38,40} ;
bool shagai_get_state = false;

Servo servos[servoSum];
IseMotorDriver arm_roll = IseMotorDriver(0x13);//13 
IseMotorDriver arm_get_shagai = IseMotorDriver(0x30);//30 


void servoDegCB(const std_msgs::Int16MultiArray& array)
{
  int index = array.data[0];
  int deg = array.data[1];
  if (index == -1)
  {
    servo_detach();
  }
  else
  {
    servos[index].attach(index + 6); //pin: 6~12
    servos[index].write(deg);
  }
}

void arm_roll_cb(const std_msgs::Int16& msg)
{
  arm_roll.setSpeed(msg.data);
}

void arm_get_shagai_cb(const std_msgs::Int16& msg)
{
  arm_get_shagai.setSpeed(msg.data);  
  if(msg.data > 0)
  {
      shagai_get_state = true;
  }

}

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Int16MultiArray>servo_sub("servo_deg", &servoDegCB);
ros::Subscriber<custom_msg::valve_msg>valve_op_sub("valve_op", &valveOpCB);
ros::Subscriber<std_msgs::Int16>arm_roll_sub("arm_pow",arm_roll_cb);
ros::Subscriber<std_msgs::Int16>arm_get_shagai_sub("shagai_get",arm_get_shagai_cb);

void initServos() {
  int i = 0;
  for (i = 0; i < servoSum; i++) {
    servos[i] = Servo();
    servos[i].attach(i + 6); //pin: 6~12
    servos[i].write(close_angle);
    delay(1000);
    servos[i].detach();
  }
}

void valveOpCB(const custom_msg::valve_msg &data)
{
  bool state = data.state;
  int number = data.valve_number;
  int i;
  if (number == -1)
  {
    for (i = 0; i < valveSum;i++)
    {
      digitalWrite(valvePin[i] , LOW);
    }
  }
  else
  {
    digitalWrite(valvePin[number] , state);
  }
}


void servo_detach()
{
  int i = 0;
  for (i = 0; i < servoSum; i++) {
    servos[i].detach();
  }
}

void init_valve() {
  int i = 0;
  for (i = 0; i < valveSum; i++)
  {
    pinMode(valvePin[i], OUTPUT);
    digitalWrite(valvePin[i], LOW);
  }
}


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  nh.initNode();
  nh.subscribe(servo_sub);
  nh.subscribe(valve_op_sub);
  nh.subscribe(arm_roll_sub);
  nh.subscribe(arm_get_shagai_sub);

  nh.getHardware()->setBaud(57600);

  initServos();
  init_valve();
  
  pinMode(TOUCH_SENSOR_PIN,INPUT_PULLUP);

}

void loop() {
    int sensor_state = digitalRead(TOUCH_SENSOR_PIN);
    if(sensor_state == 1 && shagai_get_state == true)
    {
        arm_get_shagai.setSpeed(0);
        shagai_get_state = false;  
    }
  // put your main code here, to run repeatedly:
  delay(5);
  nh.spinOnce();

}
