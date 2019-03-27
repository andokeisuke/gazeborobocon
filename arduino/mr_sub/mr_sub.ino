#include <Wire.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <custom_msg/valve_msg.h>
#include "kinjo_stp_motor_driver.h"

const uint8_t gerege_st_addr = 0x33;
const int servoSum = 7;
const int close_angle = 0;
const int open_angle = 90;
const int valveSum = 2;
const int valvePin[valveSum] = {31, 33} ;

Servo servos[servoSum];
KinjoStMotorDriver gerege_motor = KinjoStMotorDriver(gerege_st_addr);


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

void gerege_stpCB(const std_msgs::Int16 &deg)
{
  gerege_motor.rotate_degree(deg.data);
}

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Int16MultiArray>servo_sub("servo_deg", &servoDegCB);
ros::Subscriber<std_msgs::Int16>gerege_stp_sub("gerege_stepping_deg", &gerege_stpCB);
ros::Subscriber<custom_msg::valve_msg>valve_op_sub("valve_op", &valveOpCB);

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

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  nh.initNode();
  nh.subscribe(servo_sub);
  nh.subscribe(gerege_stp_sub);
  nh.subscribe(valve_op_sub);

  initServos();
}

void loop() {
  nh.getHardware()->setBaud(57600);

  // put your main code here, to run repeatedly:
  delay(5);
  nh.spinOnce();

}
