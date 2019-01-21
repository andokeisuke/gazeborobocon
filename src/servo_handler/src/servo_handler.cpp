#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include "std_msgs/Int8.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int16MultiArray.h"
//#include "conf.h"
#define  SERVO_WAIT  1
#define  SERVO_PREPARE  2
#define  SERVO_GEREGE_PASS  3
#define  SERVO_SHAGAI_GET  4
#define  SERVO_SHAGAI_SHOOT  5
#define  SERVO_CLOSE  6

// params
const int servoSum = 7;
const int closeDeg = 0; // close deg
const int openDeg = 90;   // open deg
const int hz = 10;
const int a = 0, b = 6, c = 5, d = 4, e = 3, f = 2, g = 1,servo_detach_flag = -1;
const int delaySmall = 1000, delayMedium = 3000, delayLong = 1500, delayShot = 1000, delayReset = 3000, delay2000 = 2000;

// inner values
bool delaying = false; // for delay
bool final_delay = false;
int delayCounter = 0;
int state = SERVO_WAIT;

// ros values
ros::Publisher pub;
ros::Publisher servoPub;
ros::Subscriber joy;
ros::Subscriber servoSub;
std_msgs::Int16MultiArray array;
std_msgs::Int8 servop;

// =========== callback ==========
void servoTaskCallback(const std_msgs::Int8::ConstPtr& m){
  //ROS_INFO("stcb");
  state = m->data;
}

// ===========sub func==========
void sendArr(int servoNo, int degree) {
  array.data.clear();
  array.data.push_back(servoNo);
  array.data.push_back(degree);
  pub.publish(array);
}

void sOpen(int servoNo) {
  sendArr(servoNo, openDeg);  
}

void sClose(int servoNo) {
  sendArr(servoNo, closeDeg);  
}

void delayCount() {
  if (delayCounter > 0) {
    delayCounter--;
  } else {
    delaying = false;

    if(final_delay == true)
    {
      state = SERVO_WAIT;
      sendArr(servo_detach_flag,0);  
      servop.data=state;
      servoPub.publish(servop);
      final_delay = false;
    }

  }
}

void delay(int ms) {
  ROS_INFO("delay");
  delayCounter = (ms*hz/1000);
  delaying = true;  
}

// =========== routine ==========
void setup() {
  ROS_INFO("setup");
  int i = 0;
  for (i = 0; i < servoSum; i++) {
    sClose(i);
  }
}

void prepare() {
  // ready for shooting shygai
  static int mode = 0;
   if(mode==0){
    ROS_INFO("prepare");
    sOpen(e);
    sOpen(a);
    delay(delaySmall);
    mode = 1;
  } else if (mode == 1) {
    sOpen(d);
    delay(delaySmall);
    mode = 2;
  } else if (mode == 2) {
    sClose(d);
    delay(delaySmall);
    mode = 3;
  } else if (mode == 3) {
    sOpen(c);
    sClose(d);
    delay(delaySmall);
    final_delay = true;
    mode = 0;
  } 
  
}

void Grege_pass(){
    static int mode = 0;
    if (mode == 0) {
      ROS_INFO("Gerege_pass");
      sOpen(g);
      delay(delaySmall);
      final_delay = true;
      mode = 0;
    }
}

void Shagai_get(){
    static int mode = 0;
    if (mode == 0) {
      ROS_INFO("Shagai_get");
      // shot
      sOpen(f);
      delay(delaySmall);
      mode = 1;
    } else if (mode ==1) {
      // shot done
      sClose(f);
      delay(delaySmall);
      final_delay = true;
      mode = 0;
    } 
}

void Shagai_shoot() {
    static int mode = 0;
    if (mode == 0) {
      ROS_INFO("Shagai_shoot");
      //flagup
      sOpen(b);
      delay(delaySmall);
      mode = 1;
    } else if (mode ==1) {
      // shot done
      sClose(b);
      delay(delaySmall);
      mode = 2;
    } else if (mode == 2) {
      // prepare exhaust
      sOpen(a);
      sClose(c);
      delay(delaySmall);
      mode = 3;
    } else if (mode == 3) {
    sOpen(d);
    delay(delaySmall);
    mode = 4;
  } else if (mode == 4) {
    sClose(d);
    delay(delaySmall);
    mode = 5;
  } else if (mode == 5) {
    sOpen(c);
    sClose(d);
    delay(delaySmall);
    final_delay = true;
    mode = 0;
  }
}

void all_close(){
	int dc = delaySmall;
	sClose(a);
	sClose(b);
	sClose(c);
	sClose(d);
	sClose(e);
	sClose(f);
	sClose(g);
  delay(delaySmall);
  final_delay = true;
}

void task() {
  //  ROS_INFO("task");
  if (state == SERVO_WAIT) {
	ROS_INFO("Wait\n");
    //pass
  } else if (state == SERVO_PREPARE) {
    ROS_INFO("prepare\n");
    prepare();
  } else if (state == SERVO_GEREGE_PASS) {
  ROS_INFO("Pass Gerege\n");
    Grege_pass();
  } else if (state == SERVO_SHAGAI_GET) {
  ROS_INFO("Get Shagai\n");
    Shagai_get();
  } else if (state == SERVO_SHAGAI_SHOOT) {
  ROS_INFO("Shoot Shagai\n");
    Shagai_shoot();
  } else if (state == SERVO_CLOSE) {
  ROS_INFO("close\n");
    all_close();
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_handler");
  ros::NodeHandle n; 
  pub = n.advertise<std_msgs::Int16MultiArray>("servo_deg", 1);
  servoPub = n.advertise<std_msgs::Int8>("servo_task", 1);
  servoSub = n.subscribe("servo_task", 1, servoTaskCallback);
  ros::Rate loop_rate(hz);
    
  while (ros::ok())
    {
      ros::spinOnce();
      if (delaying) {
	delayCount();
      } else {
	task();
      }
      loop_rate.sleep();
    }

  return 0;
}