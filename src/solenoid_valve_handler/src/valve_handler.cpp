#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>

#include <custom_msg/valve_msg.h>
#include <std_msgs/Int8.h>

//#include "conf.h"
#define  VALVE_WAIT  1
#define  VALVE_PREPARE  2
#define  VALVE_GEREGE_PASS  3
#define  VALVE_SHAGAI_GET  4
#define  VALVE_SHAGAI_SHOOT  5
#define  VALVE_CLOSE  6

// params
const int servoSum = 7;                                           
const int hz = 10;
const int a = 0, b = 6, c = 5, d = 4, e = 3, f = 2, g = 1,valve_detach_flag = -1;//pin_name
const int delaySmall = 1000, delayMedium = 3000, delayLong = 1500, delayShot = 1000, delayReset = 3000, delay2000 = 2000;

// inner values
bool delaying = false; // for delay
bool final_delay = false;
int delayCounter = 0;
int state = VALVE_WAIT;

// ros values
ros::Publisher valvePub;

ros::Subscriber valveSub;

custom_msg::valve_msg valve_op;

// =========== callback ==========
void valveTaskCallback(const std_msgs::Int8::ConstPtr& m){
  //ROS_INFO("stcb");
  state = m->data;
}

// ===========sub func==========
void sendValve_op(bool state, int number) {
  valve_op.state = state;
  valve_op.valve_number = number;
  valvePub.publish(valve_op);
}

void sOpen(int valveNo) {
  sendValve_op(valveNo, true);  
}

void sClose(int valveNo) {
  sendValve_op(valveNo, false);  
}

void delayCount() {
  if (delayCounter > 0) {
    delayCounter--;
  } else {
    delaying = false;

    if(final_delay == true)
    {
      state = VALVE_WAIT;
      sendValve_op(-1,false);  
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
    sClose(a);
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
    sClose(a);
    delay(delaySmall);
    final_delay = true;
    mode = 0;
  }
}

void all_close(){
    static int mode = 0;
    if (mode == 0) {
      ROS_INFO("all_close");
      //flagup
      sClose(a);
      sClose(b);
      delay(delaySmall);
      mode = 1;
    } else if (mode ==1) {
      // shot done
      sClose(c);
      sClose(d);
      delay(delaySmall);
      mode = 2;
    } else if (mode == 2) {
      // prepare exhaust
      sClose(e);
      sClose(f);
      delay(delaySmall);
      mode = 3;
    } else if (mode == 3) {
    sClose(g);
    delay(delaySmall);
    final_delay = true;
    mode = 0;
  }
}

void task() {
  //  ROS_INFO("task");
  if (state == VALVE_WAIT) {
	ROS_INFO("Wait\n");
    //pass
  } else if (state == VALVE_PREPARE) {
    ROS_INFO("prepare\n");
    prepare();
  } else if (state == VALVE_GEREGE_PASS) {
  ROS_INFO("Pass Gerege\n");
    Grege_pass();
  } else if (state == VALVE_SHAGAI_GET) {
  ROS_INFO("Get Shagai\n");
    Shagai_get();
  } else if (state == VALVE_SHAGAI_SHOOT) {
  ROS_INFO("Shoot Shagai\n");
    Shagai_shoot();
  } else if (state == VALVE_CLOSE) {
  ROS_INFO("close\n");
    all_close();
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "valve_handler");
  ros::NodeHandle n; 

  valvePub = n.advertise<custom_msg::valve_msg>("valve_op", 1);

  valveSub = n.subscribe("valve_task", 1, valveTaskCallback);

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