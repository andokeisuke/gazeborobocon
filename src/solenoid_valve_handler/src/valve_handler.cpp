#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>

#include <custom_msg/valve_msg.h>
#include <std_msgs/Int8.h>

//#include "conf.h"
#define  VALVE_WAIT         1
#define  VALVE_SHAGAI_PUSH  2
#define  VALVE_SHAGAI_PULL  3

#define  VALVE_GEREGE_GET   4
#define  VALVE_GEREGE_PASS  5

#define  VALVE_GEREGE_PUSH  6
#define  VALVE_GEREGE_PULL  7

// params
const int valveSum = 6;                                           
const int hz = 10;
const int a = 0, b = 1, c = 2, d = 3, e = 4, f = 5;//
const int delaySmall = 100 , delayLong = 2000 ;

// inner values
bool delaying = false; // for delay
bool final_delay = false;
int delayCounter = 0;
int state = VALVE_WAIT;

// ros values
ros::Publisher valve_op_Pub;
ros::Publisher valve_task_Pub;


ros::Subscriber valveSub;

custom_msg::valve_msg valve_op;
std_msgs::Int8 valve_task_state;


// =========== callback ==========
void valveTaskCallback(const std_msgs::Int8::ConstPtr& m){
  //ROS_INFO("stcb");
  state = m->data;
}

// ===========sub func==========
void sendValve_op(bool state, int number) {
  valve_op.state = state;
  valve_op.valve_number = number;
  valve_op_Pub.publish(valve_op);
}

void valveOpen(int valveNo) {
  sendValve_op(false,valveNo);  
}

void valveClose(int valveNo) {
  sendValve_op(true,valveNo);  
}

void delayCount() {
  if (delayCounter > 0) {
    delayCounter--;
  } else {
    delaying = false;

    if(final_delay == true)
    {
      valve_task_state.data = VALVE_WAIT;
      valve_task_Pub.publish(valve_task_state);
      sendValve_op(false,-1);//all close
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
void Shagai_push(){
    static int mode = 0;
    if (mode == 0) {
      valveOpen(e);
      valveClose(f);
      delay(delaySmall);
      final_delay = true;
      mode = 0;
    }
}

void Shagai_pull(){
    static int mode = 0;
    if (mode == 0) {
      valveOpen(f);
      valveClose(e);
      delay(delaySmall);
      final_delay = true;
      mode = 0;
    }
}

void Gerege_push(){
    static int mode = 0;
    if (mode == 0) {
      valveOpen(a);
      valveClose(b);
      delay(delaySmall);
//      final_delay = true;
      mode = 0;
    }
}

void Gerege_pull(){
    static int mode = 0;
    if (mode == 0) {
      valveOpen(b);
      valveClose(a);
      delay(delaySmall);
//      final_delay = true;
      mode = 0;
    }
}

void Gerege_get(){
    static int mode = 0;
    if (mode == 0) {
      valveOpen(c);
      valveClose(d);
      delay(delaySmall);
      final_delay = true;
      mode = 0;
    }
}

void Gerege_pass(){
    static int mode = 0;
    if (mode == 0) {
      Gerege_push();
      delay(delaySmall);
      mode = 1;
    }
    else if(mode == 1)
    {
      valveOpen(d);
      valveClose(c);
      delay(delaySmall);
      mode = 2;
    }
    else if(mode == 2)
    {
      Gerege_pull();
      final_delay = true;
      delay(delaySmall);
      mode = 0;
    }
}

void task() {
  //  ROS_INFO("task");
  if (state == VALVE_WAIT) {
	ROS_INFO("Wait\n");
    //pass
  } else if (state == VALVE_SHAGAI_PUSH) {
    ROS_INFO("shgai_push\n");
    Shagai_push();
  } else if (state == VALVE_SHAGAI_PULL) {
    ROS_INFO("shagai_pull\n");
    Shagai_pull();
  } else if (state == VALVE_GEREGE_PUSH) {
    ROS_INFO("gerege_push\n");
    Gerege_push();
  } else if (state == VALVE_GEREGE_PULL) {
    ROS_INFO("gerege_pull\n");
    Gerege_pull();
  } else if (state == VALVE_GEREGE_GET) {
    ROS_INFO("gerege_get\n");
    Gerege_get();
  } else if (state == VALVE_GEREGE_PASS) {
    ROS_INFO("gerege_pass\n");
    Gerege_pass();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "valve_handler");
  ros::NodeHandle n; 

  valve_op_Pub = n.advertise<custom_msg::valve_msg>("valve_op", 1);
  valve_task_Pub = n.advertise<std_msgs::Int8>("valve_task", 1);


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