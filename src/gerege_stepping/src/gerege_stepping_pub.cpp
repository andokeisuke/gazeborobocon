#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

const int hz = 10;
const int delaySmall = 1000;

bool gerege_pass_state = false;

// inner values
bool delaying = false; // for delay
int delayCounter = 0;

ros::Subscriber gerege_state_sub;
ros::Publisher deg_pub;
ros::Publisher gerege_state_pub;

std_msgs::Int16 deg; 
std_msgs::Bool state; 


void delayCount() {
  if (delayCounter > 0) {
    delayCounter--;
  } else {
    delaying = false;
    state.data = false;
    gerege_state_pub.publish(state);
  }
}

void delay(int ms) {
  ROS_INFO("delay");
  delayCounter = (ms*hz/1000);
  delaying = true;  
}


void Gerege_pass_Callback(const std_msgs::Bool &state){
  gerege_pass_state = state.data;//左のスティック左右
//  ROS_INFO(" o: [%d]",Cylinder_state);
}

void Gerege_pass(){
    static int mode = 0;
    if (mode == 0) {
      //ROS_INFO("Gerege_pass");
      // shot
      deg.data = 90;
      deg_pub.publish(deg); //publish msg
      delay(delaySmall);
      mode = 1;
    } else if (mode ==1) {
      // shot done
      deg.data = 0;
      deg_pub.publish(deg); //publish msg
      delay(delaySmall);
      mode = 0;
    } 
}

void task() {
  //  ROS_INFO("task");
  if (gerege_pass_state == false) {
	ROS_INFO("Wait\n");
    //pass
  } else if (gerege_pass_state == true) {
    ROS_INFO("gerege_pass\n");
    Gerege_pass();
  }
}

int main(int argc,char **argv){
	
	//node name "gerege_stepping"
	ros::init(argc,argv,"gerege_stepping"); 
	ros::NodeHandle nh;

	//publisher name "pub", select message type "gerege_stepping_msg", topic name "stepping_var"

	gerege_state_sub = nh.subscribe("gerege_pass_state", 1,Gerege_pass_Callback);

	gerege_state_pub = nh.advertise <std_msgs::Bool>("gerege_pass_state", 1);
	deg_pub = nh.advertise <std_msgs::Int16>("gerege_stepping_deg",1);

	ros::Rate loop_rate(hz); //loop 10Hz

		//message name is "msg"
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