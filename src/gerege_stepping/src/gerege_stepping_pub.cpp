#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <gerege_stepping/gerege_stepping_msg.h>
#include <sstream>

int PIN = 9; //PIN番号   ←なんのピン？
int CW = 10; //CW番号    ←CWのピン番号？
float PW = 1.5; //パルス幅
bool DIR = true; //回転方向
int TIM = 50; //動かす時間

int Cylinder_state = 0;

void Cylinder_Callback(const std_msgs::Int16 & cylinder){

  Cylinder_state = cylinder.data;//左のスティック左右
//  ROS_INFO(" o: [%d]",Cylinder_state);

}


int main(int argc,char **argv){
	
	//node name "gerege_stepping"
	ros::init(argc,argv,"gerege_stepping"); 
	ros::NodeHandle nh;

	//publisher name "pub", select message type "gerege_stepping_msg", topic name "stepping_var"

	ros::Subscriber gerege_sub = nh.subscribe("gerege_cylinder", 1, Cylinder_Callback);
	ros::Publisher pub = nh.advertise <gerege_stepping::gerege_stepping_msg>("stepping_var",1);

	ros::Rate loop_rate(10); //loop 10Hz

		//message name is "msg"
		gerege_stepping::gerege_stepping_msg msg; 

		//var
		msg.pin = PIN;
		msg.cw = CW;
		msg.pw = PW;
		msg.dir = DIR;
		msg.tim = TIM;


	while(ros::ok()){
		ros::spinOnce();

		if(Cylinder_state == 0)
		{
			msg.tim = 0;
		}
		else if(Cylinder_state == 1)
		{
			msg.dir = true;
			msg.tim = 50;
		}
		else if(Cylinder_state == -1)
		{
			msg.dir = false;
			msg.tim = 50;
		}

		pub.publish(msg); //publish msg


		loop_rate.sleep();
	}
	return 0;
}
