#include"ros/ros.h"
#include"std_msgs/Int8.h"
#include"std_msgs/Int16.h"
#include"std_msgs/Float64.h"
#include"std_msgs/Bool.h"
#include<gerege_stepping/gerege_stepping_msg.h>
#include<sstream>

int PIN = 9; //PIN番号
int CW = 10; //CW番号
float PW = 1.5; //パルス幅
bool DIR = true; //回転方向
int TIM = 4000; //動かす時間

int main(int argc,char **argv){
	
	//node name "gerege_stepping"
	ros::init(argc,argv,"gerege_stepping"); 
	ros::NodeHandle nh;

	//publisher name "pub", select message type "gerege_stepping_msg", topic name "stepping_var"
	ros::Publisher pub = nh.advertise <gerege_stepping::gerege_stepping_msg>("stepping_var",1000);

	ros::Rate loop_rate(10); //loop 10Hz
	while(ros::ok()){

		//message name is "msg"
		gerege_stepping::gerege_stepping_msg msg; 

		//var
		msg.pin = PIN;
		msg.cw = CW;
		msg.pw = PW;
		msg.dir = DIR;
		msg.tim = TIM;

		pub.publish(msg); //publish msg

		ros::spinOnce;
		loop_rate.sleep();
	}
	return 0;
}
