#include <ros/ros.h>
#include <std_msgs/Float64.h>

//pub side
#include"ros/ros.h"
#include"std_msgs/String.h"
#include<geometry_msgs/Twist.h>
#include<math.h>
#include<sstream>
int thetachange(float x,float y);
ros::Subscriber sub;
ros::Publisher base_pub,base1_pub,base2_pub,base3_pub,base4_pub,wheel1_pub,wheel2_pub,wheel3_pub,wheel4_pub;

void messageCallback(const geometry_msgs::Twist::ConstPtr& msg){
	//pub side
	std_msgs::Float64 v,theta;

	theta.data= thetachange(msg->linear.x,msg->linear.y)*M_PI/180;
	v.data=sqrt((msg->linear.x)*(msg->linear.x)+(msg->linear.y)*(msg->linear.y));
	if(msg->linear.y<0){
		v.data=-v.data;

	}
	v.data=10000*v.data;
	  base_pub.publish(theta);
		base1_pub.publish(theta);
		base2_pub.publish(theta);
		base3_pub.publish(theta);
		base4_pub.publish(theta);
		wheel1_pub.publish(v);
		wheel2_pub.publish(v);
		wheel3_pub.publish(v);
		wheel4_pub.publish(v);







}

int main(int argc,char **argv){
	ros::init(argc,argv,"sub_pub");
	ros::NodeHandle nh;

	sub = nh.subscribe("sub",10,messageCallback);
	base_pub = nh.advertise<std_msgs::Float64>("/my_robo/base_controller/command",1);
	base1_pub = nh.advertise<std_msgs::Float64>("/my_robo/base1_controller/command",1);
	base2_pub = nh.advertise<std_msgs::Float64>("/my_robo/base2_controller/command",1);
	base3_pub = nh.advertise<std_msgs::Float64>("/my_robo/base3_controller/command",1);
	base4_pub = nh.advertise<std_msgs::Float64>("/my_robo/base4_controller/command",1);
	wheel1_pub = nh.advertise<std_msgs::Float64>("/my_robo/wheel1_controller/command",1);
	wheel2_pub = nh.advertise<std_msgs::Float64>("/my_robo/wheel2_controller/command",1);
	wheel3_pub = nh.advertise<std_msgs::Float64>("/my_robo/wheel3_controller/command",1);
	wheel4_pub = nh.advertise<std_msgs::Float64>("/my_robo/wheel4_controller/command",1);

	ros::spin();
	return 0;
}




int thetachange(float x,float y){
	int theta = atan(y/x)*180/M_PI;

	if((x<=0&&y>=0) || (x>0&&y<0)){

		theta = theta + 180;
	}

	if((0<=theta&&theta<=3&&x>0)||(177<=theta&&theta<=180&&x>0)){

		theta = 0;
	}

	if( (0<=theta&&theta<=3&&x<0)||(177<=theta&&theta<=180&&x<0)){
		theta = 180;

	}


	return theta;

}
