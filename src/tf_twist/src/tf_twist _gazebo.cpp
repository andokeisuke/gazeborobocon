#include <ros/ros.h>
#include <std_msgs/Float64.h>

//pub side
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sstream>

#define WHEEL_RADIUS 0.05

class Wheel
{
public:
	float right_rear;
	float right_front;
	float left_rear;
	float left_front;
};

Wheel speed;
Wheel steerring;

int thetachange(float x,float y);
ros::Subscriber sub;
ros::Publisher	base_pub,
				left_front_steering_pub,
				left_rear_steering_pub,
				right_front_steering_pub,
				right_rear_steering_pub,
				left_front_wheel_pub,
				left_rear_wheel_pub,
				right_front_wheel_pub,
				right_rear_wheel_pub;

void messageCallback(const geometry_msgs::Twist::ConstPtr& msg){
	//pub side
	std_msgs::Float64 v,theta;
	float left_front_theta = 0;
	float left_rear_theta = 0;
	float right_front_theta = 0;
	float right_rear_theta = 0;
	float temp_theta = 0;
	float temp_v = 0;

	float linear_vel , angular_vel;

	linear_vel  = sqrt((msg->linear.x)*(msg->linear.x)+(msg->linear.y)*(msg->linear.y));
	angular_vel = msg->angular.z;

	if(angular_vel == 0)
	{

		temp_theta = atan2(msg->linear.y,msg->linear.x);
		/*
		if(msg->linear.y<0){
			linear_vel = -linear_vel;
		}
		*/

		temp_v = linear_vel / WHEEL_RADIUS;

		base_pub.publish(theta);
		theta.data = temp_theta + M_PI/4;
		left_front_steering_pub.publish(theta);

		theta.data = temp_theta - M_PI/4;
		left_rear_steering_pub.publish(theta);

		theta.data = temp_theta + M_PI/4;
		right_rear_steering_pub.publish(theta);

		theta.data = temp_theta - M_PI/4;
		right_front_steering_pub.publish(theta);

		v.data = temp_v;
		right_front_wheel_pub.publish(v);
		right_rear_wheel_pub.publish(v);

		v.data = -temp_v;
		left_front_wheel_pub.publish(v);
		left_rear_wheel_pub.publish(v);

	}
	else
	{

		v.data = angular_vel / WHEEL_RADIUS;
		
		theta.data = 0;
		left_rear_steering_pub.publish(theta);
		right_front_steering_pub.publish(theta);
		left_front_steering_pub.publish(theta);
		right_rear_steering_pub.publish(theta);

		left_front_wheel_pub.publish(v);
		left_rear_wheel_pub.publish(v);
		right_front_wheel_pub.publish(v);
		right_rear_wheel_pub.publish(v);

	}
}

/*
void publish_topic()
{
	base_pub.publish(theta);
	left_front_steering_pub.publish(steering.left_front);
	left_rear_steering_pub.publish(steering.left_rear);
	right_front_steering_pub.publish(steering.right_front);
	right_rear_steering_pub.publish(steering.right_rear);
	left_front_wheel_pub.publish(speed.left_front);
	left_rear_wheel_pub.publish(speed.left_rear);
	right_front_wheel_pub.publish(speed.right_front);
	right_rear_wheel_pub.publish(speed.right_rear);
};
*/
int main(int argc,char **argv){
	ros::init(argc,argv,"sub_pub");
	ros::NodeHandle nh;

	sub = nh.subscribe("sub",10,messageCallback);

	base_pub = nh.advertise<std_msgs::Float64>("/my_robo/base_controller/command",1);
	left_front_steering_pub = nh.advertise<std_msgs::Float64>("/my_robo/left_front_steering_controller/command",1);
	left_rear_steering_pub = nh.advertise<std_msgs::Float64>("/my_robo/left_rear_steering_controller/command",1);
	right_front_steering_pub = nh.advertise<std_msgs::Float64>("/my_robo/right_front_steering_controller/command",1);
	right_rear_steering_pub = nh.advertise<std_msgs::Float64>("/my_robo/right_rear_steering_controller/command",1);
	left_front_wheel_pub = nh.advertise<std_msgs::Float64>("/my_robo/left_front_wheel_controller/command",1);
	left_rear_wheel_pub = nh.advertise<std_msgs::Float64>("/my_robo/left_rear_wheel_controller/command",1);
	right_front_wheel_pub = nh.advertise<std_msgs::Float64>("/my_robo/right_front_wheel_controller/command",1);
	right_rear_wheel_pub = nh.advertise<std_msgs::Float64>("/my_robo/right_rear_wheel_controller/command",1);

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
