#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>



#define MAX_VEL 0.5//m/s
#define MIN_VEL -0.5//m/s
#define MAX_ANGULAR_VEL 2.3//m/s
#define MIN_ANGULAR_VEL -2.3//m/s


class Joystick
{
	public:
		float linear_x;
		float linear_y;
		float right_spin;
		float left_spin;
		float arm_up;
		float arm_down;
};

Joystick Joystick;

void joy_Callback(const sensor_msgs::Joy& joy){

  Joystick.linear_x = joy.axes[1];//
  Joystick.linear_y = joy.axes[0];//

  Joystick.right_spin = joy.buttons[4];//
  Joystick.left_spin  = joy.buttons[5];//

  Joystick.arm_up = joy.buttons[1];//
  Joystick.arm_down  = joy.buttons[2];//

}


float map(float x, float in_min, float in_max, float out_min, float out_max)
{//arduinoのmap関数と同じ
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy");
  ros::NodeHandle n;
  
  ros::Subscriber joy_sub = n.subscribe("joy", 1, joy_Callback);


  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("sub",1);
  ros::Publisher arm_deg_pub = n.advertise<std_msgs::Int16>("tar_arm_deg",1);

  ros::Rate r(10.0);
  while(n.ok()){

    ros::spinOnce();

    geometry_msgs::Twist twist;
    std_msgs::Int16 deg;


    twist.linear.x = map(Joystick.linear_x,-1,1,MIN_VEL,MAX_VEL);
    twist.linear.y = map(Joystick.linear_y,-1,1,MIN_VEL,MAX_VEL);
	
	if(Joystick.right_spin == 1)
	{
		twist.angular.z = MAX_ANGULAR_VEL;
	}    
	else if(Joystick.left_spin == 1)
	{
		twist.angular.z = MIN_ANGULAR_VEL;
	}    

	if(Joystick.arm_up == 1)
	{
		deg.data = 0;
	}    
	else if(Joystick.arm_down == 1)
	{
		deg.data = 90;
	}    
    
    arm_deg_pub.publish(deg);
    
    twist_pub.publish(twist);

    r.sleep();
  }
  
}
