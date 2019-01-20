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
		float gerege_push;
		float gerege_pull;

};

Joystick Joystick;

void joy_Callback(const sensor_msgs::Joy& joy){

  Joystick.linear_x = joy.axes[1];//左のスティック上下
  Joystick.linear_y = joy.axes[0];//左のスティック左右

  Joystick.right_spin = joy.buttons[4];//L2ボタン(コントローラの5番)
  Joystick.left_spin  = joy.buttons[5];//R2ボタン(コントローラの6番)

  Joystick.arm_up = joy.axes[3];//右のスティック上下

  Joystick.gerege_push = joy.buttons[2];//右の3番
  Joystick.gerege_pull = joy.buttons[1];//右の2番

 
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
  ros::Publisher gerege_cylinder_pub = n.advertise<std_msgs::Int16>("gerege_cylinder",1);



  ros::Rate r(10.0);

  std_msgs::Int16 deg;
  std_msgs::Int16 gerege_cylinder;


  while(n.ok()){

    ros::spinOnce();

    geometry_msgs::Twist twist;
  //  int temp_deg = 0;

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

	if(Joystick.arm_up == 0 &&Joystick.arm_down == 0)
	{
		deg.data = 0;
	}    
	else if(Joystick.arm_up == 1)
	{
		deg.data = -90;
	}    

	else if(Joystick.arm_down == 1)
	{
		deg.data = 90;
	}    

	if(Joystick.gerege_push == 0 &&Joystick.gerege_pull == 0)
	{
		gerege_cylinder.data = 0;
	}    
	else if(Joystick.gerege_push == 1)
	{
		gerege_cylinder.data = 1;
	}    

	else if(Joystick.gerege_pull == 1)
	{
		gerege_cylinder.data = -1;
	}    

    
    arm_deg_pub.publish(deg);
    
    twist_pub.publish(twist);

    gerege_cylinder_pub.publish(gerege_cylinder);

    r.sleep();
  }
  
}
