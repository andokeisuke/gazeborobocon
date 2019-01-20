#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>



#define MAX_VEL 0.5//m/s
#define MIN_VEL -0.5//m/s
#define MAX_ANGULAR_VEL 2.3//m/s
#define MIN_ANGULAR_VEL -2.3//m/s

#define  SERVO_WAIT  1
#define  SERVO_PREPARE  2
#define  SERVO_GEREGE_PASS  3
#define  SERVO_SHAGAI_GET  4
#define  SERVO_SHAGAI_SHOOT  5
#define  SERVO_CLOSE  6

ros::Publisher servo_task_pub;


int servo_state = SERVO_WAIT;


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
	float air_servo;

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

  Joystick.air_servo = joy.buttons[3];//右の4番 

}


void servoTaskCallback(const std_msgs::Int8::ConstPtr& m){
  //ROS_INFO("stcb");
  servo_state = m->data;
}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{//arduinoのmap関数と同じ
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main(int argc, char** argv)
{
	int servo_count = 0;
	float pre_servo_button_state = 0;

	ros::init(argc, argv, "joy");
	ros::NodeHandle n;

	ros::Subscriber joy_sub = n.subscribe("joy", 1, joy_Callback);
	ros::Subscriber servo_sub = n.subscribe("servo_task", 1, servoTaskCallback);


	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("sub",1);
	ros::Publisher arm_deg_pub = n.advertise<std_msgs::Int16>("tar_arm_deg",1);
	ros::Publisher gerege_cylinder_pub = n.advertise<std_msgs::Int16>("gerege_cylinder",1);
	servo_task_pub = n.advertise<std_msgs::Int8>("servo_task",1);



	ros::Rate r(10.0);

	std_msgs::Int16 deg;
	std_msgs::Int16 gerege_cylinder;
	std_msgs::Int8 servo_task;

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


		if(pre_servo_button_state == 0 && Joystick.air_servo == 1 && servo_state == SERVO_WAIT)
		{
			switch (servo_count)
			{
				case 0:
					servo_task.data = SERVO_GEREGE_PASS;
					servo_task_pub.publish(servo_task);		
					servo_state = SERVO_GEREGE_PASS;
					servo_count = 1;
					break;

				case 1:	
					servo_task.data = SERVO_SHAGAI_GET;
					servo_task_pub.publish(servo_task);		
					servo_state = SERVO_SHAGAI_GET;
					servo_count = 2;
					break;

				case 2:	
					servo_task.data = SERVO_SHAGAI_SHOOT;
					servo_task_pub.publish(servo_task);		
					servo_state = SERVO_SHAGAI_SHOOT;
					servo_count = 1;
					break;


			}


		}

		arm_deg_pub.publish(deg);

		twist_pub.publish(twist);

		gerege_cylinder_pub.publish(gerege_cylinder);

		pre_servo_button_state = Joystick.air_servo;
		r.sleep();

//		ROS_INFO("servo_state = %d",servo_state);
//		ROS_INFO("pre_servo_button_state = %f",pre_servo_button_state);
//		ROS_INFO("Joystick.air_servo = %f",Joystick.air_servo);


	}

}
