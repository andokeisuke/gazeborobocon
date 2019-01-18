#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>



#define MAX_VEL 0.5//m/s
#define MIN_VEL -0.5//m/s
#define MAX_ANGULAR_VEL 2.3//m/s
#define MIN_ANGULAR_VEL -2.3//m/s



float linear_joy;
float angular_joy;

std_msgs::Int16 left_pw;
std_msgs::Int16 right_pw;


void joy_Callback(const sensor_msgs::Joy& joy){
  linear_joy = joy.axes[1];//ps -> 2 ,blue -> 1
  angular_joy = joy.axes[2];//ps -> 4, blue -> 2
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

/*
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    
  ros::Rate r(10.0);
  while(n.ok()){

    ros::spinOnce();

    geometry_msgs::Twist twist;
    twist.linear.x = map(linear_joy,-1,1,MIN_VEL,MAX_VEL);
    twist.angular.z = map(angular_joy,-1,1,MIN_ANGULAR_VEL,MAX_ANGULAR_VEL);
    
    twist_pub.publish(twist);

    r.sleep();
  }
*/


  ros::Publisher l_power_pub = n.advertise<std_msgs::Int16>("left_pw", 1000);
  ros::Publisher r_power_pub = n.advertise<std_msgs::Int16>("right_pw", 1000);

  ros::Rate r(10.0);
  while(n.ok()){

    ros::spinOnce();
    if(angular_joy>0)
    {
      right_pw.data = map(angular_joy,-1,1,-10,10);
      left_pw.data=-1*map(angular_joy,-1,1,-10,10);
    }
    else if(angular_joy<0)
    {
      left_pw.data = -1*map(angular_joy,-1,1,-10,10);
      right_pw.data = map(angular_joy,-1,1,-10,10);
    }
    else
    {
      right_pw.data = map(linear_joy,-1,1,-10,10);
      left_pw.data = map(linear_joy,-1,1,-10,10);
    }

    //left_pw.data = map(angular_joy,-1,1,-100,100);
    
    r_power_pub.publish(right_pw);
    l_power_pub.publish(left_pw);

    r.sleep();
  } 

}
