#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#define D 0.200//0.205//m
#define R 0.135//0.125//m
#define MOTOR_ONE_SPIN_PULSE 230000
#define GEAR_RATIO 3
#define WHEEL_ONE_SPIN_PULSE MOTOR_ONE_SPIN_PULSE * GEAR_RATIO

volatile long now_pulse[2] = {0,0};//{R,L}
volatile long target_pulse[2] = {0,0};//{R,L}
//volatile long before_pulse[2] = {0,0};//{R,L}
long delta_pulse[2];

double vx = 0.0;
double vth = 0.0;

double v[2] = {0,0};//{R,L}
double w[2] = {0,0};//{R,L}

long r_pulse_data = 0, l_pulse_data = 0;

void r_chatterCallback(const std_msgs::Int32::ConstPtr& r_pulse)
{
  //ROS_INFO("receive r_pulse: [%d]",r_pulse->data);
  r_pulse_data = r_pulse->data;
}

void l_chatterCallback(const std_msgs::Int32::ConstPtr& l_pulse)
{
  //ROS_INFO("receive l_pulse: [%d]",l_pulse->data);
  l_pulse_data = l_pulse->data;
}

void cmd_velCallback(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("receive vx: [%f]",vx);
  vx = msg.linear.x;
  vth = msg.angular.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel");
  ros::init(argc, argv, "r_delta_pub");
  ros::init(argc, argv, "l_delta_pub");
  
  ros::NodeHandle n;

  ros::Subscriber sub_r_pulse = n.subscribe("r_pulse", 100, r_chatterCallback);
  ros::Subscriber sub_l_pulse = n.subscribe("l_pulse", 100, l_chatterCallback);
  
  ros::Subscriber cmd_vel = n.subscribe("cmd_vel", 100, cmd_velCallback);

  ros::Publisher r_target_pub = n.advertise<std_msgs::Int32>("r_target_pub", 1000);
  ros::Publisher l_target_pub = n.advertise<std_msgs::Int32>("l_target_pub", 1000);

 ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);
  while(n.ok()){

    std_msgs::Int32 r_target_pulse;
    std_msgs::Int32 l_target_pulse;

    now_pulse[0] = r_pulse_data;
    now_pulse[1] = l_pulse_data;
    
    ros::spinOnce();   // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    //ROS_INFO("vx=%f",vx);
    //ROS_INFO("vth=%f",vth);

    v[0] = vx + D*vth;
    v[1] = vx - D*vth;

    for(int i=0; i<2; i++){
      w[i] = v[i] / R;
      delta_pulse[i] = w[i] * dt * WHEEL_ONE_SPIN_PULSE / (2*M_PI);
      //ROS_INFO("delta_pulse:[%d]",delta_pulse[i]);
      if(target_pulse[i] != now_pulse[i]){
	target_pulse[i] = now_pulse[i] + delta_pulse[i];
      }
      
    }

    r_target_pulse.data = target_pulse[0];
    l_target_pulse.data = target_pulse[1];
    
    r_target_pub.publish(r_target_pulse);
    l_target_pub.publish(l_target_pulse);

    //ROS_INFO("r_target_pulse:[%d]",target_pulse[0]);
    //ROS_INFO("l_target_pulse[%d]",target_pulse[1]);

    last_time = current_time;
     
    r.sleep();
  }





    
    
  

}

  
