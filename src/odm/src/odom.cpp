#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include<gazebo_msgs/LinkStates.h>

    double state_odom_x=0.0;//オドメトリX座標[m]
    double state_odom_y=0.0;//オドメトリY座標[m]
    double state_odom_th=0.0; //オドメトリ姿勢[rad]
geometry_msgs::Quaternion odom_quat ;
void chatterCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
   {
     state_odom_x=msg->pose[2].position.x;
     state_odom_y=msg->pose[2].position.y;
     odom_quat.x=msg->pose[2].orientation.x;
     odom_quat.y=msg->pose[2].orientation.y;
     odom_quat.z=msg->pose[2].orientation.z;
     odom_quat.w=msg->pose[2].orientation.w;

   }


int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    current_time = ros::Time::now();
    ros::Subscriber sub = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, chatterCallback);
    ros::Rate r(30.0);



    while(n.ok()){

    current_time = ros::Time::now();

    //tf odom->base_link
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = state_odom_x;
    odom_trans.transform.translation.y = state_odom_y;
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    ros::spinOnce();
    r.sleep();
    }
}
