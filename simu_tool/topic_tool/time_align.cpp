//
// Created by ray on 21-1-18.
//

//
// Created by ray on 20-11-9.
//
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

void imageHander(const sensor_msgs::ImageConstPtr &msg){
    double time=msg->header.stamp.toSec();
    ROS_INFO("imageHander time:%f",time);
}
void poseHander(const geometry_msgs::PoseStampedConstPtr &msg){
    double time=msg->header.stamp.toSec();
    ROS_INFO("poseHander time:%f",time);
}
int main(int argc,char** argv)
{
    ros::init(argc, argv, "time_aligin");

    ros::NodeHandle node;

    ros::Subscriber image_sub= node_.subscribe<sensor_msgs::Image>("/sdf_map/depth",1,&imageHander);
    ros::Subscriber pose_sub= node_.subscribe<geometry_msgs::PoseStamped>("/sdf_map/pose",1,&poseHander);
    ros::Rate rate(100.0);
    while(node.ok())
    {

        rate.sleep();
        ros::spinOnce();

    }
    return 0;
}