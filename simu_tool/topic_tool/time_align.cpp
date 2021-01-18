//
// Created by ray on 21-1-18.
//

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
bool getPose=false;
bool getImage=false;
ros::Time first_pose;
ros::Time first_image;
ros::Publisher aligin_image_pub;
void imageHander(const sensor_msgs::ImageConstPtr &msg){
    if(!getImage||!getPose){
        first_image=msg->header.stamp;
        getImage=true;
    }
    double time=msg->header.stamp.toSec();
    //ROS_INFO("imageHander time:%f",time);

    double after_aligin;
    if(getPose&&getImage){
        ros::Duration offset_time=first_pose-first_image;
        ros::Time aligin_time=msg->header.stamp+offset_time;
        after_aligin=aligin_time.toSec();
        //ROS_INFO("imageHander after_aligin:%f",after_aligin);
        sensor_msgs::Image align_msg=*msg;
        align_msg.header.stamp=aligin_time;
        aligin_image_pub.publish(align_msg);
    }
}
void poseHander(const geometry_msgs::PoseStampedConstPtr &msg){
    if(!getImage||!getPose){
        getPose=true;
        first_pose=msg->header.stamp;
    }
    double time=msg->header.stamp.toSec();
    //ROS_INFO("poseHander time:%f",time);
}
int main(int argc,char** argv)
{
    ros::init(argc, argv, "time_aligin");

    ros::NodeHandle node;
    ros::Subscriber image_sub= node.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw",1,&imageHander);
    ros::Subscriber pose_sub= node.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",1,&poseHander);
    aligin_image_pub = node.advertise<sensor_msgs::Image>("/camera/depth/image_rect_raw/aligin",10);
    ros::Rate rate(1000.0);
    while(node.ok())
    {

        rate.sleep();
        ros::spinOnce();

    }
    return 0;
}