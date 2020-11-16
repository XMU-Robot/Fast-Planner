/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "plan_manage/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/CompanionProcessStatus.h>
enum class MAV_STATE {
    MAV_STATE_UNINIT,
    MAV_STATE_BOOT,
    MAV_STATE_CALIBRATIN,
    MAV_STATE_STANDBY,
    MAV_STATE_ACTIVE,
    MAV_STATE_CRITICAL,
    MAV_STATE_EMERGENCY,
    MAV_STATE_POWEROFF,
    MAV_STATE_FLIGHT_TERMINATION,
};

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;
ros::Publisher trajectory_pub,mavros_system_status_pub,mavros_cmd_vel_pub;
nav_msgs::Odometry odom;

bool pub_flag=false;
quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {5.7, 5.7, 6.2};
// double vel_gain[3] = {3.4, 3.4, 4.0};
double pos_gain[3] = { 5.7, 5.7, 6.2 };
double vel_gain[3] = { 3.4, 3.4, 4.0 };

using fast_planner::NonUniformBspline;

bool receive_traj_ = false;
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_;
double time_forward_;

vector<Eigen::Vector3d> traj_cmd_, traj_real_;

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void bsplineCallback(plan_manage::BsplineConstPtr msg) {
  // parse pos traj

  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i) {
    knots(i) = msg->knots[i];
  }

  for (int i = 0; i < msg->pos_pts.size(); ++i) {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }

  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // parse yaw traj

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    yaw_pts(i, 0) = msg->yaw_pts[i];
  }

  NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());

  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;
}

void replanCallback(std_msgs::Empty msg) {
  /* reset duration */
  const double time_out = 0.01;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out;
  traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(std_msgs::Empty msg) {
  traj_cmd_.clear();
  traj_real_.clear();
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;

  traj_real_.push_back(
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

  if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void visCallback(const ros::TimerEvent& e) {
  // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
  // 1),
  //                      1);

  displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}
//lx add
void fillUnusedTrajectorySetpoints(mavros_msgs::PositionTarget &point) {
point.position.x = NAN;
point.position.y = NAN;
point.position.z = NAN;
point.velocity.x = NAN;
point.velocity.y = NAN;
point.velocity.z = NAN;
point.acceleration_or_force.x = NAN;
point.acceleration_or_force.y = NAN;
point.acceleration_or_force.z = NAN;
point.yaw = NAN;
point.yaw_rate = NAN;
}
//lx add

void mavrosCmdVelPub(quadrotor_msgs::PositionCommand cmd){
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = cmd.header.stamp;
    cmd_vel.header.frame_id = "local_origin";
    cmd_vel.twist.linear.x =  cmd.velocity.x;
    cmd_vel.twist.linear.y =  cmd.velocity.y;
    cmd_vel.twist.linear.z =  cmd.velocity.z;
    cmd_vel.twist.angular.x = NAN;
    cmd_vel.twist.angular.y = NAN;
    cmd_vel.twist.angular.z = cmd.yaw_dot;//航角度
    mavros_cmd_vel_pub.publish(cmd_vel);
}
int set_point_type=1;
void mavrosTrajectoryPub(quadrotor_msgs::PositionCommand cmd){
    mavros_msgs::Trajectory setpoint;
    setpoint.header.stamp = cmd.header.stamp;
    setpoint.header.frame_id = "local_origin";

    setpoint.type = set_point_type;  // MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS
    if(setpoint.type==0) {
        setpoint.point_1.position.x = cmd.position.x;
        setpoint.point_1.position.y = cmd.position.y;
        setpoint.point_1.position.z = cmd.position.z;
        setpoint.point_1.velocity.x = NAN;
        setpoint.point_1.velocity.y = NAN;
        setpoint.point_1.velocity.z = NAN;
        setpoint.point_1.acceleration_or_force.x = NAN;
        setpoint.point_1.acceleration_or_force.y = NAN;
        setpoint.point_1.acceleration_or_force.z = NAN;
        setpoint.point_1.yaw = cmd.yaw;
        setpoint.point_1.yaw_rate = NAN;
    }
    else if(setpoint.type==1){
        setpoint.point_1.position.x = NAN;
        setpoint.point_1.position.y = NAN;
        setpoint.point_1.position.z = NAN;
        setpoint.point_1.velocity.x = cmd.velocity.x + (cmd.position.x-odom.pose.pose.position.x)*0.5;
        setpoint.point_1.velocity.y = cmd.velocity.y + (cmd.position.y-odom.pose.pose.position.y)*0.5;
        setpoint.point_1.velocity.z = cmd.velocity.z + (cmd.position.z-odom.pose.pose.position.z)*0.5;
        ROS_INFO("D v_x:%f,v_y:%f,v_z:%f",cmd.velocity.x,cmd.velocity.y,cmd.velocity.z);
        ROS_INFO("D e_x:%f,e_y:%f,e_z:%f",(cmd.position.x-odom.pose.pose.position.x)*0.5,(cmd.position.y-odom.pose.pose.position.y)*0.5,(cmd.position.z-odom.pose.pose.position.z)*0.5);
        setpoint.point_1.acceleration_or_force.x = NAN;
        setpoint.point_1.acceleration_or_force.y = NAN;
        setpoint.point_1.acceleration_or_force.z = NAN;
        setpoint.point_1.yaw = cmd.yaw;
        setpoint.point_1.yaw_rate = NAN;
    }
    fillUnusedTrajectorySetpoints(setpoint.point_2);
    fillUnusedTrajectorySetpoints(setpoint.point_3);
    fillUnusedTrajectorySetpoints(setpoint.point_4);
    fillUnusedTrajectorySetpoints(setpoint.point_5);

    setpoint.time_horizon = {NAN, NAN, NAN, NAN, NAN};

    bool xy_pos_sp_valid = std::isfinite(setpoint.point_1.position.x) && std::isfinite(setpoint.point_1.position.y);
    bool xy_vel_sp_valid = std::isfinite(setpoint.point_1.velocity.x) && std::isfinite(setpoint.point_1.velocity.y);


    if ((xy_pos_sp_valid || xy_vel_sp_valid) &&
          (std::isfinite(setpoint.point_1.position.z || std::isfinite(setpoint.point_1.velocity.z)))) {
            setpoint.point_valid = {true, false, false, false, false};
            //ROS_INFO("mavrosTrajectoryPub setPoint is true");
        } else {
        setpoint.point_valid = {false, false, false, false, false};
    }
    trajectory_pub.publish(setpoint);
}
void mavrosSystemStatusPub() {
    mavros_msgs::CompanionProcessStatus status_msg;
    status_msg.state = static_cast<int>(MAV_STATE::MAV_STATE_ACTIVE);
    status_msg.header.stamp = ros::Time::now();
    status_msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE we need to add a new component
    mavros_system_status_pub.publish(status_msg);
}

void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
// roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

// yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}
double compute_diff_yaw(double tar_yaw,double cur_yaw){
    double diff_yaw = tar_yaw - cur_yaw;
    if (diff_yaw > M_PI)diff_yaw = -2 * M_PI + diff_yaw;
    else if (diff_yaw < -M_PI)diff_yaw = 2 * M_PI + diff_yaw;
    return diff_yaw;
}
double compute_odom_yaw(){
    Eigen::Quaterniond q(odom.pose.pose.orientation.w,
                         odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y,
                         odom.pose.pose.orientation.z);
    Eigen::Vector3d angle;
    toEulerAngle(q, angle[0], angle[1], angle[2]);
    return angle[2];
}
double compute_diff_yaw_with_odom(double tar_yaw){
    compute_diff_yaw(tar_yaw,compute_odom_yaw());
}
void cmdCallback(const ros::TimerEvent& e) {
  /* no publishing before receive traj_ */
  mavrosSystemStatusPub();

  if (!receive_traj_) return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();
  Eigen::Vector3d pos, vel, acc, pos_f;
  double yaw, yawdot;
  if (t_cur < traj_duration_ && t_cur >= 0.0) {

   //lx add control yaw

    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf);

  } else if (t_cur >= traj_duration_) {
    /* hover when finish traj_ */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];

    pos_f = pos;
  } else {
    cout << "[Traj server]: invalid time." << endl;
  }

  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw;
  cmd.yaw_dot = yawdot;

  auto pos_err = pos_f - pos;
  // if (pos_err.norm() > 1e-3) {
  //   cmd.yaw = atan2(pos_err(1), pos_err(0));
  // } else {
  //   cmd.yaw = last_yaw_;
  // }
  // cmd.yaw_dot = 1.0;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd);

  //lx add
//  static int count=0;
//  if(++count>=5) {
//      pub_flag=true;
        mavrosTrajectoryPub(cmd);
        //mavrosCmdVelPub(cmd);
//      count=0;
//  }
  // draw cmd

  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));

  Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));

  traj_cmd_.push_back(pos);
  if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}
//void trajectoryCallback (const mavros_msgs::Trajectory &msg){
//    ROS_INFO("recevie trajectory desired!");
//}
int main(int argc, char** argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);

  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

  mavros_system_status_pub=node.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 1);
  trajectory_pub = node.advertise<mavros_msgs::Trajectory>("/mavros/trajectory/generated", 10);
  mavros_cmd_vel_pub =  node.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",10);

  //ros::Subscriber trajectory_sub = node.subscribe("/mavros/trajectory/desired", 1, &trajectoryCallback);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  /* control parameter */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  last_yaw_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
