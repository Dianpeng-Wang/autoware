/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <deque>
#include <cmath>
#include <cassert>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>	
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

#include "pose_corrector_srv/pose_corrector.h"

class PoseCorrector
{
  public:
    PoseCorrector(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~PoseCorrector();
  private:
    void sub1Callback(const nav_msgs::Odometry::ConstPtr& input_msgs);
    void sub2Callback(const sensor_msgs::Imu::ConstPtr& input_msgs);
    bool poseCorrectorCallback(pose_corrector_srv::pose_corrector::Request& req, pose_corrector_srv::pose_corrector::Response& res);
    geometry_msgs::PoseStamped calc(geometry_msgs::PoseStamped begin_pose, ros::Time begin_time, ros::Time end_time);
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_;
    ros::Subscriber sub2_;
    ros::ServiceServer srv_;

    std::deque<geometry_msgs::TwistStamped> queue1;
    std::deque<geometry_msgs::TwistStamped> queue2;
};

PoseCorrector::PoseCorrector(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) :
     nh_(nh)
    ,private_nh_(private_nh)
{
  sub_ = nh_.subscribe("/odom_pose", 1, &PoseCorrector::sub1Callback, this);
  sub2_ = nh_.subscribe("/imu_raw", 1, &PoseCorrector::sub2Callback, this);
  srv_= nh_.advertiseService("/pose_corrector", &PoseCorrector::poseCorrectorCallback, this);
}

PoseCorrector::~PoseCorrector()
{
}

void PoseCorrector::sub1Callback(const nav_msgs::Odometry::ConstPtr& input_msgs)
{
  //This Code is not support when restarting with changing start time
  if(!queue1.empty() && queue1.front().header.stamp >= input_msgs->header.stamp)
  {
    std::cout << "Maybe ROSBAG is restarted. Cleared all buffer" << std::endl;
    queue1.clear();
  }

  geometry_msgs::TwistStamped tmp;
  tmp.header = input_msgs->header;
  tmp.twist = input_msgs->twist.twist;
  queue1.push_back(tmp);

  while(queue1.back().header.stamp - queue1.front().header.stamp > ros::Duration(10.0))
  {
    std::cout << "pop " << std::fmod(queue1.front().header.stamp.toSec(), 100.0) << std::endl;
    queue1.pop_front();
  }
}

void PoseCorrector::sub2Callback(const sensor_msgs::Imu::ConstPtr& input_msgs)
{
  //This Code is not support when restarting with changing start time
  if(!queue2.empty() && queue2.front().header.stamp >= input_msgs->header.stamp)
  {
    std::cout << "Maybe ROSBAG is restarted. Cleared all buffer" << std::endl;
    queue2.clear();
  }

  geometry_msgs::TwistStamped tmp;
  tmp.header        = input_msgs->header;
  tmp.twist.linear  = input_msgs->linear_acceleration;
  tmp.twist.angular = input_msgs->angular_velocity;
  queue2.push_back(tmp);

  while(queue2.back().header.stamp - queue2.front().header.stamp > ros::Duration(10.0))
  {
    std::cout << "pop " << std::fmod(queue2.front().header.stamp.toSec(), 100.0) << std::endl;
    queue2.pop_front();
  }
}

geometry_msgs::PoseStamped PoseCorrector::calc(geometry_msgs::PoseStamped begin_pose, ros::Time begin_time, ros::Time end_time)
{  
  geometry_msgs::PoseStamped end_pose;
  double x = 0,y = 0,z = 0;
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(begin_pose.pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  std::vector<geometry_msgs::TwistStamped> merged_array;
  auto q1_it = std::begin(queue1);
  auto q2_it = std::begin(queue2);

  while(q1_it != std::end(queue1) || q2_it != std::end(queue2))
  {
    geometry_msgs::TwistStamped tmp;
    static ros::Time time = q1_it->header.stamp < q2_it->header.stamp ? q1_it->header.stamp : q1_it->header.stamp;
    
    tmp.header.stamp = time;
    tmp.twist.linear = q1_it->twist.linear;
    tmp.twist.angular = q2_it->twist.angular;

    merged_array.push_back(tmp);

    if((q1_it+1)->header.stamp < (q1_it+1)->header.stamp)
    {
      ++q1_it;
      time = q1_it->header.stamp;
    }
    else
    {
      ++q2_it;
      time = q2_it->header.stamp;
    }
  }

  for(auto it = std::begin(merged_array); it != std::end(merged_array); ++it)
  {
    if(it != std::begin(merged_array) && it->header.stamp > end_time)
      break;

    const auto it2 = (it != std::begin(merged_array) && it+1 != std::end(merged_array)) ? it+1 : it;
    if(it+1 != std::end(merged_array) && it2->header.stamp < begin_time)
      continue;

    const ros::Time previous_time = (it != std::begin(merged_array) && it->header.stamp > begin_time) ? it->header.stamp : begin_time; 
    const ros::Time current_time  = (it+1 != std::end(merged_array) && it2->header.stamp < end_time)  ? it2->header.stamp : end_time;

//    std::cout << std::fmod(previous_time.toSec(), 100.0)  << " " << std::fmod(current_time.toSec(), 100.0) << std::endl;

    const double diff_time = (current_time - previous_time).toSec();
    assert(diff_time >= 0);

    roll  += it2->twist.angular.x * diff_time;
    pitch += it2->twist.angular.y * diff_time;
    yaw   += it2->twist.angular.z * diff_time;

    const double dis = (it2->twist.linear.x + it2->twist.linear.y + it2->twist.linear.z) * diff_time;
    x += dis*cos(-pitch)*cos(yaw);
    y += dis*cos(-pitch)*sin(yaw);
    z += dis*sin(-pitch);

//    x += dis          *cos(pitch)*cos(yaw);
//    y += dis*sin(roll)           *sin(yaw);
//    z += dis*cos(roll)*sin(pitch);
  }

  end_pose.header = begin_pose.header;
  end_pose.header.stamp = end_time;
  end_pose.pose.position.x = begin_pose.pose.position.x + x;
  end_pose.pose.position.y = begin_pose.pose.position.y + y;
  end_pose.pose.position.z = begin_pose.pose.position.z + z;
  end_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

  return end_pose;
}

bool PoseCorrector::poseCorrectorCallback(pose_corrector_srv::pose_corrector::Request& req, pose_corrector_srv::pose_corrector::Response& res)
{
  assert(req.previous_time.data <= req.current_time.data);

  res.pose = calc(req.pose, req.previous_time.data, req.current_time.data);
  std::cout << res.pose.pose.position.x << std::endl;
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_corrector");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  PoseCorrector pose_corrector(nh, private_nh);

  ros::spin();
  return 0;
}


