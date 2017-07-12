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


#ifndef POSE_CORRECTOR_BASE_H
#define POSE_CORRECTOR_BASE_H

#include <cmath>
#include <vector>
#include <deque>
#include <chrono>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include "pose_corrector/merge_base.h"
#include "pose_corrector_srv/pose_corrector.h"

class PoseCorrectorBase
{
  public:
    PoseCorrectorBase(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, const boost::shared_ptr<const MergeBase>& merge_base_ptr);
    virtual ~PoseCorrectorBase();
    geometry_msgs::PoseStamped calc(const geometry_msgs::PoseStamped& begin_pose, const ros::Time& begin_time, const ros::Time& end_time);

    bool srvCallback(pose_corrector_srv::pose_corrector::Request& req, pose_corrector_srv::pose_corrector::Response& res);

//  private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
//    ros::Subscriber lidar_sub_;
//    ros::Publisher pub_;
    ros::ServiceServer srv_;

    boost::shared_ptr<const MergeBase> merge_base_ptr_;
    
};

//TODO: Move to cpp

PoseCorrectorBase::PoseCorrectorBase(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, const boost::shared_ptr<const MergeBase>& merge_base_ptr) :
     nh_(nh)
    ,private_nh_(private_nh)
    ,merge_base_ptr_(merge_base_ptr)
{
//  lidar_sub_ = nh_.subscribe("/points_raw", 10, &PoseCorrectorBase::lidarsubCallback, this);
//  pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose_correct", 1, this);
  srv_= nh_.advertiseService("/pose_corrector", &PoseCorrectorBase::srvCallback, this);
}

PoseCorrectorBase::~PoseCorrectorBase()
{
}

bool PoseCorrectorBase::srvCallback(pose_corrector_srv::pose_corrector::Request& req, pose_corrector_srv::pose_corrector::Response& res)
{
    std::chrono::time_point<std::chrono::system_clock> srv_start = std::chrono::system_clock::now();
  res.pose = calc(req.pose, req.previous_time.data, req.current_time.data);
    std::chrono::time_point<std::chrono::system_clock> srv_end = std::chrono::system_clock::now();
    double srv_time = std::chrono::duration_cast<std::chrono::microseconds>(srv_end - srv_start).count() / 1000.0;
    std::cout << "time: " << srv_time << std::endl;
  return true;
}

geometry_msgs::PoseStamped PoseCorrectorBase::calc(const geometry_msgs::PoseStamped& begin_pose, const ros::Time& begin_time, const ros::Time& end_time)
{  
  geometry_msgs::PoseStamped end_pose;
  double x = 0,y = 0,z = 0;
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(begin_pose.pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  auto merged_array = merge_base_ptr_->mergeQueue();

  for(auto it = std::begin(merged_array); it != std::end(merged_array); ++it)
  {
    if(it != std::begin(merged_array) && it->header.stamp > end_time)
      break;

    const auto it2 = (it != std::begin(merged_array) && it+1 != std::end(merged_array)) ? it+1 : it;
    if(it+1 != std::end(merged_array) && it2->header.stamp < begin_time)
      continue;

    const ros::Time previous_time = (it != std::begin(merged_array) && it->header.stamp > begin_time) ? it->header.stamp : begin_time;
    const ros::Time current_time  = (it+1 != std::end(merged_array) && it2->header.stamp < end_time) ? it2->header.stamp : end_time;

    std::cout << std::fmod(it->header.stamp.toSec(), 100.0) 
       << " " << std::fmod(previous_time.toSec(), 100.0)
       << " " << std::fmod(it2->header.stamp.toSec(), 100.0) 
       << " " << std::fmod(current_time.toSec(), 100.0)
       << std::endl;

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

#endif
