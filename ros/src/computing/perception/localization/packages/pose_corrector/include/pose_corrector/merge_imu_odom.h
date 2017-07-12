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

#ifndef MERGE_IMU_ODOM_H
#define MERGE_IMU_ODOM_H

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "pose_corrector/merge_base_two.h"
#include "pose_corrector/sub_imu.h"
#include "pose_corrector/sub_odom.h"

class MergeImuOdom : public MergeBaseTwo
{
  public:
    MergeImuOdom(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, const std::string imu_topic_name, const std::string odom_topic_name);
    ~MergeImuOdom() override;
  //private:
    geometry_msgs::TwistStamped mergeData(const ros::Time time, const geometry_msgs::TwistStamped& data1, const geometry_msgs::TwistStamped& data2) const override
    {
      geometry_msgs::TwistStamped tmp;
      tmp.header.stamp = time;
      tmp.twist.linear = data1.twist.linear;
      tmp.twist.angular = data2.twist.angular;
      return tmp;
    };
    
};

MergeImuOdom::MergeImuOdom(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, const std::string imu_topic_name, const std::string odom_topic_name) :
   MergeBaseTwo(boost::make_shared<const SubImu>(nh, private_nh, imu_topic_name), boost::make_shared<const SubOdom>(nh, private_nh, odom_topic_name))
{
}

MergeImuOdom::~MergeImuOdom()
{
}

#endif
