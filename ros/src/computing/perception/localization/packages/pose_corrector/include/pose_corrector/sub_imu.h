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

#ifndef SUB_IMU_H
#define SUB_IMU_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <boost/shared_ptr.hpp>

#include "pose_corrector/sub_base_template.h"

class SubImu : public SubBaseTemplate<sensor_msgs::Imu>
{
  public:
    SubImu(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, std::string topic_name);
    ~SubImu() override;

  private:
    geometry_msgs::TwistStamped convertToTwistStamped(const boost::shared_ptr<const sensor_msgs::Imu>& input_msgs) const override
    {
      geometry_msgs::TwistStamped tmp;
      tmp.header        = input_msgs->header;
      tmp.twist.linear  = input_msgs->linear_acceleration;
      tmp.twist.angular = input_msgs->angular_velocity;
      return tmp;
    };

};

//TODO: Move to cpp

SubImu::SubImu(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, std::string topic_name) : 
   SubBaseTemplate(nh, private_nh, topic_name)
{
}

SubImu::~SubImu()
{
}



#endif
