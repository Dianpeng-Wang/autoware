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

#ifndef SUB_BASE_TEMPLATE_H
#define SUB_BASE_TEMPLATE_H

#include <iostream>
#include <string>
#include <deque>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <boost/shared_ptr.hpp>

#include "pose_corrector/sub_base.h"

template <class T>
class SubBaseTemplate : public SubBase
{
  public:
    SubBaseTemplate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, std::string topic_name);
    virtual ~SubBaseTemplate() override;
    std::deque<geometry_msgs::TwistStamped> getQueue() const override { return queue_; };
    
  protected:
    virtual geometry_msgs::TwistStamped convertToTwistStamped(const boost::shared_ptr<const T>& input_msgs) const = 0;

  private:
    void subCallback(const boost::shared_ptr<const T>& input_msgs);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_;

    std::string topic_name_;
    std::deque<geometry_msgs::TwistStamped> queue_;

};

template <class T>
SubBaseTemplate<T>::SubBaseTemplate(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, std::string topic_name) :
     nh_(nh)
    ,private_nh_(private_nh)
    ,topic_name_(topic_name)
{
  sub_ = nh_.subscribe(topic_name_, 10, &SubBaseTemplate::subCallback, this);
}

template <class T>
SubBaseTemplate<T>::~SubBaseTemplate()
{
}

template <class T>
void SubBaseTemplate<T>::subCallback(const boost::shared_ptr<const T>& input_msgs)
{
  //This Code is not support when restarting with changing start time
  if(!queue_.empty() && queue_.front().header.stamp >= input_msgs->header.stamp)
  {
    std::cout << "Maybe ROSBAG is restarted. Cleared all buffer" << std::endl;
    queue_.clear();
  }

  auto tmp = convertToTwistStamped(input_msgs);
  queue_.push_back(tmp);

  while(queue_.back().header.stamp - queue_.front().header.stamp > ros::Duration(5.0))
  {
    //std::cout << "pop " << std::fmod(queue_.front().header.stamp.toSec(), 100.0) << std::endl;
    queue_.pop_front();
  }
}



#endif
