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

#ifndef MERGE_BASE_TWO_H
#define MERGE_BASE_TWO_H

#include <vector>
#include <deque>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "pose_corrector/merge_base.h"
#include "pose_corrector/sub_base.h"


class MergeBaseTwo : public MergeBase
{
  public:
    MergeBaseTwo(const boost::shared_ptr<const SubBase>& sub1, const boost::shared_ptr<const SubBase>& sub2);
    virtual ~MergeBaseTwo();
    virtual geometry_msgs::TwistStamped mergeData(const ros::Time time, const geometry_msgs::TwistStamped& data1, const geometry_msgs::TwistStamped& data2) const = 0;
    std::vector<geometry_msgs::TwistStamped> mergeQueue() const override;

  private:
    boost::shared_ptr<const SubBase> sub1_ptr_;
    boost::shared_ptr<const SubBase> sub2_ptr_;
};

MergeBaseTwo::MergeBaseTwo(const boost::shared_ptr<const SubBase>& sub1, const boost::shared_ptr<const SubBase>& sub2) :
    sub1_ptr_(sub1)
   ,sub2_ptr_(sub2)
{
}

MergeBaseTwo::~MergeBaseTwo()
{
}

std::vector<geometry_msgs::TwistStamped> MergeBaseTwo::mergeQueue() const
{
  std::vector<geometry_msgs::TwistStamped> merged_array;
  auto queue1 = sub1_ptr_->getQueue();
  auto queue2 = sub2_ptr_->getQueue();
  
  auto it1 = std::begin(queue1);
  auto it2 = std::begin(queue2);

  ros::Time time = it1->header.stamp < it2->header.stamp ? it1->header.stamp : it2->header.stamp;
  while(1)
  {
    auto tmp = mergeData(time, *it1, *it2);

    //std::cout << std::fmod(time.toSec(), 100.0) << std::endl;
    merged_array.push_back(tmp);

    if(it1+1 == std::end(queue1) && it2+1 == std::end(queue2))
      break;
    else if( it2+1 == std::end(queue2) 
          || ( it1+1 != std::end(queue1) 
            && (it1+1)->header.stamp < (it2+1)->header.stamp))
    {
      ++it1;
      time = it1->header.stamp;
    }
    else
    {
      ++it2;
      time = it2->header.stamp;
    }
  }
  return merged_array;
}

#endif
