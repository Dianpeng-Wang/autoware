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
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "mqtt_socket/RemoteCmd.h"

class SelectCommand
{
  using remote_msgs_t = mqtt_socket::RemoteCmd;
  using auto_msgs_t = mqtt_socket::RemoteCmd;  //TODO fix message type

  public:
    SelectCommand(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    ~SelectCommand();
  private:
    void watchdog_timer();
    void remote_command_callback(const remote_msgs_t::ConstPtr& input);
    void auto_command_callback(const auto_msgs_t::ConstPtr& input);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_; 
    ros::Publisher emergency_stop_pub_;
    ros::Publisher select_command_pub_;
    ros::Subscriber remote_command_sub_;
    ros::Subscriber auto_command_sub_;
    ros::Time remote_command_time_;
    ros::Duration timeout_period_;
    std_msgs::Bool emergency_stop_;

    std::thread watchdog_timer_thread_;
    enum class CommandMode{AUTO=1, REMOTE} command_mode_;
};

SelectCommand::SelectCommand(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) :
     nh_(nh)
    ,private_nh_(private_nh)
    ,timeout_period_(1.0)
    ,command_mode_(CommandMode::REMOTE)
{
  emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop", 1, true);
  select_command_pub_ = nh_.advertise<remote_msgs_t>("/select_cmd", 1, true);
  remote_command_sub_ = nh_.subscribe("/remote_cmd", 1, &SelectCommand::remote_command_callback, this);
  auto_command_sub_ = nh_.subscribe("/auto_cmd", 1, &SelectCommand::auto_command_callback, this);

  emergency_stop_.data = false;
  
  remote_command_time_ = ros::Time::now();
  watchdog_timer_thread_ = std::thread(&SelectCommand::watchdog_timer, this);
  watchdog_timer_thread_.detach();
}

SelectCommand::~SelectCommand()
{
}

void SelectCommand::watchdog_timer()
{
  while(1)
  {
    ros::Time now_time = ros::Time::now();

    if(now_time - remote_command_time_ >  timeout_period_ 
       || emergency_stop_.data == true)
    {
        command_mode_ = CommandMode::AUTO;
        emergency_stop_.data = true;
        emergency_stop_pub_.publish(emergency_stop_);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::cout << "c_mode:"     << static_cast<int>(command_mode_) 
              << " e_stop:"    << static_cast<bool>(emergency_stop_.data)
              << " diff_time:" << (now_time - remote_command_time_).toSec() 
              << std::endl;
  }
}

void SelectCommand::remote_command_callback(const remote_msgs_t::ConstPtr& input_msgs)
{
  command_mode_ = static_cast<CommandMode>(input_msgs->mode);
  emergency_stop_.data = static_cast<bool>(input_msgs->emergency);
  remote_command_time_ = ros::Time::now();
  
  if(command_mode_ == CommandMode::REMOTE)
    select_command_pub_.publish(*input_msgs);
}

void SelectCommand::auto_command_callback(const auto_msgs_t::ConstPtr& input_msgs)
{
  if(command_mode_ == CommandMode::AUTO)
    select_command_pub_.publish(*input_msgs);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "select_command");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  SelectCommand select_command(nh, private_nh);

  ros::spin();
  return 0;
}


