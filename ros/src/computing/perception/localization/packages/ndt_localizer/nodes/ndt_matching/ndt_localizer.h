/*
 *  Copyright (c) 2015, Nagoya University
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
/*
 Localization program using Normal Distributions Transform

 Yuki KITSUKAWA
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef USE_FAST_PCL
#include <fast_pcl/registration/ndt.h>
#else
#include <pcl/registration/ndt.h>
#endif

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <runtime_manager/ConfigNdt.h>

#include <ndt_localizer/ndt_stat.h>
#include "pose_corrector_msgs/Request.h"
#include "pose_corrector_msgs/Response.h"
#include "pose_corrector_msgs/Service.h"

//TODO: remove define
#define PREDICT_POSE_THRESHOLD 0.5

#define Wa 0.4
#define Wb 0.3
#define Wc 0.3

struct Pose
{
  Pose() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0) {};
  Pose(double x, double y, double z, double roll, double pitch, double yaw)
    : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {};
  void clear(){x = y = z = roll = pitch = yaw = 0;};
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;

  Pose operator-(const Pose& rhs_pose) const 
  {
    Pose tmp_pose;
    tmp_pose.x = x - rhs_pose.x;
    tmp_pose.y = y - rhs_pose.y;
    tmp_pose.z = z - rhs_pose.z;
    tmp_pose.roll = roll - rhs_pose.roll;
    tmp_pose.pitch = pitch - rhs_pose.pitch;
    tmp_pose.yaw = yaw - rhs_pose.yaw;
    return tmp_pose;
  };
  
  bool operator!=(const Pose& rhs_pose) const
  {
    return !(x == rhs_pose.x && y == rhs_pose.y && z == rhs_pose.z
       && roll == rhs_pose.roll && pitch == rhs_pose.pitch && yaw == rhs_pose.yaw);
  }

};

struct Linear
{
  double x;
  double y;
  double z;
  double xyz;
};

struct Angular
{
  double x;
  double y;
  double z;
};

struct Velocity
{
  void clear(){linear.x = linear.y = linear.z = linear.xyz = angular.x = angular.y = angular.z;};
  
  void setVelocity(const Pose& current_pose, const Pose& previous_pose, double time_diff_sec)
  {
    Pose diff_pose = current_pose - previous_pose;
    const double diff_pose_distance = sqrt(diff_pose.x * diff_pose.x + diff_pose.y * diff_pose.y + diff_pose.z * diff_pose.z);

    linear.x = diff_pose.x / time_diff_sec;
    linear.y = diff_pose.y / time_diff_sec;
    linear.z = diff_pose.z / time_diff_sec;
    linear.xyz = diff_pose_distance / time_diff_sec;
    angular.x = diff_pose.roll / time_diff_sec;
    angular.y = diff_pose.pitch / time_diff_sec;
    angular.z = diff_pose.yaw / time_diff_sec;
  };

  Velocity operator-(const Velocity& rhs_v) const
  {
    Velocity tmp_v;
    tmp_v.linear.x = linear.x - rhs_v.linear.x;
    tmp_v.linear.y = linear.y - rhs_v.linear.y;
    tmp_v.linear.z = linear.z - rhs_v.linear.z;
    tmp_v.linear.xyz = tmp_v.linear.xyz - rhs_v.linear.xyz;
    tmp_v.angular.x = tmp_v.angular.x - rhs_v.angular.x;
    tmp_v.angular.y = tmp_v.angular.y - rhs_v.angular.y;
    tmp_v.angular.z = tmp_v.angular.z - rhs_v.angular.z;
    return tmp_v;
  }

  Linear linear;
  Angular angular;
};

struct Accel
{
  void clear(){linear.x = linear.y = linear.z = linear.xyz = angular.x = angular.y = angular.z;};
  void setAccel(const Velocity& current_v, const Velocity& previous_v, double time_diff_sec)
  {
    Velocity diff_v = current_v - previous_v;

    linear.x = diff_v.linear.x / time_diff_sec;
    linear.y = diff_v.linear.y / time_diff_sec;
    linear.z = diff_v.linear.z / time_diff_sec;
    linear.xyz = diff_v.linear.xyz / time_diff_sec;
    angular.x = diff_v.angular.x / time_diff_sec;
    angular.y = diff_v.angular.y / time_diff_sec;
    angular.z = diff_v.angular.z / time_diff_sec;
  };
  
  Linear linear;
  Angular angular;

};

template <class PointT>
class NDTLocalizer
{
  public:
    NDTLocalizer(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

  private:
    void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& input);  //TODO: map_publiser  nodelet
    void paramCallback(const runtime_manager::ConfigNdt::ConstPtr& input);
    void gnssCallback(const geometry_msgs::PoseStamped::ConstPtr& input);
    void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input);
    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    void poseCorrectorCallback(const pose_corrector_msgs::Response::ConstPtr& input);
    
    void matching();
    void calcPoses();
    void calcStats();
    void calcVelocities();
    void outputLogs();
    
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    ros::Publisher predict_pose_pub_;
    ros::Publisher ndt_pose_pub_;
    //ros::Publisher current_pose_pub_;
    ros::Publisher localizer_pose_pub_;
    ros::Publisher estimate_twist_pub_;
    ros::Publisher pose_corrector_pub_;
    ros::Publisher estimated_vel_mps_pub_;
    ros::Publisher estimated_vel_kmph_pub_;
    ros::Publisher estimated_vel_pub_;
    ros::Publisher time_ndt_matching_pub_;
    ros::Publisher ndt_reliability_pub_;
    ros::Publisher ndt_stat_pub_;

    ros::Subscriber param_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber initialpose_sub_;
    ros::Subscriber points_sub_;
    ros::Subscriber pose_corrector_sub_;

    ros::ServiceClient pose_corrector_client_;

    ros::Time current_scan_time_;  //TODO: sholud remove
    std_msgs::Float32 time_ndt_matching_; //TODO: sholud remove
    std_msgs::Float32 ndt_reliability_;  //TODO: sholud remove
    std_msgs::Header current_scan_header_;

    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    tf::StampedTransform local_transform_;

    bool map_loaded_;
    bool init_pos_set_;
    bool use_local_transform_;
    bool use_gnss_;
    bool use_map_height_;
    bool use_openmp_;
    std::string localizer_;
    int queue_size_;
    int max_iter_;
    double ndt_res_;
    double step_size_;
    double trans_eps_; 
    double fitness_score_; //TODO: sholud remove
    double trans_probability_; //TODO: sholud remove
    double exe_time_; //TODO: sholud remove
    double align_time_; //TODO: sholud remove
    double getFitnessScore_time_; //TODO: sholud remove
    double predict_pose_error_; //TODO: sholud remove
    double current_velocity_smooth_;  //TODO: sholud remove
    std::ofstream ofs_;

    pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
    pcl::PointCloud<PointT> map_;
    pcl::PointCloud<PointT> scan_;

    Eigen::Matrix4f tf_btol_; //TODO: sholud remove
    Eigen::Matrix4f tf_ltob_; //TODO: sholud remove
    
    Pose current_pose_;
    Pose previous_pose_;
    Pose localizer_pose_;
    Pose ndt_pose_;
    Pose predict_pose_;

    Velocity current_velocity_;
    Velocity previous_velocity_;
    Velocity previous_previous_velocity_;
    Accel current_accel_;
};

template <class PointT>
NDTLocalizer<PointT>::NDTLocalizer(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) :
    nh_(nh)
   ,private_nh_(private_nh)
   ,map_loaded_(false)
   ,init_pos_set_(false)
   ,queue_size_(1000)
   ,max_iter_(30)
   ,ndt_res_(1.0)
   ,step_size_(0.1)
   ,trans_eps_(0.01)
   ,localizer_("velodyne")
{
  
  char buffer[80];
  std::time_t now = std::time(NULL);
  std::tm* pnow = std::localtime(&now);
  std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
  std::string filename = "ndt_matching_" + std::string(buffer) + ".csv";
  ofs_.open(filename.c_str(), std::ios::app);
  
  // Geting parameters
  private_nh_.getParam("use_gnss", use_gnss_);
  private_nh_.getParam("queue_size", queue_size_);
  private_nh_.getParam("use_openmp", use_openmp_);
  private_nh_.getParam("get_height", use_map_height_);  //TODO rename
  private_nh_.getParam("use_local_transform_", use_local_transform_);

  if (nh_.getParam("localizer", localizer_) == false)
  {
    std::cout << "localizer is not set." << std::endl;
    exit(1);
  }

  static double tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw;

  //TODO: should rename parameters
  //e.g. tf_x -> /base_link_to_localizer/tf_x
  if (nh_.getParam("tf_x", tf_x) == false)
  {
    std::cout << "tf_x is not set." << std::endl;
    exit(1);
  }
  if (nh_.getParam("tf_y", tf_y) == false)
  {
    std::cout << "tf_y is not set." << std::endl;
    exit(1);
  }
  if (nh_.getParam("tf_z", tf_z) == false)
  {
    std::cout << "tf_z is not set." << std::endl;
    exit(1);
  }
  if (nh_.getParam("tf_roll", tf_roll) == false)
  {
    std::cout << "tf_roll is not set." << std::endl;
    exit(1);
  }
  if (nh_.getParam("tf_pitch", tf_pitch) == false)
  {
    std::cout << "tf_pitch is not set." << std::endl;
    exit(1);        
  }
  if (nh_.getParam("tf_yaw", tf_yaw) == false)
  {
    std::cout << "tf_yaw is not set." << std::endl;
    exit(1);
  }

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Log file: " << filename << std::endl;
  std::cout << "use_gnss: " << use_gnss_ << std::endl;
  std::cout << "queue_size: " << queue_size_ << std::endl;
  std::cout << "use_openmp: " << use_openmp_ << std::endl;
  std::cout << "get_height: " << use_map_height_ << std::endl;  //TODO rename
  std::cout << "use_local_transform_: " << use_local_transform_ << std::endl;
  std::cout << "localizer: " << localizer_ << std::endl;
  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << tf_x << ", " << tf_y << ", " << tf_z << ", " << tf_roll << ", " << tf_pitch << ", " << tf_yaw << ")" << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  Eigen::Translation3f tl_btol(tf_x, tf_y, tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  Eigen::Translation3f tl_ltob((-1.0) * tf_x, (-1.0) * tf_y, (-1.0) * tf_z);  // tl: translation
  Eigen::AngleAxisf rot_x_ltob((-1.0) * tf_roll, Eigen::Vector3f::UnitX());     // rot: rotation
  Eigen::AngleAxisf rot_y_ltob((-1.0) * tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_ltob((-1.0) * tf_yaw, Eigen::Vector3f::UnitZ());
  tf_ltob_ = (tl_ltob * rot_z_ltob * rot_y_ltob * rot_x_ltob).matrix();

  // Publishers
  predict_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/predict_pose", 1000);
  ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/ndt_pose", 1000);
  // current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);
  localizer_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/localizer_pose", 1000);
  estimate_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/estimate_twist", 1000);
  estimated_vel_mps_pub_ = nh_.advertise<std_msgs::Float32>("/estimated_vel_mps", 1000);
  estimated_vel_kmph_pub_ = nh_.advertise<std_msgs::Float32>("/estimated_vel_kmph", 1000);
  estimated_vel_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/estimated_vel", 1000);
  time_ndt_matching_pub_ = nh_.advertise<std_msgs::Float32>("/time_ndt_matching_", 1000);
  ndt_stat_pub_ = nh_.advertise<ndt_localizer::ndt_stat>("/ndt_stat", 1000);
  ndt_reliability_pub_ = nh_.advertise<std_msgs::Float32>("/ndt_reliability", 1000);
  pose_corrector_pub_ = nh_.advertise<pose_corrector_msgs::Request>("/pose_corrector_request", 1000);

  // Subscribers
  param_sub_ = nh_.subscribe("/config/ndt", 10, &NDTLocalizer::paramCallback, this);
  gnss_sub_ = nh_.subscribe("/gnss_pose", 10, &NDTLocalizer::gnssCallback, this);
  map_sub_ = nh_.subscribe("/points_map", 10, &NDTLocalizer::mapCallback, this);
  initialpose_sub_ = nh_.subscribe("/initialpose", 1000, &NDTLocalizer::initialposeCallback, this);
  points_sub_ = nh_.subscribe("/filtered_points", queue_size_, &NDTLocalizer::pointsCallback, this);
  pose_corrector_sub_ = nh_.subscribe("/pose_corrector_response", 1, &NDTLocalizer::poseCorrectorCallback, this);

  // Services
  pose_corrector_client_ = nh_.serviceClient<pose_corrector_msgs::Service>("/pose_corrector_service");
}

template <class PointT>
void NDTLocalizer<PointT>::paramCallback(const runtime_manager::ConfigNdt::ConstPtr& input)
{
  std::cout << __func__ << std::endl;
  Pose input_pose(input->x, input->y, input->z, input->roll, input->pitch, input->yaw);
  static Pose initial_pose = input_pose;
  if (use_gnss_ != input->init_pos_gnss)
  {
    init_pos_set_ = false;
  }
  else if (use_gnss_ == false && initial_pose != input_pose)
  {
    init_pos_set_ = false;
  }

  use_gnss_ = input->init_pos_gnss;

  // Setting parameters
  if (input->resolution != ndt_res_)
  {
    ndt_res_ = input->resolution;
    ndt_.setResolution(ndt_res_);
  }
  if (input->step_size != step_size_)
  {
    step_size_ = input->step_size;
    ndt_.setStepSize(step_size_);
  }
  if (input->trans_epsilon != trans_eps_)
  {
    trans_eps_ = input->trans_epsilon;
    ndt_.setTransformationEpsilon(trans_eps_);
  }
  if (input->max_iterations != max_iter_)
  {
    max_iter_ = input->max_iterations;
    ndt_.setMaximumIterations(max_iter_);
  }

  if (use_gnss_ == false && init_pos_set_ == false)
  {

    if (use_local_transform_ == true)
    {
      tf::Vector3 v(input->x, input->y, input->z);
      tf::Quaternion q;
      q.setRPY(input->roll, input->pitch, input->yaw);
      tf::Transform transform(q, v);
      initial_pose.x = (local_transform_.inverse() * transform).getOrigin().getX();
      initial_pose.y = (local_transform_.inverse() * transform).getOrigin().getY();
      initial_pose.z = (local_transform_.inverse() * transform).getOrigin().getZ();

      tf::Matrix3x3 m(q);
      m.getRPY(initial_pose.roll, initial_pose.pitch, initial_pose.yaw);
    }
    else
       initial_pose = input_pose;

    // Setting position and posture for the first time.
    localizer_pose_ = initial_pose;
    previous_pose_ = initial_pose;
    current_pose_ = initial_pose;

    current_velocity_.clear();

    init_pos_set_ = true;

    std::cout << "initial_pose.x: " << initial_pose.x << std::endl;
    std::cout << "initial_pose.y: " << initial_pose.y << std::endl;
    std::cout << "initial_pose.z: " << initial_pose.z << std::endl;
    std::cout << "initial_pose.roll: " << initial_pose.roll << std::endl;
    std::cout << "initial_pose.pitch: " << initial_pose.pitch << std::endl;
    std::cout << "initial_pose.yaw: " << initial_pose.yaw << std::endl;

  }
}

template <class PointT>
void NDTLocalizer<PointT>::mapCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  std::cout << __func__ << std::endl;
  if (map_loaded_ == false)
  {
    // Convert the data type(from sensor_msgs to pcl).
    pcl::fromROSMsg(*input, map_);

    if (use_local_transform_ == true)
    {
      try
      {
        ros::Time now = ros::Time(0);
        tf_listener_.waitForTransform("/map", "/world", now, ros::Duration(10.0));
        tf_listener_.lookupTransform("/map", "world", now, local_transform_);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
      }

      pcl_ros::transformPointCloud(map_, map_, local_transform_.inverse());
    }

    boost::shared_ptr< pcl::PointCloud<PointT> > map_ptr(new pcl::PointCloud<PointT>(map_));
    // Setting point cloud to be aligned to.
    ndt_.setInputTarget(map_ptr);

    // Setting NDT parameters to default values
    ndt_.setMaximumIterations(max_iter_);
    ndt_.setResolution(ndt_res_);
    ndt_.setStepSize(step_size_);
    ndt_.setTransformationEpsilon(trans_eps_);
    map_loaded_ = true;
  }
}

template <class PointT>
void NDTLocalizer<PointT>::gnssCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
  std::cout << __func__ << std::endl;

  tf::Quaternion gnss_q(input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z, input->pose.orientation.w);
  tf::Matrix3x3 gnss_m(gnss_q);

  Pose current_gnss_pose;
  current_gnss_pose.x = input->pose.position.x;
  current_gnss_pose.y = input->pose.position.y;
  current_gnss_pose.z = input->pose.position.z;
  gnss_m.getRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);

  static Pose previous_gnss_pose = current_gnss_pose;
  ros::Time current_gnss_time = input->header.stamp;
  static ros::Time previous_gnss_time = current_gnss_time;

  Velocity current_gnss_velocity;
  static Velocity previous_gnss_velocity;
  static Velocity previous_previous_gnss_velocity;
  double time_diff_sec = (current_gnss_time - current_gnss_time).toSec();
  current_gnss_velocity.setVelocity(current_gnss_pose, previous_gnss_pose, time_diff_sec);

  if ((use_gnss_ == true && init_pos_set_ == false) || fitness_score_ >= 500.0)
  {
    previous_pose_ = previous_gnss_pose;
    current_pose_ = current_gnss_pose;

    current_velocity_ = current_gnss_velocity;
    previous_velocity_ = previous_gnss_velocity;
    previous_previous_velocity_ = previous_previous_gnss_velocity;

    current_accel_.setAccel(current_velocity_, previous_velocity_, time_diff_sec);

    init_pos_set_ = true;
  }

  previous_gnss_pose = current_gnss_pose;
  previous_gnss_time = current_gnss_time;

  previous_gnss_velocity = current_gnss_velocity;
  previous_previous_gnss_velocity = previous_gnss_velocity;

}

template <class PointT>
void NDTLocalizer<PointT>::initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input)
{
  Pose initial_pose;
  tf::StampedTransform transform;

  if (use_local_transform_ == true)
  {
    initial_pose.x = input->pose.pose.position.x;
    initial_pose.y = input->pose.pose.position.y;
    initial_pose.z = input->pose.pose.position.z;
  }
  else
  {
    try
    {
      ros::Time now = ros::Time(0);
      tf_listener_.waitForTransform("/map", "/world", now, ros::Duration(10.0));
      tf_listener_.lookupTransform("/map", "/world", now, transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    initial_pose.x = input->pose.pose.position.x + transform.getOrigin().x();
    initial_pose.y = input->pose.pose.position.y + transform.getOrigin().y();
    initial_pose.z = input->pose.pose.position.z + transform.getOrigin().z();
  }

  tf::Quaternion q(input->pose.pose.orientation.x, input->pose.pose.orientation.y, input->pose.pose.orientation.z, input->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(initial_pose.roll, initial_pose.pitch, initial_pose.yaw);

  if (use_map_height_ == true && map_loaded_ == true)
  {
    //TODO: refactoring
    double min_distance = DBL_MAX;
    double nearest_z = current_pose_.z;
    for (const auto& p : map_)
    {
      double distance = hypot(current_pose_.x - p.x, current_pose_.y - p.y);
      if (distance < min_distance)
      {
        min_distance = distance;
        nearest_z = p.z;
      }
    }
    current_pose_.z = nearest_z;
  }
  current_pose_ = initial_pose;
  previous_pose_ = current_pose_;

  current_velocity_.clear();
  previous_velocity_.clear();
  previous_previous_velocity_.clear();

  current_accel_.clear();

}

Pose convertROSMsgtoPose(const pose_corrector_msgs::Response::ConstPtr& msg)
{
  Pose tmp_pose;
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  tmp_pose.x = msg->pose.pose.position.x;
  tmp_pose.y = msg->pose.pose.position.y;
  tmp_pose.z = msg->pose.pose.position.z;
  tmp_pose.roll  = roll;
  tmp_pose.pitch = pitch;
  tmp_pose.yaw   = yaw;

  return tmp_pose;
}

geometry_msgs::PoseStamped convertPosetoROSMsg(const std_msgs::Header& header, const Pose& pose)
{
  tf::Quaternion q;
  q.setRPY(pose.roll, pose.pitch, pose.yaw);

  geometry_msgs::PoseStamped msg;
  msg.header = header;
  msg.pose.position.x = pose.x;
  msg.pose.position.y = pose.y;
  msg.pose.position.z = pose.z;
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  return msg;
}

geometry_msgs::PoseStamped convertPosetoROSMsg(const std_msgs::Header& header, const Pose& pose, const tf::Transform& local_transform_)
{
  tf::Quaternion q;
  q.setRPY(pose.roll, pose.pitch, pose.yaw);

  tf::Vector3 v(pose.x, pose.y, pose.z);
  tf::Transform transform(q, v);

  geometry_msgs::PoseStamped msg;
  msg.header = header;
  msg.pose.position.x = (local_transform_ * transform).getOrigin().getX();
  msg.pose.position.y = (local_transform_ * transform).getOrigin().getY();
  msg.pose.position.z = (local_transform_ * transform).getOrigin().getZ();
  msg.pose.orientation.x = (local_transform_ * transform).getRotation().x();
  msg.pose.orientation.y = (local_transform_ * transform).getRotation().y();
  msg.pose.orientation.z = (local_transform_ * transform).getRotation().z();
  msg.pose.orientation.w = (local_transform_ * transform).getRotation().w();
  return msg;
}

template <class PointT>
void NDTLocalizer<PointT>::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  std::cout << __func__ << std::endl;
  if (map_loaded_ == false)
  {
    ROS_INFO("received points. But map is not loaded");
    return;
  }
  else if(init_pos_set_ == false)
  {
    ROS_INFO("received points. But initial pose is not set");
    return;
  }

  //matching_start = std::chrono::system_clock::now();
  current_scan_header_ = input->header;
  current_scan_time_ = current_scan_header_.stamp;

  pcl::fromROSMsg(*input, scan_);
  boost::shared_ptr< pcl::PointCloud<PointT> > scan_ptr(new pcl::PointCloud<PointT>(scan_));

  int scan_points_num = scan_ptr->size();
  std::cout << "scan_points_num" << scan_points_num << std::endl;
  // Setting point cloud to be aligned.
  ndt_.setInputSource(scan_ptr);

  std_msgs::Time current_time;
  current_time.data = input->header.stamp;
  static std_msgs::Time previous_time = current_time;

  pose_corrector_msgs::Request req;
  req.pose = convertPosetoROSMsg(current_scan_header_, current_pose_);
  req.previous_time = previous_time;
  req.current_time = current_time;
  pose_corrector_pub_.publish(req);

  previous_time = current_time;
}



template <class PointT>
void NDTLocalizer<PointT>::poseCorrectorCallback(const pose_corrector_msgs::Response::ConstPtr& input)
{
  std::cout << __func__ << std::endl;

  //TODO: synchronize to points
  if(input->pose.header.stamp != current_scan_header_.stamp)
  {
    ROS_INFO("received pose correct. But its different to points stamp");
    return;
  }

  auto matching_start = std::chrono::system_clock::now();

  predict_pose_ = convertROSMsgtoPose(input);
  matching();
  calcStats();
  calcPoses();
  calcVelocities();

  auto matching_end = std::chrono::system_clock::now();
  exe_time_ = std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() / 1000.0;

  outputLogs();

  previous_pose_ = current_pose_;
}

template <class PointT>
void NDTLocalizer<PointT>::matching()
{
  std::cout << __func__ << std::endl;
  Eigen::Translation3f init_translation(predict_pose_.x, predict_pose_.y, predict_pose_.z);
  Eigen::AngleAxisf init_rotation_x(predict_pose_.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(predict_pose_.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(predict_pose_.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol_;

  boost::shared_ptr< pcl::PointCloud<PointT> > output_cloud(new pcl::PointCloud<PointT>);
  std::chrono::time_point<std::chrono::system_clock> align_start, align_end;
#ifdef USE_FAST_PCL
  if (use_openmp_ == true)
  {
    align_start = std::chrono::system_clock::now();
    ndt_.omp_align(*output_cloud, init_guess);
    align_end = std::chrono::system_clock::now();
  }
  else
  {
#endif
    align_start = std::chrono::system_clock::now();
    ndt_.align(*output_cloud, init_guess);
    align_end = std::chrono::system_clock::now();
#ifdef USE_FAST_PCL
  }
#endif

  align_time_ = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

}

template <class PointT>
void NDTLocalizer<PointT>::calcStats()
{

  std::chrono::time_point<std::chrono::system_clock> getFitnessScore_start, getFitnessScore_end;
  double fitness_score_;
#ifdef USE_FAST_PCL
  if (use_openmp_ == true)
  {
    getFitnessScore_start = std::chrono::system_clock::now();
    fitness_score_ = ndt_.omp_getFitnessScore();
    getFitnessScore_end = std::chrono::system_clock::now();
  }
  else
  {
#endif
    getFitnessScore_start = std::chrono::system_clock::now();
    fitness_score_ = ndt_.getFitnessScore();
    getFitnessScore_end = std::chrono::system_clock::now();
#ifdef USE_FAST_PCL
  }
#endif
  getFitnessScore_time_ = std::chrono::duration_cast<std::chrono::microseconds>(getFitnessScore_end - getFitnessScore_start).count() / 1000.0;


  const int iteration = ndt_.getFinalNumIteration();
  double trans_probability_ = ndt_.getTransformationProbability();

  // Set values for /ndt_stat
  ndt_localizer::ndt_stat ndt_stat_msg;
  ndt_stat_msg.header = current_scan_header_;
  ndt_stat_msg.exe_time = time_ndt_matching_.data;
  ndt_stat_msg.iteration = iteration;
  ndt_stat_msg.score = fitness_score_;
  ndt_stat_msg.velocity = current_velocity_.linear.xyz;
  ndt_stat_msg.acceleration = current_accel_.linear.xyz;
  ndt_stat_msg.use_predict_pose = 0;

  ndt_stat_pub_.publish(ndt_stat_msg);

  // Compute ndt_reliability
  ndt_reliability_.data = Wa * (exe_time_ / 100.0) * 100.0 + Wb * (iteration / 10.0) * 100.0 +
                           Wc * ((2.0 - trans_probability_) / 2.0) * 100.0;
  ndt_reliability_pub_.publish(ndt_reliability_);

}

template <class PointT>
void NDTLocalizer<PointT>::calcPoses()
{
  Eigen::Matrix4f t = ndt_.getFinalTransformation();  // localizer
  tf::Matrix3x3 mat_l;  // localizer
  mat_l.setValue(t(0, 0), t(0, 1), t(0, 2),
                 t(1, 0), t(1, 1), t(1, 2),
                 t(2, 0), t(2, 1), t(2, 2));

  // Update localizer_pose_
  localizer_pose_.x = t(0, 3);
  localizer_pose_.y = t(1, 3);
  localizer_pose_.z = t(2, 3);
  mat_l.getRPY(localizer_pose_.roll, localizer_pose_.pitch, localizer_pose_.yaw, 1);

  Eigen::Matrix4f t2 = t * tf_ltob_; // base_link
  tf::Matrix3x3 mat_b;  // base_link
  mat_b.setValue(t2(0, 0), t2(0, 1), t2(0, 2),
                 t2(1, 0), t2(1, 1), t2(1, 2),
                 t2(2, 0), t2(2, 1), t2(2, 2));

  // Update ndt_pose_
  ndt_pose_.x = t2(0, 3);
  ndt_pose_.y = t2(1, 3);
  ndt_pose_.z = t2(2, 3);
  mat_b.getRPY(ndt_pose_.roll, ndt_pose_.pitch, ndt_pose_.yaw, 1);

  // Calculate the difference between ndt_pose_ and predict_pose_
  predict_pose_error_ = sqrt((ndt_pose_.x - predict_pose_.x) * (ndt_pose_.x - predict_pose_.x) +
                              (ndt_pose_.y - predict_pose_.y) * (ndt_pose_.y - predict_pose_.y) +
                              (ndt_pose_.z - predict_pose_.z) * (ndt_pose_.z - predict_pose_.z));

  //TODO: use_predict_pose is not uesed
  int use_predict_pose;
  if (predict_pose_error_ <= PREDICT_POSE_THRESHOLD)
    use_predict_pose = 0;
  else
    use_predict_pose = 1;
  use_predict_pose = 0;

  if (use_predict_pose == 0)
    current_pose_ = ndt_pose_;
  else
    current_pose_ = predict_pose_;

  geometry_msgs::PoseStamped predict_pose_msg 
    = use_local_transform_== true 
    ? convertPosetoROSMsg(current_scan_header_, predict_pose_, local_transform_) 
    : convertPosetoROSMsg(current_scan_header_, predict_pose_);
  predict_pose_pub_.publish(predict_pose_msg);

  geometry_msgs::PoseStamped ndt_pose_msg
    = use_local_transform_== true 
    ? convertPosetoROSMsg(current_scan_header_, ndt_pose_, local_transform_) 
    : convertPosetoROSMsg(current_scan_header_, ndt_pose_);
  ndt_pose_pub_.publish(ndt_pose_msg);

  tf::Quaternion current_q;
  current_q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
  /*
  // current_pose_ is published by vel_pose_mux
  geometry_msgs::PoseStamped current_pose_msg
    = use_local_transform_== true 
    ? convertPosetoROSMsg(current_scan_header_, current_pose_, local_transform_) 
    : convertPosetoROSMsg(current_scan_header_, current_pose_);
  current_pose_pub_.publish(current_pose_msg);
  */

  geometry_msgs::PoseStamped localizer_pose_msg
    = use_local_transform_== true 
    ? convertPosetoROSMsg(current_scan_header_, localizer_pose_, local_transform_) 
    : convertPosetoROSMsg(current_scan_header_, localizer_pose_);
  localizer_pose_pub_.publish(localizer_pose_msg);

  // Send TF "/base_link" to "/map"
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(current_pose_.x, current_pose_.y, current_pose_.z));
  transform.setRotation(current_q);
  if (use_local_transform_ == true)
    tf_broadcaster_.sendTransform(tf::StampedTransform(local_transform_ * transform, current_scan_time_, "/map", "/base_link"));
  else
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, current_scan_time_, "/map", "/base_link"));

}

template <class PointT>
void NDTLocalizer<PointT>::calcVelocities()
{
  // Compute the velocity and acceleration
  static ros::Time previous_scan_time = current_scan_time_;
  const double time_diff_sec = (current_scan_time_ - previous_scan_time).toSec();
  current_velocity_.setVelocity(current_pose_, previous_pose_, time_diff_sec);

  current_velocity_smooth_ = (current_velocity_.linear.xyz + previous_velocity_.linear.xyz + previous_previous_velocity_.linear.xyz) / 3.0;
  if(current_velocity_smooth_ < 0.2)
    current_velocity_smooth_ = 0.0;

  current_accel_.setAccel(current_velocity_, previous_velocity_, time_diff_sec);

  std_msgs::Float32 estimated_vel_mps, estimated_vel_kmph;
  estimated_vel_mps.data = current_velocity_.linear.xyz;
  estimated_vel_kmph.data = current_velocity_.linear.xyz * 3.6;

  estimated_vel_mps_pub_.publish(estimated_vel_mps);
  estimated_vel_kmph_pub_.publish(estimated_vel_kmph);

  time_ndt_matching_.data = exe_time_;
  time_ndt_matching_pub_.publish(time_ndt_matching_);

  // Set values for /estimate_twist
  geometry_msgs::TwistStamped estimate_twist_msg;
  estimate_twist_msg.header = current_scan_header_;
  estimate_twist_msg.header.frame_id = "/base_link";
  estimate_twist_msg.twist.linear.x = current_velocity_.linear.xyz;
  estimate_twist_msg.twist.linear.y = 0.0;
  estimate_twist_msg.twist.linear.z = 0.0;
  estimate_twist_msg.twist.angular.x = 0.0;
  estimate_twist_msg.twist.angular.y = 0.0;
  estimate_twist_msg.twist.angular.z = current_velocity_.angular.z;

  estimate_twist_pub_.publish(estimate_twist_msg);

  geometry_msgs::Vector3Stamped estimate_vel_msg;
  estimate_vel_msg.header = current_scan_header_;
  estimate_vel_msg.vector.x = current_velocity_.linear.xyz;
  estimated_vel_pub_.publish(estimate_vel_msg);
  
  previous_previous_velocity_ = previous_velocity_;
  previous_velocity_ = current_velocity_;

}

template <class PointT>
void NDTLocalizer<PointT>::outputLogs()
{
  if (!ofs_)
  {
    //std::cerr << "Could not open " << fileame << "." << std::endl;
    std::cerr << "Could not open log file." << std::endl;
    exit(1);
  }

  ofs_ << current_scan_header_.seq << "," << scan_.size() << "," << step_size_ << "," << trans_eps_ << "," << std::fixed
       << std::setprecision(5) << current_pose_.x << "," << std::fixed << std::setprecision(5) << current_pose_.y << ","
       << std::fixed << std::setprecision(5) << current_pose_.z << "," << current_pose_.roll << "," << current_pose_.pitch
       << "," << current_pose_.yaw << "," << predict_pose_.x << "," << predict_pose_.y << "," << predict_pose_.z << ","
       << predict_pose_.roll << "," << predict_pose_.pitch << "," << predict_pose_.yaw << ","
       << current_pose_.x - predict_pose_.x << "," << current_pose_.y - predict_pose_.y << ","
       << current_pose_.z - predict_pose_.z << "," << current_pose_.roll - predict_pose_.roll << ","
       << current_pose_.pitch - predict_pose_.pitch << "," << current_pose_.yaw - predict_pose_.yaw << ","
       << predict_pose_error_ << "," << ndt_.getFinalNumIteration() << "," << fitness_score_ << "," << trans_probability_ << ","
       << ndt_reliability_.data << "," << current_velocity_.linear.xyz << "," << current_velocity_smooth_ << "," << current_accel_.linear.xyz
       << "," << current_velocity_.angular.z << "," << time_ndt_matching_.data << "," << align_time_ << "," << getFitnessScore_time_
       << std::endl;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence: " << current_scan_header_.seq << std::endl;
  std::cout << "Timestamp: " << current_scan_header_.stamp << std::endl;
  std::cout << "Frame ID: " << current_scan_header_.frame_id << std::endl;
  std::cout << "Number of Scan Points: " << scan_.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << ndt_.hasConverged() << std::endl;
  std::cout << "Fitness Score: " << fitness_score_ << std::endl;
  std::cout << "Transformation Probability: " << ndt_.getTransformationProbability() << std::endl;
  std::cout << "Execution Time: " << exe_time_ << " ms." << std::endl;
  std::cout << "Number of Iterations: " << ndt_.getFinalNumIteration() << std::endl;
  std::cout << "NDT Reliability: " << ndt_reliability_.data << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw): " << std::endl;
  std::cout << "(" << current_pose_.x << ", " << current_pose_.y << ", " << current_pose_.z 
            << ", " << current_pose_.roll << ", " << current_pose_.pitch << ", " << current_pose_.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix: " << std::endl;
  std::cout << ndt_.getFinalTransformation() << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

}
