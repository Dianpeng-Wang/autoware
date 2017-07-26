/*
 * camera_lidar_fuser.cpp
 *
 *  Created on: Jul 18, 2017
 *      Author: ne0
 */

#include "ros/ros.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <autoware_msgs/CloudCluster.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/Pictogram.h>
#include <jsk_rviz_plugins/PictogramArray.h>

//TODO:
//1. Sync:
//    -Clusters
//    -Image Detections
//    -Image Frame
//    -Extrinsic Calibration


class CameraLidarFuser
{
private:

	ros::NodeHandle* 	node_handle_;
	ros::Publisher 		cloud_clusters_class_pub_;
	ros::Publisher 		bounding_box__class_pub_;

	typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::CloudClusterArray,
															autoware_msgs::DetectedObjectArray,
															autoware_msgs::DetectedObjectArray> FusionSyncPolicy;

	//message_filters::Subscriber<autoware_msgs::CloudClusterArray> extrinsic_sub_;
	message_filters::Subscriber<autoware_msgs::CloudClusterArray> cloud_clusters_sub_;
	message_filters::Subscriber<autoware_msgs::DetectedObjectArray> car_detection_sub_;
	message_filters::Subscriber<autoware_msgs::DetectedObjectArray> person_detection_sub_;

	message_filters::Synchronizer<FusionSyncPolicy> fusion_sync_;

public:
	void SyncedCallback(const autoware_msgs::CloudClusterArray::ConstPtr& in_lidar_detections,
									const autoware_msgs::DetectedObjectArray::ConstPtr& in_car_image_detections,
									const autoware_msgs::DetectedObjectArray::ConstPtr& in_person_image_detections)
	{
		ROS_INFO("Sync OK");
	}

	CameraLidarFuser(ros::NodeHandle* in_handle) :
		node_handle_(in_handle),
		cloud_clusters_sub_(*node_handle_, "cloud_clusters" , 1),
		car_detection_sub_(*node_handle_, "obj_car/image_obj" , 1),
		person_detection_sub_(*node_handle_, "obj_person/image_obj" , 1),
		//extrinsic_sub_(*node_handle_, "obj_person/image_obj" , 1),
		fusion_sync_(FusionSyncPolicy(1), cloud_clusters_sub_, car_detection_sub_, person_detection_sub_)
	{

		fusion_sync_.registerCallback(boost::bind(&CameraLidarFuser::SyncedCallback, this, _1, _2, _3));

		cloud_clusters_class_pub_ = node_handle_->advertise<autoware_msgs::CloudClusterArray>("/cloud_clusters_class",1);
		bounding_box__class_pub_ = node_handle_->advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_box_class",1);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_lidar_fuser");

	ros::NodeHandle node_handle;
	ros::NodeHandle private_node_handle("~");//to receive args

	CameraLidarFuser fusion_node(&node_handle);

	ros::Rate loop_rate(10);
	ROS_INFO("camera_lidar_fuser: Waiting for /cloud_clusters, /obj_car/image_obj, /obj_person/image_obj and ");
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

