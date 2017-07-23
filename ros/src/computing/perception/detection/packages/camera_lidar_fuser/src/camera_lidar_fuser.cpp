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

#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

//TODO:
//1. Sync:
//    -Clusters
//    -Image Detections
//    -Image Frame
//    -Extrinsic Calibration


class CameraLidarFuser
{
private:

	ros::NodeHandle node_handle_;
	ros::Subscriber extrinsic_sub_;
	ros::Publisher cloud_clusters_class_pub_;


	typedef message_filters::sync_policies::ApproximateTime<autoware_msgs::CloudClusterArray,
															autoware_msgs::DetectedObjectArray,
															autoware_msgs::DetectedObjectArray> FusionSyncPolicy;

	message_filters::Subscriber<autoware_msgs::CloudClusterArray> *cloud_clusters_sub_;
	message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *car_detection_sub_;
	message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *person_detection_sub_;

	message_filters::Synchronizer<FusionSyncPolicy> *sync_;
public:
	CameraLidarFuser() :
		node_handle_("~")
	{
		cloud_clusters_sub_ = node_handle_.subscribe("/cloud_clusters", 10, &CameraLidarFuser::CloudClustersCallback, this);
		cloud_clusters_class_pub_ = node_handle_.advertise<autoware_msgs::CloudClusterArray>( "/detected_objects", 10);
	}


	void SyncedCallback(const autoware_msgs::CloudClusterArray::ConstPtr& in_lidar_detections,
								const autoware_msgs::DetectedObjectArray::ConstPtr& in_car_image_detections,
								const autoware_msgs::DetectedObjectArray::ConstPtr& in_person_image_detections)
	{

	}

	void Run()
	{
		cloud_clusters_sub_ = new message_filters::Subscriber<autoware_msgs::CloudClusterArray>(node_handle_, "cloud_clusters", 10);
		car_detection_sub_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_, "/obj_car/image_obj", 10);
		person_detection_sub_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(node_handle_, "/obj_person/image_obj", 10);

		sync_ = new message_filters::Synchronizer<FusionSyncPolicy>(TrackerS);

		sync_->registerCallback(boost::bind(&CameraLidarFuser::SyncedCallback, this, _1, _2, _3));

		ros::spin();
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_lidar_fuser");

	CameraLidarFuser node;

	node.Run();

	return 0;
}

