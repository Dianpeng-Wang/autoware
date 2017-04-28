/*
 *  Copyright (c) 2016, Nagoya University
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
#include <ros/ros.h>
#include "ros/package.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <thread>
#include <signal.h>
#include<unistd.h>

#include "VectaCam.h"

class RosVectaCam
{
public:
	void Run()
	{
		std::string config_file_path;
		std::string camera_ip;
		std::string camera_intrinsic_calibration_file;

		ros::NodeHandle private_node_handle("~");

		ROS_INFO("%s", ros::package::getPath("vectacam").c_str());

		if (private_node_handle.getParam("configfile", config_file_path))
		{
			ROS_INFO("Setting config file path to %s", config_file_path.c_str());
		}
		else
		{
			config_file_path = ros::package::getPath("vectacam")+"/initialization_params.txt";
			ROS_INFO("No config file received. trying default... %s", config_file_path.c_str());
			//return;
		}
		if (private_node_handle.getParam("camera_ip", camera_ip))
		{
			ROS_INFO("Setting camera IP to %s", camera_ip.c_str());
		}
		else
		{
			ROS_INFO("No IP, defaulting to %s, you can use _img_obj_node:=YOUR_TOPIC", VECTACAM_CAMERA_IP);
			camera_ip = VECTACAM_CAMERA_IP;
		}

		if (private_node_handle.getParam("camera_intrinsic_calibration_file", camera_intrinsic_calibration_file))
		{
			ROS_INFO("Camera Intrinsic Calibration File %s", camera_intrinsic_calibration_file.c_str());
		}
		else
		{
			ROS_INFO("No Calibration, defaulting to %s, you can use _img_obj_node:=YOUR_TOPIC", VECTACAM_CAMERA_IP);
			camera_intrinsic_calibration_file = ros::package::getPath("vectacam")+"/vectacam_intrinsic.yaml";
		}

		bool camera_info_ready = _getMatricesFromFile(camera_intrinsic_calibration_file, camerainfo_msg_);

		for (unsigned int i=0; i< VECTACAM_NUM_CAMERAS; i++)
		{
			std::string current_topic = "camera" + std::to_string(i);
			publishers_cameras_[i] = node_handle_.advertise<sensor_msgs::Image>(current_topic+ "/image_raw", 1);

			if (camera_info_ready)
			{
				publishers_camera_info_[i] = node_handle_.advertise<sensor_msgs::CameraInfo>(current_topic+"/camera_info", 1);
			}
		}

		VectaCam vectacamera(VECTACAM_CONFIG_PORT, VECTACAM_DATA_PORT, config_file_path);
		std::thread *capture_thread= new std::thread(&VectaCam::StartCamera, &vectacamera);

		cv::Mat image;
		std::vector<cv::Mat> camera_images(VECTACAM_NUM_CAMERAS);
		unsigned long int counter = 0;
		ros::Rate loop_rate(7); // Hz
		ros::Publisher full_publisher = node_handle_.advertise<sensor_msgs::Image>("image_raw", 1);
		ros::Publisher full_publisher_info = node_handle_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
		while(vectacamera.IsReady() && ros::ok())
		{
			vectacamera.GetImage(image);
			if(!image.empty())
			{
				cv::flip(image, image, 0);
				ros::Time current_time=ros::Time::now();
				_publish_image(image, full_publisher, counter, current_time);
				_publish_camera_info(full_publisher_info, counter, current_time);
				for (unsigned int i=0; i< VECTACAM_NUM_CAMERAS; i++)
				{
					camera_images[i]= image(cv::Rect(i*image.cols/VECTACAM_NUM_CAMERAS, 0, image.cols/VECTACAM_NUM_CAMERAS,image.rows));
					//if(!camera_images[i].empty())

						_publish_image(camera_images[i], publishers_cameras_[i], counter, current_time);
						_publish_camera_info(publishers_camera_info_[i], counter, current_time);
					//else
						//std::cout << "Empty frame from image: " << i << " at frame " << counter << std::endl;
				}
				//_publish_image(image, publishers_cameras_[NUM_CAMERAS], counter);

				counter++;
				if (counter<=2)
					std::cout << "Image received" << std::endl;
			}
			else
			{
				std::cout << "No image received" << std::endl;
			}
			loop_rate.sleep();
		}
		vectacamera.StopCamera();
		capture_thread->join();
		delete capture_thread;
	}
private:
	ros::Publisher 		publishers_cameras_[VECTACAM_NUM_CAMERAS];
	ros::Publisher 		publishers_camera_info_[VECTACAM_NUM_CAMERAS];

	ros::NodeHandle 	node_handle_;
	sensor_msgs::CameraInfo camerainfo_msg_;

	void _publish_image(cv::Mat &in_image, ros::Publisher &in_publisher, unsigned long int in_counter, ros::Time current_time)
	{
		sensor_msgs::ImagePtr msg;
		std_msgs::Header header;
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", in_image).toImageMsg();
		msg->header.frame_id = "camera";
		msg->header.stamp.sec = current_time.sec;
		msg->header.stamp.nsec = current_time.nsec;
		msg->header.seq = in_counter;

		in_publisher.publish(msg);
	}

	void _publish_camera_info(ros::Publisher &in_publisher, unsigned long int in_counter, ros::Time current_time)
	{
		camerainfo_msg_.header.stamp.sec =current_time.sec;
		camerainfo_msg_.header.stamp.nsec =current_time.nsec;
		camerainfo_msg_.header.seq = in_counter;

		in_publisher.publish(camerainfo_msg_);
	}

	bool _getMatricesFromFile(std::string filename, sensor_msgs::CameraInfo &camerainfo_msg)
	{
		//////////////////CAMERA INFO/////////////////////////////////////////
		cv::Mat  cameraExtrinsicMat;
		cv::Mat  cameraMat;
		cv::Mat  distCoeff;
		cv::Size imageSize;

		if (filename!="")
		{
			ROS_INFO("Trying to parse calibrationfile :");
			ROS_INFO("> %s", filename.c_str());
		}
		else
		{
			ROS_INFO("No calibrationfile param was received");
			return false;
		}

		cv::FileStorage fs(filename, cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			ROS_INFO("Cannot open %s", filename.c_str());;
			return false;
		}
		else
		{
			fs["CameraMat"] >> cameraMat;
			fs["DistCoeff"] >> distCoeff;
			fs["ImageSize"] >> imageSize;
		}
		_parseCameraInfo(cameraMat, distCoeff, imageSize, camerainfo_msg);
		return true;
	}

	void _parseCameraInfo(const cv::Mat  &camMat,
	                       const cv::Mat  &disCoeff,
	                       const cv::Size &imgSize,
	                       sensor_msgs::CameraInfo &msg)
	{
		msg.header.frame_id = "camera";
		//  msg.header.stamp    = ros::Time::now();

		msg.height = imgSize.height;
		msg.width  = imgSize.width;

		for (int row=0; row<3; row++)
		{
			for (int col=0; col<3; col++)
			{
				msg.K[row * 3 + col] = camMat.at<double>(row, col);
			}
		}

		for (int row=0; row<3; row++)
		{
			for (int col=0; col<4; col++)
			{
				if (col == 3)
				{
					msg.P[row * 4 + col] = 0.0f;
				} else
				{
					msg.P[row * 4 + col] = camMat.at<double>(row, col);
				}
			}
		}

		for (int row=0; row<disCoeff.rows; row++)
		{
			for (int col=0; col<disCoeff.cols; col++)
			{
				msg.D.push_back(disCoeff.at<double>(row, col));
			}
		}
	}
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "vectacam");

	RosVectaCam app;

	app.Run();

	return 0;
}
