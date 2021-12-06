/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2021 Patrick Geneva
 * Copyright (C) 2021 Guoquan Huang
 * Copyright (C) 2021 OpenVINS Contributors
 * Copyright (C) 2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

#include "core/RosVisualizer.h"
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/dataset_reader.h"
#include "utils/parse_ros.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

std::shared_ptr<VioManager>    sys;
std::shared_ptr<RosVisualizer> viz;

using cam_param_type = std::map<size_t, Eigen::VectorXd>;
using cam_size_type  = std::map<size_t, std::pair<int, int>>;
using tf_buffer_ptr    = std::shared_ptr<tf2_ros::Buffer>;
using tf_transform_ptr = std::shared_ptr<tf2_ros::TransformListener>;

tf_buffer_ptr    tf_;
tf_transform_ptr tf_listener_;

cam_param_type    cam_ins, cam_ex {};
cam_size_type     cam_size {};
VioManagerOptions params {};

ros::Publisher              tf_publisher {};
Eigen::Matrix<double, 7, 1> cam_left_eigen {};
Eigen::Matrix<double, 7, 1> cam_right_eigen {};

bool left_init  = false;
bool right_init = false;
bool tf_init    = false;

// Callback functions
void callback_inertial(const sensor_msgs::Imu::ConstPtr& msg);

void callback_monocular(const sensor_msgs::ImageConstPtr& msg0, int cam_id0);

void callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1, int cam_id0,
                     int cam_id1);

void camera_left_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg, ros::NodeHandle& nh);

void camera_right_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg, ros::NodeHandle& nh);

void tf_static_callback(tf2_msgs::TFMessage const& data);

void get_tf_transform();

void publish_pose(ros::Publisher& pub);

// Main function
int main(int argc, char** argv) {

	// Launch our ros node
	ros::init(argc, argv, "run_subscribe_msckf");
	ros::NodeHandle nh("~");

	// Create our VIO system
//	ros::Subscriber tf_sub                = nh.subscribe("/tf", 100, tf_static_callback);
	ros::Subscriber camera_info_left_sub  = nh
			.subscribe<sensor_msgs::CameraInfo>("/camera/stereo/left/camera_info", 100,
			                                    boost::bind(&camera_left_info_callback, _1, nh));
	ros::Subscriber camera_info_right_sub = nh
			.subscribe<sensor_msgs::CameraInfo>("/camera/stereo/right/camera_info", 100,
			                                    boost::bind(&camera_right_info_callback, _1, nh));
	tf_publisher = nh.advertise<geometry_msgs::PoseArray>("tf_pose", 2, true);

	params = parse_ros_nodehandler(nh);

	tf_.reset(new tf2_ros::Buffer(ros::Duration(100)));
	tf_listener_.reset(new tf2_ros::TransformListener(*tf_));

//	params.set_cam_param(cam_ins, cam_ex, cam_size);

//	sys = std::make_shared<VioManager>(params);
//	viz = std::make_shared<RosVisualizer>(nh, sys);

	//===================================================================================
	//===================================================================================
	//===================================================================================

	// Create imu subscriber
	std::string topic_imu;
	nh.param<std::string>("topic_imu", topic_imu, "/imu0");
	ros::Subscriber subimu = nh.subscribe(topic_imu, 9999999, callback_inertial);

	// Create camera subscriber data vectors
	std::vector<ros::Subscriber>                                                                    subs_cam;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
	std::vector<std::unique_ptr<message_filters::Synchronizer<sync_pol>>>                           sync_cam;
	std::vector<std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>>                   sync_subs_cam;

	// Logic for sync stereo subscriber
	// https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491
	if (params.state_options.num_cameras == 2) {
		// Read in the topics
		std::string cam_topic0, cam_topic1;
		nh.param<std::string>("topic_camera" + std::to_string(0), cam_topic0,
		                      "/cam" + std::to_string(0) + "/image_raw");
		nh.param<std::string>("topic_camera" + std::to_string(1), cam_topic1,
		                      "/cam" + std::to_string(1) + "/image_raw");
		// Create sync filter (they have unique pointers internally, so we have to use move logic here...)
		auto image_sub0 = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
				new message_filters::Subscriber<sensor_msgs::Image>(nh, cam_topic0, 5));
		auto image_sub1 = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
				new message_filters::Subscriber<sensor_msgs::Image>(nh, cam_topic1, 5));
		auto sync       = std::unique_ptr<message_filters::Synchronizer<sync_pol>>(
				new message_filters::Synchronizer<sync_pol>(sync_pol(5), *image_sub0, *image_sub1));
		sync->registerCallback(boost::bind(&callback_stereo, _1, _2, 0, 1));
		// Append to our vector of subscribers
		sync_cam.push_back(std::move(sync));
		sync_subs_cam.push_back(std::move(image_sub0));
		sync_subs_cam.push_back(std::move(image_sub1));
		ROS_INFO("subscribing to cam (stereo): %s", cam_topic0.c_str());
		ROS_INFO("subscribing to cam (stereo): %s", cam_topic1.c_str());
	} else {
		// Now we should add any non-stereo callbacks here
		for (int i = 0; i < params.state_options.num_cameras; i++) {
			// read in the topic
			std::string cam_topic;
			nh.param<std::string>("topic_camera" + std::to_string(i), cam_topic,
			                      "/cam" + std::to_string(i) + "/image_raw");
			// create subscriber
			subs_cam.push_back(nh.subscribe<sensor_msgs::Image>(cam_topic, 5, boost::bind(callback_monocular, _1, i)));
			ROS_INFO("subscribing to cam (mono): %s", cam_topic.c_str());
		}
	}

	//===================================================================================
	//===================================================================================
	//===================================================================================

	// Spin off to ROS
	// TODO: maybe should use multi-thread spinner
	// TODO: but need to support multi-threaded calls to viomanager
	ROS_INFO("done...spinning to ros");
	ros::spin();

	// Final visualization
	viz->visualize_final();

	// Done!
	return EXIT_SUCCESS;
}

void callback_inertial(const sensor_msgs::Imu::ConstPtr& msg) {
	if (!tf_init || !right_init || !left_init) {
		ROS_ERROR("Imu call back, not init success return!");
		return;
	}

	// convert into correct format
	ov_core::ImuData message;
	message.timestamp = msg->header.stamp.toSec();
	message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
	message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

	// send it to our VIO system
	sys->feed_measurement_imu(message);
	viz->visualize();
	viz->visualize_odometry(message.timestamp);
}

void callback_monocular(const sensor_msgs::ImageConstPtr& msg0, int cam_id0) {
	if (!tf_init || !right_init || !left_init) {
		ROS_ERROR("monocular call back, not init success return!");
		return;
	}

	// Get the image
	cv_bridge::CvImageConstPtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Create the measurement
	ov_core::CameraData message;
	message.timestamp = cv_ptr->header.stamp.toSec();
	message.sensor_ids.push_back(cam_id0);
	message.images.push_back(cv_ptr->image.clone());

	// Load the mask if we are using it, else it is empty
	// TODO: in the future we should get this from external pixel segmentation
	if (sys->get_params().use_mask) {
		message.masks.push_back(sys->get_params().masks.at(cam_id0));
	} else {
		message.masks.push_back(cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1));
	}

	// send it to our VIO system
	sys->feed_measurement_camera(message);
}

void callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1, int cam_id0,
                     int cam_id1) {
	if (!tf_init || !right_init || !left_init) {
		ROS_ERROR("stereo call back, not init success return!");
		return;
	}

	// Get the image
	cv_bridge::CvImageConstPtr cv_ptr0;
	try {
		cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Get the image
	cv_bridge::CvImageConstPtr cv_ptr1;
	try {
		cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Create the measurement
	ov_core::CameraData message;
	message.timestamp = cv_ptr0->header.stamp.toSec();
	message.sensor_ids.push_back(cam_id0);
	message.sensor_ids.push_back(cam_id1);
	message.images.push_back(cv_ptr0->image.clone());
	message.images.push_back(cv_ptr1->image.clone());

	// Load the mask if we are using it, else it is empty
	// TODO: in the future we should get this from external pixel segmentation
	if (sys->get_params().use_mask) {
		message.masks.push_back(sys->get_params().masks.at(cam_id0));
		message.masks.push_back(sys->get_params().masks.at(cam_id1));
	} else {
		// message.masks.push_back(cv::Mat(cv_ptr0->image.rows, cv_ptr0->image.cols, CV_8UC1, cv::Scalar(255)));
		message.masks.push_back(cv::Mat::zeros(cv_ptr0->image.rows, cv_ptr0->image.cols, CV_8UC1));
		message.masks.push_back(cv::Mat::zeros(cv_ptr1->image.rows, cv_ptr1->image.cols, CV_8UC1));
	}

	// send it to our VIO system
	sys->feed_measurement_camera(message);
}

void camera_right_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg, ros::NodeHandle& nh) {
	if (!tf_init) {
		get_tf_transform();
	}

	if (right_init) {
		return;
	}

	Eigen::Matrix<double, 8, 1> cam_calib;
	cam_calib << msg->K[0], msg->K[4], msg->K[2], msg->K[5], msg->D[0], msg->D[1], msg->D[2], msg->D[3];
	cam_ins.insert({1, cam_calib});
	cam_size.insert({1, std::make_pair(msg->width, msg->height)});
	right_init = true;
	ROS_INFO("right cam info call back!");
	ROS_INFO_STREAM("right cam info callback! ins is " << cam_ins[1] << "\n size is " << cam_size[1].first << " "
	                                                   << cam_size[0].second);

	if (tf_init & right_init & left_init) {
		ROS_ERROR("right call back, init success !");
		params.set_cam_param(cam_ex, cam_ins, cam_size);
		sys = std::make_shared<VioManager>(params);
		viz = std::make_shared<RosVisualizer>(nh, sys);
		publish_pose(tf_publisher);
	}
}

void camera_left_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg, ros::NodeHandle& nh) {
	if (!tf_init) {
		get_tf_transform();
	}

	if (left_init) {
		return;
	}

	Eigen::Matrix<double, 8, 1> cam_calib;
	cam_calib << msg->K[0], msg->K[4], msg->K[2], msg->K[5], msg->D[0], msg->D[1], msg->D[2], msg->D[3];
	cam_ins.insert({0, cam_calib});
	cam_size.insert({0, std::make_pair(msg->width, msg->height)});
	left_init = true;
	ROS_INFO_STREAM("left cam info callback! ins is " << cam_ins[0] << "\n size is " << cam_size[0].first << " "
	                                                  << cam_size[0].second);

	if (tf_init & right_init & left_init) {
		ROS_ERROR("left call back, init success !");
		params.set_cam_param(cam_ins, cam_ex, cam_size);
		sys = std::make_shared<VioManager>(params);
		viz = std::make_shared<RosVisualizer>(nh, sys);
		publish_pose(tf_publisher);
	}
}

void tf_static_callback(tf2_msgs::TFMessage const& tf_msg) {
	for (auto tf: tf_msg.transforms) {
		if (tf.child_frame_id == "imu") {
			auto lidar_offset = static_cast<float>(tf.transform.translation.x);
			auto yaw_angle    = tf::getYaw(tf.transform.rotation);
			return;
		}
	}

	ROS_INFO("Tf static receive!");
}

void get_tf_transform() {
	try {
		auto Tic_left = tf_->lookupTransform("imu_Link", "camera_link", ros::Time(5.0));

//		tf::Transform  Tic {};
//		tf::Quaternion Ric {};
//		Tic.setOrigin(tf::Vector3 {Tic_left.transform.translation.x, Tic_left.transform.translation.y,
//		                           Tic_left.transform.translation.z});
//		Ric.setX(Tic_left.transform.rotation.x);
//		Ric.setY(Tic_left.transform.rotation.y);
//		Ric.setZ(Tic_left.transform.rotation.z);
//		Ric.setW(Tic_left.transform.rotation.w);
//		Tic.setRotation(Ric);
//
//		auto Rci = Tic.inverse();
//		Tic_left.transform.translation.x = Rci.getOrigin().x();
//		Tic_left.transform.translation.y = Rci.getOrigin().y();
//		Tic_left.transform.translation.z = Rci.getOrigin().z();
//
//		auto            matrix = Tic.getBasis().inverse();
//		Eigen::Matrix3d Mtic;
//		Mtic << matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0], matrix[1][1],
//				matrix[1][2], matrix[2][0], matrix[2][1], matrix[2][2];
//
//		auto jpl = rot_2_quat(Mtic);
//
//		Tic_left.transform.rotation.x = jpl(0, 0);
//		Tic_left.transform.rotation.y = jpl(1, 0);
//		Tic_left.transform.rotation.z = jpl(2, 0);
//		Tic_left.transform.rotation.w = jpl(3, 0);

		ROS_INFO_STREAM("IMU to cam transform is " << Tic_left.transform);

		cam_left_eigen.block(0, 0, 4, 1) << Tic_left.transform.rotation.x, Tic_left.transform.rotation.y, Tic_left
				.transform.rotation.z, Tic_left.transform.rotation.w;
		cam_left_eigen.block(4, 0, 3, 1) << Tic_left.transform.translation.x, Tic_left.transform.translation.y, Tic_left
				.transform.translation.z;

		cam_right_eigen.block(0, 0, 4, 1) << Tic_left.transform.rotation.x, Tic_left.transform.rotation.y, Tic_left
				.transform.rotation.z, Tic_left.transform.rotation.w;

		tf::Quaternion rico {};
		tf::Transform  Ric0 {};
		rico.setX(Tic_left.transform.rotation.x);
		rico.setY(Tic_left.transform.rotation.y);
		rico.setZ(Tic_left.transform.rotation.z);
		rico.setW(Tic_left.transform.rotation.w);
		Ric0.setRotation(rico);

		Ric0.setOrigin(tf::Vector3 {Tic_left.transform.translation.x, Tic_left.transform.translation.y,
		                            Tic_left.transform.translation.z});

		//Ric1 = Ric0*Rc0c1
		tf::Transform  Rc0c1 {};
		tf::Quaternion rc0c1(0., 0., 0., 1.f);
		Rc0c1.setOrigin(tf::Vector3 {0.12f, 0., 0.});
//		rc0c1.normalize();
		Rc0c1.setRotation(rc0c1);

		auto Ric1 = Ric0 * Rc0c1;

//		cam_right_eigen.block(4, 0, 3, 1) << Tic_left.transform.translation.x + 0.12,
//				Tic_left.transform.translation.y, Tic_left.transform.translation.z;


		ROS_INFO_STREAM("Before right is " << cam_right_eigen);

		cam_right_eigen.block(4, 0, 3, 1) << Ric1.getOrigin().x(), Ric1.getOrigin().y(), Ric1.getOrigin().z();

		ROS_INFO_STREAM("After right is " << cam_right_eigen);
		ROS_INFO_STREAM("After left is " << cam_left_eigen);

		cam_ex.insert({0, cam_left_eigen});
		cam_ex.insert({1, cam_right_eigen});
		ROS_INFO("Get imu to cam left param");
		tf_init = true;
	} catch (tf2::TransformException& e) {
		ROS_INFO_THROTTLE(1, "%s", e.what());
	}
}

void publish_pose(ros::Publisher& pub) {
	geometry_msgs::PoseArray pose {};
	geometry_msgs::Pose      cam_left, cam_right;

	cam_left.orientation.x = cam_left_eigen(0, 0);
	cam_left.orientation.y = cam_left_eigen(1, 0);
	cam_left.orientation.z = cam_left_eigen(2, 0);
	cam_left.orientation.w = cam_left_eigen(3, 0);
	cam_left.position.x    = cam_left_eigen(4, 0);
	cam_left.position.y    = cam_left_eigen(5, 0);
	cam_left.position.z    = cam_left_eigen(6, 0);

	cam_right.orientation.x = cam_right_eigen(0, 0);
	cam_right.orientation.y = cam_right_eigen(1, 0);
	cam_right.orientation.z = cam_right_eigen(2, 0);
	cam_right.orientation.w = cam_right_eigen(3, 0);
	cam_right.position.x    = cam_right_eigen(4, 0);
	cam_right.position.y    = cam_right_eigen(5, 0);
	cam_right.position.z    = cam_right_eigen(6, 0);

	pose.poses.push_back(cam_left);
	pose.poses.push_back(cam_right);
	pose.header.frame_id = "global";
	pub.publish(pose);
}
