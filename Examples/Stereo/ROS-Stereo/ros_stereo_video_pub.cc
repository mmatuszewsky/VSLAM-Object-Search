/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include<System.h>
#include<Converter.h>

using namespace std;

void Publish(ORB_SLAM3::System &SLAM, ros::Publisher &pub_pts_and_pose, ros::Publisher &pub_all_kf_and_pts, 
	ros::Publisher &pub_cloud, ros::Publisher &pub_camera_pose, int frame_id);


// ./publisher_video_node /home/meciek/VSLAM-Object-Search/Vocabulary/ORBvoc.txt /home/meciek/VSLAM-Object-Search/Examples/Stereo/oak-d.yaml /home/meciek/Downloads/oakd

bool read_from_topic = false; 
bool read_from_camera = false;
bool pub_all_pts = false;
int pub_count = 0;
int all_pts_pub_gap = 0;


int main(int argc, char **argv)
{  
    cout << "Number of args: " << argc << endl;
	ros::init(argc, argv, "Stereo");
	ros::start();

	if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_video path_to_vocabulary path_to_settings path_to_videos" << endl;
        return 1;
    }

    cout << "Initializing video inputs from " << argv[3] << endl;

    string leftFile = string(argv[3]) + "/left.mp4";
    string rightFile = string(argv[3]) + "/right.mp4";

    cv::VideoCapture capLeft(leftFile);
    cv::VideoCapture capRight(rightFile);

    if (!capLeft.isOpened() || !capRight.isOpened()) {
        cerr << "Error: Could not open one or both videos." << endl;
        return -1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    cout << "Creating VSLAM System ..." << endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
	float imageScale = SLAM.GetImageScale();

    // ROS section
	ros::NodeHandle nodeHandler;
	ros::Publisher pub_cloud = nodeHandler.advertise<sensor_msgs::PointCloud2>("cloud_in", 1000);
	ros::Publisher pub_pts_and_pose = nodeHandler.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
	ros::Publisher pub_all_kf_and_pts = nodeHandler.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
	ros::Publisher pub_camera_pose = nodeHandler.advertise<geometry_msgs::Pose>("camera_pose", 1000);
	cout << endl << "Started publisher topics!" << endl;

    // ros::Rate loop_rate(1);

    cv::Mat imLeft, imRight;
    double t_resize = 0;
    double t_rect = 0;
    double t_track = 0;
    int num_rect = 0;
    
    while (capLeft.isOpened() && capRight.isOpened()) {

        capLeft.read(imLeft);
        capRight.read(imRight);

        if(imLeft.empty() || imRight.empty())
        {
            cerr << endl << "Failed to load image! " << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft, imRight, 0.0);

		Publish(SLAM, pub_pts_and_pose, pub_all_kf_and_pts, pub_cloud, pub_camera_pose, 0);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        // Wait to load the next frame
        cv::waitKey(1);
	}
    // Stop all threads
    SLAM.Shutdown();

    return 0;
}

void Publish(ORB_SLAM3::System &SLAM, ros::Publisher &pub_pts_and_pose,
	ros::Publisher &pub_all_kf_and_pts, ros::Publisher &pub_cloud, 
	ros::Publisher &pub_camera_pose, int frame_id) 
{
	// if (all_pts_pub_gap > 0 && pub_count >= all_pts_pub_gap) {
	// 	pub_all_pts = true;
	// 	pub_count = 0;
	// }

	if (pub_all_pts || SLAM.getLoopClosing()->loop_detected || SLAM.getTracker()->loop_detected) {

		pub_all_pts = SLAM.getTracker()->loop_detected = SLAM.getLoopClosing()->loop_detected = false;
		geometry_msgs::PoseArray kf_pt_array;
		vector<ORB_SLAM3::KeyFrame*> key_frames = SLAM.getMap()->GetAllKeyFrames();
		//! placeholder for number of keyframes
		kf_pt_array.poses.push_back(geometry_msgs::Pose());
		sort(key_frames.begin(), key_frames.end(), ORB_SLAM3::KeyFrame::lId);
		unsigned int n_kf = 0;
		for (auto key_frame : key_frames) {
			// pKF->SetPose(pKF->GetPose()*Two);

			if (key_frame->isBad())
				continue;

			cv::Mat R = ORB_SLAM3::Converter::toCvMat(key_frame->GetRotation()).t();
			vector<float> q = ORB_SLAM3::Converter::toQuaternion(R);
			cv::Mat twc = ORB_SLAM3::Converter::toCvMat(key_frame->GetCameraCenter());
			geometry_msgs::Pose kf_pose;

			kf_pose.position.x = twc.at<float>(0);
			kf_pose.position.y = twc.at<float>(1);
			kf_pose.position.z = twc.at<float>(2);
			kf_pose.orientation.x = q[0];
			kf_pose.orientation.y = q[1];
			kf_pose.orientation.z = q[2];
			kf_pose.orientation.w = q[3];
			kf_pt_array.poses.push_back(kf_pose);

			unsigned int n_pts_id = kf_pt_array.poses.size();
			//! placeholder for number of points
			kf_pt_array.poses.push_back(geometry_msgs::Pose());
			std::set<ORB_SLAM3::MapPoint*> map_points = key_frame->GetMapPoints();
			unsigned int n_pts = 0;
			for (auto map_pt : map_points) {
				if (!map_pt || map_pt->isBad()) {
					//printf("Point %d is bad\n", pt_id);
					continue;
				}
				cv::Mat pt_pose = ORB_SLAM3::Converter::toCvMat(map_pt->GetWorldPos());
				if (pt_pose.empty()) {
					//printf("World position for point %d is empty\n", pt_id);
					continue;
				}
				geometry_msgs::Pose curr_pt;
				//printf("wp size: %d, %d\n", wp.rows, wp.cols);
				// pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
				curr_pt.position.x = pt_pose.at<float>(0);
				curr_pt.position.y = pt_pose.at<float>(1);
				curr_pt.position.z = pt_pose.at<float>(2);
				kf_pt_array.poses.push_back(curr_pt);
				++n_pts;
			}
			geometry_msgs::Pose n_pts_msg;
			n_pts_msg.position.x = n_pts_msg.position.y = n_pts_msg.position.z = n_pts;
			kf_pt_array.poses[n_pts_id] = n_pts_msg;
			++n_kf;
		}
		geometry_msgs::Pose n_kf_msg;
		n_kf_msg.position.x = n_kf_msg.position.y = n_kf_msg.position.z = n_kf;
		kf_pt_array.poses[0] = n_kf_msg;
		kf_pt_array.header.frame_id = "1";
		kf_pt_array.header.seq = frame_id + 1;
		printf("Publishing data for %u keyfranmes\n", n_kf);
		pub_all_kf_and_pts.publish(kf_pt_array);
	}
	else if (SLAM.getTracker()->mCurrentFrame.is_keyframe) {
		++pub_count;
		SLAM.getTracker()->mCurrentFrame.is_keyframe = false;
		ORB_SLAM3::KeyFrame* pKF = SLAM.getTracker()->mCurrentFrame.mpReferenceKF;

		cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

		vector<ORB_SLAM3::KeyFrame*> vpKFs = SLAM.getMap()->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = ORB_SLAM3::Converter::toCvMat(vpKFs[0]->GetPoseInverse());

		Trw = Trw * ORB_SLAM3::Converter::toCvMat(pKF->GetPose()) * Two;
		cv::Mat lit = ORB_SLAM3::Converter::toCvMat(SLAM.getTracker()->mlRelativeFramePoses.back());
		cv::Mat Tcw = lit*Trw;
		cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
		cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);

		vector<float> q = ORB_SLAM3::Converter::toQuaternion(Rwc);
		//geometry_msgs::Pose camera_pose;
		//std::vector<ORB_SLAM2::MapPoint*> map_points = SLAM.getMap()->GetAllMapPoints();
		std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
		int n_map_pts = map_points.size();

		//printf("n_map_pts: %d\n", n_map_pts);

		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		geometry_msgs::PoseArray pt_array;
		//pt_array.poses.resize(n_map_pts + 1);

		geometry_msgs::Pose camera_pose;

		camera_pose.position.x = twc.at<float>(0);
		camera_pose.position.y = twc.at<float>(1);
		camera_pose.position.z = twc.at<float>(2);

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];

		pt_array.poses.push_back(camera_pose);
		//printf("Done getting camera pose\n");

		for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){
			
			// point is bad
			if (!map_points[pt_id - 1] || map_points[pt_id - 1]->isBad()) {
				continue;
			}
			cv::Mat wp = ORB_SLAM3::Converter::toCvMat(map_points[pt_id - 1]->GetWorldPos());

			if (wp.empty()) {
				continue;
			}
			geometry_msgs::Pose curr_pt;
			pcl_cloud->push_back(pcl::PointXYZ(wp.at<float>(0), wp.at<float>(1), wp.at<float>(2)));
			curr_pt.position.x = wp.at<float>(0);
			curr_pt.position.y = wp.at<float>(1);
			curr_pt.position.z = wp.at<float>(2);
			pt_array.poses.push_back(curr_pt);
			//printf("Done getting map point %d\n", pt_id);
		}

		sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*pcl_cloud, ros_cloud);
		ros_cloud.header.frame_id = "1";
		ros_cloud.header.seq = frame_id + 1;

		pub_cloud.publish(ros_cloud);
		pt_array.header.frame_id = "1";
		pt_array.header.seq = frame_id + 1;
		pub_pts_and_pose.publish(pt_array);
		pub_camera_pose.publish(camera_pose);
	}
}
