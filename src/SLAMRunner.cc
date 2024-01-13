#include "SLAMRunner.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/PoseArray.h"


SLAMRunner::SLAMRunner(int argc, char **argv, ORB_SLAM3::System &SLAM) 
    : SLAM(SLAM), is_video(false), read_from_topic(false), pub_all_pts(false), 
    pub_count(0), all_pts_pub_gap(0), frame_id(0)
{
    // Video as a source
    if (argc == 5)
    {
        leftFileName = string(argv[4]) + "/left.mp4";
        rightFileName = string(argv[4]) + "/right.mp4";
        capLeft = cv::VideoCapture(leftFileName);
        capRight = cv::VideoCapture(rightFileName);

        if (!capLeft.isOpened() || !capRight.isOpened()) {
            std::cerr << "Error: Couldn't open videos." << std::endl;
            return;
        }

        is_video = true;
    }
    // Topics as a source
    else if (argc == 4)
    {
        sub_left = nh.subscribe("/camera/left", 3, &SLAMRunner::leftImageCallback, this);
        sub_right = nh.subscribe("/camera/right", 3, &SLAMRunner::rightImageCallback, this);
    }
    else 
    {
        std::cerr << "\nInvalid number of args!" << std::endl;
        std::cerr << "\nUsage: ./stereo_video path_to_vocabulary path_to_settings path_to_videos" << std::endl;
        return;
    }

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("cloud_in", 1000);
	pub_pts_and_pose = nh.advertise<geometry_msgs::PoseArray>("pts_and_pose", 1000);
	pub_all_kf_and_pts = nh.advertise<geometry_msgs::PoseArray>("all_kf_and_pts", 1000);
}

SLAMRunner::~SLAMRunner() { SLAM.Shutdown(); }

void SLAMRunner::run()
{
    std::cout << std::endl << "vSLAM is running!" << std::endl;
    float imageScale = SLAM.GetImageScale();

    if (is_video)
    {
        while (capLeft.isOpened() && capRight.isOpened()) {
            capLeft.read(left_img);
            capRight.read(right_img);

            if (left_img.empty() || right_img.empty())
            {
                cerr << endl << "Failed to load image! " << endl;
                break;
            }

            if (imageScale != 1.f)
            {
                int width = left_img.cols * imageScale;
                int height = left_img.rows * imageScale;
                cv::resize(left_img, left_img, cv::Size(width, height));
                cv::resize(right_img, right_img, cv::Size(width, height));
            }

            SLAM.TrackStereo(left_img, right_img, 0.0);
            publish();
            frame_id++;

            int key = cv::waitKey(1);
            if (key == 'q') {
                break;
            }
        }
    }
    else
    {
        ros::Rate rate(30);
        while (ros::ok()) {

            if (left_img.empty() || right_img.empty())
            {
                cerr << endl << "Failed to load image! " << endl;
                continue;
            }

            if (imageScale != 1.f)
            {
                int width = left_img.cols * imageScale;
                int height = left_img.rows * imageScale;
                cv::resize(left_img, left_img, cv::Size(width, height));
                cv::resize(right_img, right_img, cv::Size(width, height));
            }

            SLAM.TrackStereo(left_img, right_img, 0.0);
            publish();
            frame_id++;

            ros::spinOnce();
            rate.sleep();
            int key = cv::waitKey(1);
            if (key == 'q') {
                break;
            }
        }
    }

    SLAM.Shutdown();
    return;
}

void SLAMRunner::publish()
{

	if (pub_all_pts || SLAM.getLoopClosing()->loop_detected || SLAM.getTracker()->loop_detected) {

		pub_all_pts = SLAM.getTracker()->loop_detected = SLAM.getLoopClosing()->loop_detected = false;
		geometry_msgs::PoseArray kf_pt_array;
		vector<ORB_SLAM3::KeyFrame*> key_frames = SLAM.getMap()->GetAllKeyFrames();

		//! placeholder for number of keyframes
		kf_pt_array.poses.push_back(geometry_msgs::Pose());
		sort(key_frames.begin(), key_frames.end(), ORB_SLAM3::KeyFrame::lId);
		unsigned int n_kf = 0;
		for (auto key_frame : key_frames) {
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
					continue;
				}
				cv::Mat pt_pose = ORB_SLAM3::Converter::toCvMat(map_pt->GetWorldPos());
				if (pt_pose.empty()) {
					continue;
				}
				geometry_msgs::Pose curr_pt;

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
		kf_pt_array.header.frame_id = "map";
		kf_pt_array.header.seq = frame_id + 1;
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

		std::vector<ORB_SLAM3::MapPoint*> map_points = SLAM.GetTrackedMapPoints();
		int n_map_pts = map_points.size();

		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		geometry_msgs::PoseArray pt_array;
		geometry_msgs::Pose camera_pose;

		camera_pose.position.x = twc.at<float>(0,0);
		camera_pose.position.y = twc.at<float>(0,1);
		camera_pose.position.z = twc.at<float>(0,2);

		camera_pose.orientation.x = q[0];
		camera_pose.orientation.y = q[1];
		camera_pose.orientation.z = q[2];
		camera_pose.orientation.w = q[3];

		pt_array.poses.push_back(camera_pose);

		for (int pt_id = 1; pt_id <= n_map_pts; ++pt_id){
			
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

            // printf("%d %d\n", curr_pt.position.x, curr_pt.position.z);
		}

		sensor_msgs::PointCloud2 ros_cloud;
		pcl::toROSMsg(*pcl_cloud, ros_cloud);
		ros_cloud.header.frame_id = "map";
		ros_cloud.header.seq = frame_id + 1;
		pub_cloud.publish(ros_cloud);

		pt_array.header.frame_id = "map";
		pt_array.header.seq = frame_id + 1;
		pub_pts_and_pose.publish(pt_array);

        // printf("%d %d %d\n", camera_pose.position.x, camera_pose.position.y, camera_pose.position.z);
    }
}

void SLAMRunner::leftImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = 
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    left_img = cv_ptr->image;
}

void SLAMRunner::rightImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr = 
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    right_img = cv_ptr->image;
}
