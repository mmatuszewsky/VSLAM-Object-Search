#ifndef SLAM_RUNNER_H
#define SLAM_RUNNER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <thread>

#include <System.h>
#include "MapBuilder.h"


class SLAMRunner
{
public:
    SLAMRunner(int argc, char **argv, ORB_SLAM3::System &SLAM);
    ~SLAMRunner();

    void run();
    void publish();
    void leftImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void rightImageCallback(const sensor_msgs::Image::ConstPtr& msg);

private:
    ORB_SLAM3::System& SLAM;
    // MapBuilder mapBuilder;
    // std::thread mapBuilderThread_;

    ros::NodeHandle nh;
    ros::Publisher pub_pts_and_pose;
    ros::Publisher pub_all_kf_and_pts;
    ros::Publisher pub_cloud;

    ros::Subscriber sub_left;
    ros::Subscriber sub_right;

    cv::Mat left_img;
    cv::Mat right_img;

    std::string leftFileName;
    std::string rightFileName;

    cv::VideoCapture capLeft;
    cv::VideoCapture capRight;

    bool is_video;
    bool read_from_topic; 
    bool pub_all_pts;
    int pub_count;
    int all_pts_pub_gap;
    int frame_id;
};

#endif
