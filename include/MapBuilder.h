#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <vector>
#include <string>

#include <tf/transform_datatypes.h>

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <sensor_msgs/Image.h>

struct DetectedObject {
	double x;
	double y;
	std::string label;
};

struct Parameters {
    float scale_factor;
    float resize_factor;
    float cloud_max_x;
    float cloud_min_x;
    float cloud_max_z;
    float cloud_min_z;
    float free_thresh;
    float occupied_thresh;
    float thresh_diff;
    int visit_thresh;
    float upper_left_x;
    float upper_left_y;
    int resolution;
    bool use_local_counters;
    double radius;
};

class MapBuilder
{
public:
    MapBuilder(std::string configFilePath);

    void run();
    void readConfig(const std::string& filename);
    void printParams();
    void updateGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);
    void resetGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);
    void ptCallback(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose);
    void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr& all_kf_and_pts);
    void showGridMap(unsigned int id = 0);
    void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied,
        cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z);
    void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
        unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z);
    void getGridMap();
    void pubCameraPose(const geometry_msgs::Point &kf_location, 
        const geometry_msgs::Quaternion &kf_orientation);

    bool isAnyObjectInRadius(const DetectedObject& currentPos, 
	    const std::vector<DetectedObject>& objectPositions); 
    void detectionCallback(const visualization_msgs::MarkerArray::ConstPtr& pts_and_pose);
    void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);


private:

ros::NodeHandle nh;

std::string configFilePath;

// verified objects
std::vector<DetectedObject> object_locations;

// objects to be verified
std::vector<DetectedObject> tmp_object_locations;

Parameters params;

float grid_max_x, grid_min_x,grid_max_z, grid_min_z;
float norm_factor_x, norm_factor_z;
float kf_pos_x, kf_pos_z;
unsigned int n_kf_received;
int kf_pos_grid_x, kf_pos_grid_z;
int h, w;
int id;
bool loop_closure_being_processed;

double grid_res_x; 
double grid_res_z;

cv::Mat global_occupied_counter, global_visit_counter;
cv::Mat local_occupied_counter, local_visit_counter;
cv::Mat local_map_pt_mask;
cv::Mat grid_map, grid_map_int, grid_map_thresh, grid_map_thresh_resized;
cv::Mat depth_img;

ros::Publisher pub_grid_map;
ros::Publisher pub_camera_pose;

ros::Subscriber sub_pts_and_pose;
ros::Subscriber sub_all_kf_and_pts;
ros::Subscriber sub_detected_objs;
ros::Subscriber sub_depth_map;

nav_msgs::OccupancyGrid grid_map_msg;
visualization_msgs::Marker camera_pose;

geometry_msgs::Point min_pt, max_pt;
tf::Quaternion q90;

};

#endif
