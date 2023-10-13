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


#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Settings.h"

#include <mutex>

namespace ORB_SLAM3
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Settings;

class GridMap
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GridMap(System* pSystem, Tracking *pTracking, const string &strSettingPath, Settings* settings);

    void newParameterLoader(Settings* settings);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    bool isStepByStep();

    void Release();

    //void SetTrackingPause();

    bool both;

private:
    // bool ParseGridMapParamFile(cv::FileStorage &fSettings);

    bool Stop();

    System* mpSystem;
    // FrameDrawer* mpFrameDrawer;
    // MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;
    float mImageGridMapScale;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

    bool mbStopTrack;

    // parameters
    float scale_factor = 3;
    float resize_factor = 5;
    float cloud_max_x = 10;
    float cloud_min_x = -10.0;
    float cloud_max_z = 16;
    float cloud_min_z = -5;
    float free_thresh = 0.55;
    float occupied_thresh = 0.50;
    float thresh_diff = 0.01;
    int visit_thresh = 0;
    float upper_left_x = -1.5;
    float upper_left_y = -2.5;
    const int resolution = 10;
    unsigned int use_local_counters = 0;

    float grid_max_x, grid_min_x,grid_max_z, grid_min_z;
    cv::Mat global_occupied_counter, global_visit_counter;
    cv::Mat local_occupied_counter, local_visit_counter;
    cv::Mat local_map_pt_mask;
    cv::Mat grid_map, grid_map_int, grid_map_thresh, grid_map_thresh_resized;
    float norm_factor_x, norm_factor_z;
    int h, w;
    unsigned int n_kf_received;
    bool loop_closure_being_processed = false;

    // ros::Publisher pub_grid_map;
    // nav_msgs::OccupancyGrid grid_map_msg;

    float kf_pos_x, kf_pos_z;
    int kf_pos_grid_x, kf_pos_grid_z;


    void UpdateGridMap(const std::vector<Eigen::Isometry3d>& pts_and_pose);

    void ResetGridMap(const std::vector<Eigen::Isometry3d>& pts_and_pose);

    // void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pt_cloud);
    // void KfCallback(const geometry_msgs::PoseStamped::ConstPtr& camera_pose);

    void SaveMap(unsigned int id = 0);

    void PtCallback(const std::vector<Eigen::Isometry3d>& pts_and_pose);

    void LoopClosingCallback(const std::vector<Eigen::Isometry3d>& all_kf_and_pts);

    void ParseParams(int argc, char **argv);

    void PrintParams();

    void ShowGridMap(unsigned int id = 0);

    void GetMixMax(const std::vector<Eigen::Isometry3d>& pts_and_pose,
        Eigen::Isometry3d& min_pt, Eigen::Isometry3d& max_pt);

    void ProcessMapPt(const Eigen::Isometry3d &curr_pt, cv::Mat &occupied,
        cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z);

    void ProcessMapPts(const std::vector<Eigen::Isometry3d> &pts, unsigned int n_pts,
        unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z);

    void GetGridMap();

// --------------

    std::vector<Eigen::Isometry3d> GetPoses();

};

}


#endif // GridMap_H
	

