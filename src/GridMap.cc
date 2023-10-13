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


#include "GridMap.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM3
{

GridMap::GridMap(System* pSystem, Tracking *pTracking, const string &strSettingPath, Settings* settings):
    both(false), mpSystem(pSystem), mpTracker(pTracking), mbFinishRequested(false), 
    mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    mbStopTrack = false;
}

void GridMap::Run()
{
    mbFinished = false;
    mbStopped = false;

    cv::namedWindow("ORB-SLAM3: 2D Grid Map");

    float trackedImageScale = mpTracker->GetImageScale();

	grid_max_x = cloud_max_x*scale_factor;
	grid_min_x = cloud_min_x*scale_factor;
	grid_max_z = cloud_max_z*scale_factor;
	grid_min_z = cloud_min_z*scale_factor;

	double grid_res_x = grid_max_x - grid_min_x; 
    double grid_res_z = grid_max_z - grid_min_z;

	h = grid_res_z;
	w = grid_res_x;

	n_kf_received = 0;

	global_occupied_counter.create(h, w, CV_32SC1);
	global_visit_counter.create(h, w, CV_32SC1);
	global_occupied_counter.setTo(cv::Scalar(0));
	global_visit_counter.setTo(cv::Scalar(0));

	// grid_map_msg.data.resize(h*w);
	// grid_map_msg.info.width = w;
	// grid_map_msg.info.height = h;
	// grid_map_msg.info.resolution = 1.0/scale_factor;

    // occupancy grid 0 - unoccupied, 1 - occupied, -1 - unknown
	grid_map_int = cv::Mat(h, w, CV_8SC1, (char*)(-1));

    grid_map.create(h, w, CV_32FC1);
	grid_map_thresh.create(h, w, CV_8UC1);
	grid_map_thresh_resized.create(h*resize_factor, w*resize_factor, CV_8UC1);

    local_occupied_counter.create(h, w, CV_32SC1);
	local_visit_counter.create(h, w, CV_32SC1);
	local_map_pt_mask.create(h, w, CV_8UC1);

    norm_factor_x = float(grid_res_x - 1) / float(grid_max_x - grid_min_x);
	norm_factor_z = float(grid_res_z - 1) / float(grid_max_z - grid_min_z);

    cout << "Starting the Grid Map" << endl;
    while(1)
    {
        cv::Mat toShow;
        // cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);

        cv::imshow("ORB-SLAM3: Current Frame", toShow);
        cv::waitKey(mT);

        if (Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

std::vector<Eigen::Isometry3d> GridMap::GetPoses()
{

}

void GridMap::UpdateGridMap(const std::vector<Eigen::Isometry3d>& pts_and_pose)
{
    const Eigen::Isometry3d &kf_location = pts_and_pose[0];
}


void GridMap::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool GridMap::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void GridMap::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool GridMap::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void GridMap::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool GridMap::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool GridMap::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void GridMap::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

/*void GridMap::SetTrackingPause()
{
    mbStopTrack = true;
}*/

}
