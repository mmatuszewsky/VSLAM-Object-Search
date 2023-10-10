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

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

// ./stereo_video /home/meciek/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/meciek/ORB_SLAM3/Examples/Stereo/IMX219.yaml /home/meciek/slambook2/ch14/app/videos

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_video path_to_vocabulary path_to_settings path_to_videos" << endl;
        return 1;
    }

    cout << "Initializing video inputs from " << argv[3] << endl;

    // string leftFile = string(argv[3]) + "/left.mkv";
    // string rightFile = string(argv[3]) + "/right.mkv";

    // cout << leftFile << " | " << rightFile << endl;

    // cv::VideoCapture capLeft(leftFile, cv::IMREAD_UNCHANGED);
    // cv::VideoCapture capRight(rightFile, cv::IMREAD_UNCHANGED);

    cv::VideoCapture capLeft("/home/meciek/IMX219-83-Stereo-Demo-V2/18-load-videos/videos/left.mkv");
    cv::VideoCapture capRight("/home/meciek/IMX219-83-Stereo-Demo-V2/18-load-videos/videos/right.mkv");

    if (!capLeft.isOpened() || !capRight.isOpened()) {
        cerr << "Error: Could not open one or both videos." << endl;
        return -1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    cout << "Creating VSLAM System ..." << endl;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO,true);
    float imageScale = SLAM.GetImageScale();

    double t_track = 0.f;
    double t_resize = 0.f;

    // Main loop
    cout << endl << "-------" << endl;
    cout << "Start processing video ..." << endl;

    cv::Mat imLeft, imRight;

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
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC14
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = imLeft.cols * imageScale;
            int height = imLeft.rows * imageScale;
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC14
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft, imRight, 0.0);

#ifdef COMPILEDWITHC14
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        // Wait to load the next frame
        cv::waitKey(1);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}
