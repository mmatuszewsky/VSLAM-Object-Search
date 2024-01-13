#include <ros/ros.h>
#include <System.h>
#include <SLAMRunner.h>

#include <string>
#include <thread>

// ./main path_to_vocabulary path_to_camera_config path_to_map_config path_to_videos(optional)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "INzyniErKa");
	ros::start();
    std::cout << "Number of args: " << argc << std::endl;

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, false);
    SLAMRunner SlamRunner(argc, argv, SLAM);

    std::thread slamRunnerThread = std::thread(&SLAMRunner::run, &SlamRunner);

    MapBuilder mapBuilder = MapBuilder(std::string(argv[3]));

    if (slamRunnerThread.joinable()) {
        slamRunnerThread.join();
    }

    SLAM.Shutdown();
    return 0;
}
//  ./main /home/jetbot/VSLAM-Object-Search/Vocabulary/ORBvoc.txt /home/jetbot/VSLAM-Object-Search/Examples/Stereo/oak-d.yaml /home/jetbot/VSLAM-Object-Search/Examples/Stereo/map_config.json

// ./main /home/meciek/VSLAM-Object-Search/Vocabulary/ORBvoc.txt /home/meciek/VSLAM-Object-Search/Examples/Stereo/oak-d.yaml /home/meciek/VSLAM-Object-Search/Examples/Stereo/map_config.json

// ./main /home/meciek/VSLAM-Object-Search/Vocabulary/ORBvoc.txt /home/meciek/VSLAM-Object-Search/Examples/Stereo/oak-d.yaml /home/meciek/VSLAM-Object-Search/Examples/Stereo/map_config.json /home/meciek/Downloads/oakd
