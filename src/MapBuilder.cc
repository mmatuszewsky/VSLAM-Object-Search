#include "MapBuilder.h"

#include <fstream>
#include <iostream>
#include <cv_bridge/cv_bridge.h>

#include <nlohmann/json.hpp>
#include "geometry_msgs/PoseArray.h"
using json = nlohmann::json;


MapBuilder::MapBuilder(std::string configFilePath) : 
    loop_closure_being_processed(false), configFilePath(configFilePath), n_kf_received(0)
{
    // ros::init(argc, argv, "Monosub");
	// ros::start();
    readConfig(configFilePath);

    grid_max_x = params.cloud_max_x * params.scale_factor;
	grid_min_x = params.cloud_min_x * params.scale_factor;
	grid_max_z = params.cloud_max_z * params.scale_factor;
	grid_min_z = params.cloud_min_z * params.scale_factor;
    // printf("grid_max: %f, %f\t grid_min: %f, %f\n", grid_max_x, grid_max_z, grid_min_x, grid_min_z);

    max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<double>::infinity();
	min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<double>::infinity();

    grid_res_x = grid_max_x - grid_min_x; 
	grid_res_z = grid_max_z - grid_min_z;

    h = grid_res_z;
	w = grid_res_x;
    // printf("grid_size: (%d, %d)\n", h, w);

    global_occupied_counter.create(h, w, CV_32SC1);
	global_visit_counter.create(h, w, CV_32SC1);
	global_occupied_counter.setTo(cv::Scalar(0));
	global_visit_counter.setTo(cv::Scalar(0));

    grid_map_msg.data.resize(h*w);
	grid_map_msg.info.width = w;
	grid_map_msg.info.height = h;
	grid_map_msg.info.resolution = 1.0 / params.resize_factor;
	grid_map_msg.header.frame_id = "map";

    grid_map_int = cv::Mat(h, w, CV_8SC1, (char*)(grid_map_msg.data.data()));

	grid_map.create(h, w, CV_32FC1);
	grid_map_thresh.create(h, w, CV_8UC1);
	//grid_map_thresh_resized.create(h*params.resize_factor, w*params.resize_factor, CV_8UC1);
	// printf("output_size: (%d, %d)\n", grid_map_thresh_resized.rows, grid_map_thresh_resized.cols);

	local_occupied_counter.create(h, w, CV_32SC1);
	local_visit_counter.create(h, w, CV_32SC1);
	local_map_pt_mask.create(h, w, CV_8UC1);

	norm_factor_x = float(grid_res_x - 1) / float(grid_max_x - grid_min_x);
	norm_factor_z = float(grid_res_z - 1) / float(grid_max_z - grid_min_z);
	// printf("norm_factor_x: %f\n", norm_factor_x);
	// printf("norm_factor_z: %f\n", norm_factor_z);

    sub_pts_and_pose = nh.subscribe("pts_and_pose", 1000, &MapBuilder::ptCallback, this);
	sub_all_kf_and_pts = nh.subscribe("all_kf_and_pts", 1000, &MapBuilder::loopClosingCallback, this);
    sub_detected_objs = nh.subscribe("coords", 10, &MapBuilder::detectionCallback, this);
	//sub_depth_map = nh.subscribe("camera_depth/raw", 10);
	pub_camera_pose = nh.advertise<visualization_msgs::Marker>("camera_pose", 100);
	pub_grid_map = nh.advertise<nav_msgs::OccupancyGrid>("map", 1000);

    id = 1;
	q90.setRPY(0.0, -M_PI / 2.0, 0.0);

    ros::spin();
    ros::shutdown();
}

void MapBuilder::readConfig(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    json j;
    file >> j;

    params.scale_factor = j["scale_factor"];
    params.resize_factor = j["resize_factor"];
    params.cloud_max_x = j["cloud_max_x"];
    params.cloud_min_x = j["cloud_min_x"];
    params.cloud_max_z = j["cloud_max_z"];
    params.cloud_min_z = j["cloud_min_z"];
    params.free_thresh = j["free_thresh"];
    params.occupied_thresh = j["occupied_thresh"];
    params.thresh_diff = j["thresh_diff"];
    params.visit_thresh = j["visit_thresh"];
    params.upper_left_x = j["upper_left_x"];
    params.upper_left_y = j["upper_left_y"];
    params.resolution = j["resolution"];
    params.use_local_counters = j["use_local_counters"];

	params.radius = j["radius_size"];

    printParams();
}

void MapBuilder::printParams()
{
    std::cout << "scale_factor: " << params.scale_factor << std::endl;
    std::cout << "resize_factor: " << params.resize_factor << std::endl;
    std::cout << "cloud_max_x: " << params.cloud_max_x << std::endl;
    std::cout << "cloud_min_x: " << params.cloud_min_x << std::endl;
    std::cout << "cloud_max_z: " << params.cloud_max_z << std::endl;
    std::cout << "cloud_min_z: " << params.cloud_min_z << std::endl;
    std::cout << "free_thresh: " << params.free_thresh << std::endl;
    std::cout << "occupied_thresh: " << params.occupied_thresh << std::endl;
    std::cout << "thresh_diff: " << params.thresh_diff << std::endl;
    std::cout << "visit_thresh: " << params.visit_thresh << std::endl;
    std::cout << "upper_left_x: " << params.upper_left_x << std::endl;
    std::cout << "upper_left_y: " << params.upper_left_y << std::endl;
    std::cout << "resolution: " << params.resolution << std::endl;
    std::cout << "use_local_counters: " << std::boolalpha << params.use_local_counters << std::endl;
}

void MapBuilder::updateGridMap(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose)
{
	// first pose is camera_pose
	const geometry_msgs::Point &kf_location = pts_and_pose->poses[0].position;
	const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;

	kf_pos_x = kf_location.x*params.scale_factor;
	kf_pos_z = kf_location.z*params.scale_factor;

	kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
	kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

	if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
		return;

	if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
		return;

	++n_kf_received;
	unsigned int n_pts = pts_and_pose->poses.size() - 1;

	processMapPts(pts_and_pose->poses, n_pts, 1, kf_pos_grid_x, kf_pos_grid_z);

	pubCameraPose(kf_location, kf_orientation);

	getGridMap();
}

void MapBuilder::resetGridMap(const geometry_msgs::PoseArray::ConstPtr& all_kf_and_pts)
{
    global_visit_counter.setTo(0);
	global_occupied_counter.setTo(0);

	unsigned int n_kf = all_kf_and_pts->poses[0].position.x;
	if ((unsigned int) (all_kf_and_pts->poses[0].position.y) != n_kf ||
		(unsigned int) (all_kf_and_pts->poses[0].position.z) != n_kf) {
		printf("resetGridMap :: Unexpected formatting in the keyframe count element\n");
		return;
	}
	printf("Resetting grid map with %d key frames\n", n_kf);

	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

	unsigned int id = 0;
	for (unsigned int kf_id = 0; kf_id < n_kf; ++kf_id){
		const geometry_msgs::Point &kf_location = all_kf_and_pts->poses[++id].position;
		//const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;
		unsigned int n_pts = all_kf_and_pts->poses[++id].position.x;
		if ((unsigned int)(all_kf_and_pts->poses[id].position.y) != n_pts ||
			(unsigned int)(all_kf_and_pts->poses[id].position.z) != n_pts) {
			printf("resetGridMap :: Unexpected formatting in the point count element for keyframe %d\n", kf_id);
			return;
		}
		float kf_pos_x = kf_location.x*params.scale_factor;
		float kf_pos_z = kf_location.z*params.scale_factor;

		int kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
		int kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

		if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
			continue;

		if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
			continue;

		if (id + n_pts >= all_kf_and_pts->poses.size()) {
			printf("resetGridMap :: Unexpected end of the input array while processing keyframe %u with %u points: only %u out of %u elements found\n",
				kf_id, n_pts, all_kf_and_pts->poses.size(), id + n_pts);
			return;
		}
		processMapPts(all_kf_and_pts->poses, n_pts, id + 1, kf_pos_grid_x, kf_pos_grid_z);
		id += n_pts;
	}
	getGridMap();

	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

	double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
	printf("Done. Time taken: %f secs\n", ttrack);
	pub_grid_map.publish(grid_map_msg);
}

void MapBuilder::ptCallback(const geometry_msgs::PoseArray::ConstPtr& pts_and_pose)
{
    if (loop_closure_being_processed){ return; }
	updateGridMap(pts_and_pose);
	grid_map_msg.info.map_load_time = ros::Time::now();
	pub_grid_map.publish(grid_map_msg);
}

void MapBuilder::loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr& all_kf_and_pts)
{
    loop_closure_being_processed = true;
	resetGridMap(all_kf_and_pts);
	loop_closure_being_processed = false;
}

void MapBuilder::processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied,
    cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z)
{
    float pt_pos_x = curr_pt.x*params.scale_factor;
	float pt_pos_z = curr_pt.z*params.scale_factor;

	int pt_pos_grid_x = int(floor((pt_pos_x - grid_min_x) * norm_factor_x));
	int pt_pos_grid_z = int(floor((pt_pos_z - grid_min_z) * norm_factor_z));


	if (pt_pos_grid_x < 0 || pt_pos_grid_x >= w)
		return;

	if (pt_pos_grid_z < 0 || pt_pos_grid_z >= h)
		return;

	// Increment the occupency account of the grid cell where map point is located
	++occupied.at<int>(pt_pos_grid_z, pt_pos_grid_x);
	pt_mask.at<uchar>(pt_pos_grid_z, pt_pos_grid_x) = 255;

	//cout << "----------------------" << endl;
	//cout << okf_pos_grid_x << " " << okf_pos_grid_y << endl;

	// Get all grid cell that the line between keyframe and map point pass through
	int x0 = kf_pos_grid_x;
	int y0 = kf_pos_grid_z;
	int x1 = pt_pos_grid_x;
	int y1 = pt_pos_grid_z;
	bool steep = (abs(y1 - y0) > abs(x1 - x0));
	if (steep){
		std::swap(x0, y0);
		std::swap(x1, y1);
	}
	if (x0 > x1){
		std::swap(x0, x1);
		std::swap(y0, y1);
	}
	int dx = x1 - x0;
	int dy = abs(y1 - y0);
	double error = 0;
	double deltaerr = ((double)dy) / ((double)dx);
	int y = y0;
	int ystep = (y0 < y1) ? 1 : -1;
	for (int x = x0; x <= x1; ++x){
		if (steep) {
			++visited.at<int>(x, y);
		}
		else {
			++visited.at<int>(y, x);
		}
		error = error + deltaerr;
		if (error >= 0.5){
			y = y + ystep;
			error = error - 1.0;
		}
	}
}

void MapBuilder::processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
    unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z)
{
    unsigned int end_id = start_id + n_pts;
	if (params.use_local_counters) {
		local_map_pt_mask.setTo(0);
		local_occupied_counter.setTo(0);
		local_visit_counter.setTo(0);
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
			processMapPt(pts[pt_id].position, local_occupied_counter, local_visit_counter,
				local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
		}
		for (int row = 0; row < h; ++row){
			for (int col = 0; col < w; ++col){
				if (local_map_pt_mask.at<uchar>(row, col) == 0) {
					local_occupied_counter.at<int>(row, col) = 0;
				}
				else {
					local_occupied_counter.at<int>(row, col) = local_visit_counter.at<int>(row, col);
				}
			}
		}
		global_occupied_counter += local_occupied_counter;
		global_visit_counter += local_visit_counter;
	}
	else {
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id){
			processMapPt(pts[pt_id].position, global_occupied_counter, global_visit_counter,
				local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
		}
	}
}

void MapBuilder::getGridMap()
{
    for (int row = 0; row < h; ++row){
		for (int col = 0; col < w; ++col){
			int visits = global_visit_counter.at<int>(row, col);
			int occupieds = global_occupied_counter.at<int>(row, col);

			if (visits <= params.visit_thresh){
				grid_map.at<float>(row, col) = 0.5;
			}
			else {
				grid_map.at<float>(row, col) = 1.0 - float(occupieds / visits);
			}
			if (grid_map.at<float>(row, col) >= params.free_thresh) {
				grid_map_thresh.at<uchar>(row, col) = 255;
			}
			else if (grid_map.at<float>(row, col) < params.free_thresh && 
                grid_map.at<float>(row, col) >= params.occupied_thresh) {
				grid_map_thresh.at<uchar>(row, col) = 128;
			}
			else {
				grid_map_thresh.at<uchar>(row, col) = 0;
			}
			grid_map_int.at<char>(row, col) = (1 - grid_map.at<float>(row, col)) * 100;
		}
	}
	// cv::resize(grid_map_thresh, grid_map_thresh_resized, grid_map_thresh_resized.size());
}

void MapBuilder::detectionCallback(const visualization_msgs::MarkerArray::ConstPtr& objs)
{
	visualization_msgs::Marker tmp_camera_pose = camera_pose;
	
	tf::Vector3 position(tmp_camera_pose.pose.position.x, 
		tmp_camera_pose.pose.position.y, tmp_camera_pose.pose.position.z);
	
	tf::Quaternion orientation(tmp_camera_pose.pose.orientation.x, tmp_camera_pose.pose.orientation.y, 
		tmp_camera_pose.pose.orientation.z, tmp_camera_pose.pose.orientation.w);
	// sensor_msgs::Image::ConstPtr depth_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/depth_raw", nh);
    // cv_bridge::CvImagePtr cv_ptr = 
    //     cv_bridge::toCvCopy(depth_msg , sensor_msgs::image_encodings::TYPE_16UC1);
    // depth_img = cv_ptr->image;
	// 1. for every detected object in current frame
	// 2. check its corresponding depth
	// 3. calculate its position
	// 4. isAnyObjectInRadius(obj, detected_objects)
	// 5. if not publish new object
    for (auto marker : objs->markers)
    {
		// int x = marker.pose.position.x;
		// int y = marker.pose.position.y;
		// float depth = depth_img.at<float>(x, y);
		
        std::cout << marker.header.frame_id << std::endl << "X: " << marker.pose.position.x << " Y: " << marker.pose.position.z << "Z " << depth << std::endl;

		/* 
		W zależności jak to końcowo jest zaimplementowane
		w tym miejscu będzie albo sprawdzana odległość przedmiotu mając pozycje pikseli x i y
		lub po prostu brana składowa z (jeśli jest to robione po stronie skryptu pythonowego).

		Ja tutaj zakładam, że każdy obiekt jest oddalony o 100 jednostek i tyle.
		*/

		// TUTAJ W OFFSECIE DAJ WARTOŚĆ Z DEPTH MAPY
		// Dzielone przez 10 bo konwersja mm -> cm
		tf::Vector3 offset(marker.pose.position.z/10, 0.0, 0.0);
		tf::Vector3 offset_global = tf::quatRotate(orientation, offset);
		tf::Vector3 new_position = position + offset_global;

		DetectedObject obj = {
			new_position[0], 
			new_position[1], 
			marker.ns
		};

		// check if any objects lays in specified radius
		if(!isAnyObjectInRadius(obj, object_locations))
		{
			object_locations.push_back(obj);

			visualization_msgs::Marker new_pose;
			new_pose.header.stamp = ros::Time::now();
			new_pose.header.frame_id = "map";
			new_pose.type = visualization_msgs::Marker::CUBE;
			new_pose.ns = obj.label;
			new_pose.id = id++;
			new_pose.scale.x = 10;
			new_pose.scale.y = 10;
			new_pose.scale.z = 10;

			new_pose.color.r = static_cast<float>(std::rand()) / RAND_MAX;
			new_pose.color.g = static_cast<float>(std::rand()) / RAND_MAX;
			new_pose.color.b = static_cast<float>(std::rand()) / RAND_MAX;
			new_pose.color.a = 1.0;

			new_pose.pose.position.x = new_position.getX();
			new_pose.pose.position.y = new_position.getY();
			new_pose.pose.position.z = 0;
			new_pose.pose.orientation = camera_pose.pose.orientation;

			pub_camera_pose.publish(new_pose);
		}
    }
}

void MapBuilder::pubCameraPose(const geometry_msgs::Point &kf_location, 
	const geometry_msgs::Quaternion &kf_orientation) 
{
	camera_pose.header.stamp = ros::Time::now();
	camera_pose.header.frame_id = "map";

	camera_pose.ns = "my_namespace";
	camera_pose.id = 0;

	camera_pose.pose.position.x = kf_pos_grid_x / params.scale_factor;
	camera_pose.pose.position.y = kf_pos_grid_z / params.scale_factor;
	camera_pose.pose.position.z = 0;

	// rotate by 90 degrees because rviz ...
	tf::Quaternion tf_orientation;
	tf::quaternionMsgToTF(kf_orientation, tf_orientation);
	tf::Quaternion corrected_orientation = tf_orientation * q90;
	geometry_msgs::Quaternion corrected_msg_orientation;
	tf::quaternionTFToMsg(corrected_orientation, corrected_msg_orientation);

	// replace y with z and vice versa because rviz ...
	camera_pose.pose.orientation.x = corrected_msg_orientation.x;
	camera_pose.pose.orientation.y = -corrected_msg_orientation.z;
	camera_pose.pose.orientation.z = -corrected_msg_orientation.y;
	camera_pose.pose.orientation.w = corrected_msg_orientation.w;

	camera_pose.type = visualization_msgs::Marker::ARROW;

	camera_pose.scale.x = 12;
	camera_pose.scale.y = 6;
	camera_pose.scale.z = 6;

	camera_pose.color.r = 0.0;
	camera_pose.color.g = 1.0;
	camera_pose.color.b = 0.0;
	camera_pose.color.a = 1.0;

	pub_camera_pose.publish(camera_pose);
}

bool MapBuilder::isAnyObjectInRadius(const DetectedObject& currentPos, 
	const std::vector<DetectedObject>& objectPositions) 
{
    for (const DetectedObject& objectPos : objectPositions) {
        // MOŻNA PRZYSPIESZYĆ NIE LICZĄC PIERWIASTKA
        double distance = std::sqrt(
			std::pow(objectPos.x - currentPos.x, 2) + std::pow(objectPos.y - currentPos.y, 2)
		);

        if (distance <= params.radius && objectPos.label == currentPos.label) {
            return true;
        }
    }

    return false;
}

// void MapBuilder::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
// {
//     cv_bridge::CvImagePtr cv_ptr = 
//         cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
//     depth_img = cv_ptr->image;
// }
