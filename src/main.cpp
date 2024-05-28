#include <nlohmann/json.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>

// Based on: https://learn.microsoft.com/en-us/azure/kinect-dk/body-joints
static constexpr std::size_t left_shoulder_idx = 5;
static constexpr std::size_t right_shoulder_idx = 12;
static constexpr std::size_t left_hip_idx = 18;
static constexpr std::size_t right_hip_idx = 22;

auto get_joints(const std::string& file_name) -> std::vector<pcl::PointXYZ>
{
	auto joints = std::vector<pcl::PointXYZ>();
	auto json_fs = std::ifstream(file_name);
	if (not json_fs.is_open()) {
		throw std::runtime_error("Failed to open JSON file\n");
	}

	auto joints_json = nlohmann::json::parse(json_fs);
	if (joints_json.is_discarded()) {
		throw std::runtime_error("Failed to parse JSON file\n");
	}

	for (const auto& frame : joints_json["frames"]) {
		for (const auto& body : frame["bodies"]) {
			for (const auto& joint : body["joints"]) {
				const auto& joint_arr = joint.at(0);
				const auto joint_x = joint_arr["x"].get<double>();
				const auto joint_y = joint_arr["y"].get<double>();
				const auto joint_z = joint_arr["z"].get<double>();

				joints.emplace_back(joint_x, joint_y, joint_z);
			}
		}
	}

	return joints;
}

static auto get_viewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) -> pcl::visualization::PCLVisualizer::Ptr
{
	auto viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Metamorphosis task"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	return viewer;
}

static auto enrich_viewer_with_intersections(
    pcl::visualization::PCLVisualizer::Ptr viewer,
    const std::vector<pcl::PointXYZ>& joints,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	auto left_shoulder = joints[left_shoulder_idx];
	auto right_shoulder = joints[right_shoulder_idx];
	auto left_hip = joints[left_hip_idx];
	auto right_hip = joints[right_hip_idx];

	auto shoulder_midpoint = pcl::PointXYZ(
	    (left_shoulder.x + right_shoulder.x) / 2,
	    (left_shoulder.y + right_shoulder.y) / 2,
	    (left_shoulder.z + right_shoulder.z) / 2);
	auto hip_midpoint = pcl::PointXYZ(
	    (left_hip.x + right_hip.x) / 2,
	    (left_hip.y + right_hip.y) / 2,
	    (left_hip.z + right_hip.z) / 2);
	auto shoulder_hip_midpoint = pcl::PointXYZ(
	    (hip_midpoint.x + shoulder_midpoint.x) / 2,
	    (hip_midpoint.y + shoulder_midpoint.y) / 2,
	    (hip_midpoint.z + shoulder_midpoint.z) / 2);

	viewer->addSphere(shoulder_midpoint, 0.01, 0, 0, 255, "shoulder_midpoint");
	viewer->addSphere(hip_midpoint, 0.01, 0, 0, 255, "hip_midpoint");
	viewer->addSphere(shoulder_hip_midpoint, 0.01, 0, 255, 0, "intersection_point");
	viewer->addLine<pcl::PointXYZ>(hip_midpoint, shoulder_midpoint, "midline");

	auto avg_y = (left_shoulder.y + right_shoulder.y + left_hip.y + right_hip.y) / 4.0;
	auto avg_x = (left_shoulder.x + right_shoulder.x + left_hip.x + right_hip.x) / 4.0;
	auto intersection_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	for (const auto& point : cloud->points) {
		if (std::abs(point.y - avg_y) < 0.01 && std::abs(point.x - avg_x) < 0.01) {
			intersection_cloud->points.push_back(point);
		}
	}

	auto min_z_point = *std::min_element(intersection_cloud->points.begin(), intersection_cloud->points.end(), [](const auto& a, const auto& b) {
		return a.z < b.z;
	});

	viewer->addPointCloud<pcl::PointXYZ>(intersection_cloud, "intersection_cloud");
	viewer->addSphere(min_z_point, 0.02, 255, 0, 0, "min_z_point");
}

auto main(const int argc, const char* argv[]) -> int
{
	if (argc < 3) {
		std::cerr << "Bad input!\n";
		std::cerr << "Usage: " << argv[0] << " <file_name>.ply <file_name>.json\n";
		return EXIT_FAILURE;
	}

	auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	auto reader = pcl::PLYReader();
	if (reader.read(argv[1], *cloud) < 0) {
		return EXIT_FAILURE;
	}

	// Get all joints
	auto joints = get_joints(argv[2]);
	// Visualise everything
	auto viewer = get_viewer(cloud);
	enrich_viewer_with_intersections(viewer, joints, cloud);

	while (!viewer->wasStopped()) {
		viewer->spin();
	}
	return EXIT_SUCCESS;
}
