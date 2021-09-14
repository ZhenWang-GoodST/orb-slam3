#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <fstream>

#include <iostream> //标准输入输出流
// #include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
// #include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
// #include <pcl/visualization/cloud_viewer.h>

namespace tergeo {
namespace visualodometry {
void extract_features(
	const cv::Mat &image,
	cv::Mat& descriptors,
	std::vector<cv::Vec3b>& colors,
	std::vector<cv::KeyPoint>& key_points);


void match_features(cv::Mat& query, cv::Mat& train, std::vector<cv::DMatch>& matches);

void match_features(std::vector<cv::Mat>& descriptor_for_all, 
	std::vector<std::vector<cv::DMatch>>& matches_for_all);

//? &Mat 变成可选
bool find_transform(cv::Mat& K, const std::vector<cv::Point2f>& p1, const std::vector<cv::Point2f>& p2, 
	cv::Mat& R, cv::Mat& T, cv::Mat& mask);

void get_matched_points(
	std::vector<cv::KeyPoint>& p1, 
	std::vector<cv::KeyPoint>& p2, 
	std::vector<cv::DMatch> matches, 
	std::vector<cv::Point2f>& out_p1, 
	std::vector<cv::Point2f>& out_p2);


void get_matched_colors(
	std::vector<cv::Vec3b>& c1,
	std::vector<cv::Vec3b>& c2,
	std::vector<cv::DMatch> matches,
	std::vector<cv::Vec3b>& out_c1,
	std::vector<cv::Vec3b>& out_c2);

void reconstruct(
	const cv::Mat& K, 
	const cv::Mat& R1, const cv::Mat& T1, 
	const cv::Mat& R2, const cv::Mat& T2, 
	const std::vector<cv::Point2f>& p1, const std::vector<cv::Point2f>& p2, 
	std::vector<cv::Point3f>& structure);


void maskout_points(std::vector<cv::Point2f>& p1, cv::Mat& mask);


void maskout_colors(std::vector<cv::Vec3b>& p1, cv::Mat& mask);


void save_structure(std::string file_name, 
	std::vector<cv::Mat>& rotations, std::vector<cv::Mat>& motions, 
	std::vector<cv::Point3f>& structure, std::vector<cv::Vec3b>& colors);


void get_objpoints_and_imgpoints(
	std::vector<cv::DMatch>& matches,
	std::vector<int>& struct_indices, 
	std::vector<cv::Point3f>& structure, 
	std::vector<cv::KeyPoint>& key_points,
	std::vector<cv::Point3f>& object_points,
	std::vector<cv::Point2f>& image_points);

void fusion_structure(
	std::vector<cv::DMatch>& matches, 
	std::vector<int>& struct_indices, 
	std::vector<int>& next_struct_indices,
	std::vector<cv::Point3f>& structure, 
	std::vector<cv::Point3f>& next_structure,
	std::vector<cv::Vec3b>& colors,
	std::vector<cv::Vec3b>& next_colors);


void init_structure(
	cv::Mat K,
	std::vector<std::vector<cv::KeyPoint>>& key_points_for_all, 
	std::vector<std::vector<cv::Vec3b>>& colors_for_all,
	std::vector<std::vector<cv::DMatch>>& matches_for_all,
	std::vector<cv::Point3f>& structure,
	std::vector<std::vector<int>>& correspond_struct_idx,
	std::vector<cv::Vec3b>& colors,
	std::vector<cv::Mat>& rotations,
	std::vector<cv::Mat>& motions);

void get_file_names(std::string dir_name, std::vector<std::string> & names);


}
}