#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>

#include <iostream> //标准输入输出流
// #include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
// #include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
// #include <pcl/visualization/cloud_viewer.h>

// using namespace cv;
// using namespace std;
namespace tergeo {
namespace visualodometry {
void extract_features(
	const cv::Mat &image,
	cv::Mat& descriptors,
	std::vector<cv::Vec3b>& colors,
	std::vector<cv::KeyPoint>& key_points) {
	descriptors.release();
	colors.clear();
	key_points.clear();
	if (image.empty()) return;
	// cout << "Extracing features: " << endl;

	//偶尔出现内存分配失败的错误
	// cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create(20, 5, 0.005, 1.6);
	// sift->detectAndCompute(image, cv::noArray(), key_points, descriptors);

	cv::Ptr<cv::Feature2D> surf = cv::xfeatures2d::SURF::create(100, 3, 3);
	surf->detectAndCompute(image, cv::noArray(), key_points, descriptors);
    
	// cv::Ptr<cv::ORB> orb = cv::ORB::create(100, 1.2f, 3, 30, 0);
	// orb->detectAndCompute(image, cv::noArray(), key_points, descriptors);
	//特征点过少，则排除该图像
	std::cout << key_points.size() << "\n";
	if (key_points.size() <= 10) return;

	colors.resize(key_points.size());
	for (int i = 0; i < key_points.size(); ++i)
	{
		cv::Point2f& p = key_points[i].pt;
		colors[i] = image.at<cv::Vec3b>(p.y, p.x);
	}
}

void match_features(cv::Mat& query, cv::Mat& train, std::vector<cv::DMatch>& matches)
{
	std::vector<std::vector<cv::DMatch>> knn_matches;
	cv::BFMatcher matcher(cv::NORM_L2);
	matcher.knnMatch(query, train, knn_matches, 2);

	//获取满足Ratio Test的最小匹配的距离
	float min_dist = FLT_MAX;
	for (int r = 0; r < knn_matches.size(); ++r)
	{
		//Ratio Test
		if (knn_matches[r][0].distance > 0.6*knn_matches[r][1].distance)
			continue;

		float dist = knn_matches[r][0].distance;
		if (dist < min_dist) min_dist = dist;
	}

	matches.clear();
	for (size_t r = 0; r < knn_matches.size(); ++r)
	{
		//排除不满足Ratio Test的点和匹配距离过大的点
		if (
			knn_matches[r][0].distance > 0.6*knn_matches[r][1].distance ||
			knn_matches[r][0].distance > 5 * std::max(min_dist, 10.0f)
			)
			continue;

		//保存匹配点
		matches.push_back(knn_matches[r][0]);
	}
}

void template_match(const cv::Mat &mat1, const cv::Mat &mat2, cv::Mat &result) {
	cv::matchTemplate(mat1, mat2, result, cv::TM_SQDIFF);
}
void match_features(std::vector<cv::Mat>& descriptor_for_all, std::vector<std::vector<cv::DMatch>>& matches_for_all)
{
	matches_for_all.clear();
	// n个图像，两两顺次有 n-1 对匹配
	// 1与2匹配，2与3匹配，3与4匹配，以此类推
	std::cout << descriptor_for_all.size() << "\n";
	for (int i = 0; i + 1< descriptor_for_all.size() ; ++i)
	{
		std::cout << "Matching images " << i << " - " << i + 1 << std::endl;
		std::vector<cv::DMatch> matches;
		match_features(descriptor_for_all[i], descriptor_for_all[i + 1], matches);
		matches_for_all.push_back(matches);
	}
}

bool find_transform(cv::Mat& K, const std::vector<cv::Point2f>& p1, const std::vector<cv::Point2f>& p2, cv::Mat& R, cv::Mat& T, cv::Mat& mask)
{
	//根据内参矩阵获取相机的焦距和光心坐标（主点坐标）
	double focal_length = 0.5*(K.at<double>(0) + K.at<double>(4));
	cv::Point2d principle_point(K.at<double>(2), K.at<double>(5));
    // std::cout << focal_length << principle_point << "\n";
	//根据匹配点求取本征矩阵，使用RANSAC，进一步排除失配点
	cv::Mat E = findEssentialMat(p1, p2, focal_length, principle_point, cv::RANSAC, 0.999, 3.0, mask);
	// E = cv::findHomography ( p1, p2, cv::RANSAC, 3, noArray(), 2000, 0.99 );
	if (E.empty()) {
		std::cout << "return E.empty()\n";
	    return false;	
	} 
	double feasible_count = countNonZero(mask);
	std::cout << (int)feasible_count << " -in- " << p1.size() << std::endl;
	//对于RANSAC而言，outlier数量大于50%时，结果是不可靠的
	if (feasible_count < 6 || (feasible_count / p1.size()) < 0.5) {
        std::cout << "too few or small success percentage points\n";
		return false;
	}
	// if (feasible_count <= 15 || (feasible_count / p1.size()) < 0.6)

	//分解本征矩阵，获取相对变换
	// std::cout << E.size() << "E SIZE\n"; 
	int pass_count = recoverPose(E, p1, p2, R, T, focal_length, principle_point, mask);

	//同时位于两个相机前方的点的数量要足够大
	if (((double)pass_count) / feasible_count < 0.8)
		return false;
    
	return true;
}

void get_matched_points(
	std::vector<cv::KeyPoint>& p1, 
	std::vector<cv::KeyPoint>& p2, 
	std::vector<cv::DMatch> matches, 
	std::vector<cv::Point2f>& out_p1, 
	std::vector<cv::Point2f>& out_p2
	)
{
	out_p1.clear();
	out_p2.clear();
	for (int i = 0; i < matches.size(); ++i)
	{
		out_p1.push_back(p1[matches[i].queryIdx].pt);
		out_p2.push_back(p2[matches[i].trainIdx].pt);
	}
}

void get_matched_colors(
	std::vector<cv::Vec3b>& c1,
	std::vector<cv::Vec3b>& c2,
	std::vector<cv::DMatch> matches,
	std::vector<cv::Vec3b>& out_c1,
	std::vector<cv::Vec3b>& out_c2
	)
{
	out_c1.clear();
	out_c2.clear();
	for (int i = 0; i < matches.size(); ++i)
	{
		out_c1.push_back(c1[matches[i].queryIdx]);
		out_c2.push_back(c2[matches[i].trainIdx]);
	}
}

void reconstruct(
	const cv::Mat& K, 
	const cv::Mat& R1, const cv::Mat& T1, 
	const cv::Mat& R2, const cv::Mat& T2, 
	const std::vector<cv::Point2f>& p1, const std::vector<cv::Point2f>& p2, 
	std::vector<cv::Point3f>& structure)
{
	//两个相机的投影矩阵[R T]，triangulatePoints只支持float型
	cv::Mat proj1(3, 4, CV_32FC1);
	cv::Mat proj2(3, 4, CV_32FC1);

	R1.convertTo(proj1(cv::Range(0, 3), cv::Range(0, 3)), CV_32FC1);
	T1.convertTo(proj1.col(3), CV_32FC1);

	R2.convertTo(proj2(cv::Range(0, 3), cv::Range(0, 3)), CV_32FC1);
	T2.convertTo(proj2.col(3), CV_32FC1);

	cv::Mat fK;
	K.convertTo(fK, CV_32FC1);
	proj1 = fK*proj1;
	proj2 = fK*proj2;

	//三角重建
	cv::Mat s;
	triangulatePoints(proj1, proj2, p1, p2, s);

	structure.clear();
	structure.reserve(s.cols);
	for (int i = 0; i < s.cols; ++i)
	{
		cv::Mat_<float> col = s.col(i);
		col /= col(3);	//齐次坐标，需要除以最后一个元素才是真正的坐标值
		structure.push_back(cv::Point3f(col(0), col(1), col(2)));
	}
}

void maskout_points(std::vector<cv::Point2f>& p1, cv::Mat& mask)
{
	std::vector<cv::Point2f> p1_copy = p1;
	p1.clear();

	for (int i = 0; i < mask.rows; ++i)
	{
		if (mask.at<uchar>(i) > 0)
			p1.push_back(p1_copy[i]);
	}
}

void maskout_colors(std::vector<cv::Vec3b>& p1, cv::Mat& mask)
{
	std::vector<cv::Vec3b> p1_copy = p1;
	p1.clear();

	for (int i = 0; i < mask.rows; ++i)
	{
		if (mask.at<uchar>(i) > 0)
			p1.push_back(p1_copy[i]);
	}
}

void save_structure(std::string file_name, std::vector<cv::Mat>& rotations, std::vector<cv::Mat>& motions, std::vector<cv::Point3f>& structure, std::vector<cv::Vec3b>& colors)
{
	int n = (int)rotations.size();

	std::ofstream fs(file_name, std::ios::app);
	fs << "Camera Count" << n;
	fs << "Point Count" << (int)structure.size();
	
	fs << "Rotations" << "[";
	for (size_t i = 0; i < n; ++i)
	{
		fs << rotations[i];
	}
	fs << "]";

	fs << "Motions" << "[";
	for (size_t i = 0; i < n; ++i)
	{
		fs << motions[i];
	}
	fs << "]";

	fs << "Points" << "[";
	for (size_t i = 0; i < structure.size(); ++i)
	{
		fs << structure[i];
	}
	fs << "]";

	fs << "Colors" << "[";
	for (size_t i = 0; i < colors.size(); ++i)
	{
		fs << colors[i];
	}
	fs << "]";

	fs.close();
}

void get_objpoints_and_imgpoints(
	std::vector<cv::DMatch>& matches,
	std::vector<int>& struct_indices, 
	std::vector<cv::Point3f>& structure, 
	std::vector<cv::KeyPoint>& key_points,
	std::vector<cv::Point3f>& object_points,
	std::vector<cv::Point2f>& image_points)
{
	object_points.clear();
	image_points.clear();

	for (int i = 0; i < matches.size(); ++i)
	{
		int query_idx = matches[i].queryIdx;
		int train_idx = matches[i].trainIdx;

		int struct_idx = struct_indices[query_idx];
		if (struct_idx < 0) continue;

		object_points.push_back(structure[struct_idx]);
		image_points.push_back(key_points[train_idx].pt);
	}
}

void fusion_structure(
	std::vector<cv::DMatch>& matches, 
	std::vector<int>& struct_indices, 
	std::vector<int>& next_struct_indices,
	std::vector<cv::Point3f>& structure, 
	std::vector<cv::Point3f>& next_structure,
	std::vector<cv::Vec3b>& colors,
	std::vector<cv::Vec3b>& next_colors
	)
{
	for (int i = 0; i < matches.size(); ++i)
	{
		int query_idx = matches[i].queryIdx;
		int train_idx = matches[i].trainIdx;

		int struct_idx = struct_indices[query_idx];
		if (struct_idx >= 0) //若该点在空间中已经存在，则这对匹配点对应的空间点应该是同一个，索引要相同
		{
			next_struct_indices[train_idx] = struct_idx;
			continue;
		}

		//若该点在空间中已经存在，将该点加入到结构中，且这对匹配点的空间点索引都为新加入的点的索引
		structure.push_back(next_structure[i]);
		colors.push_back(next_colors[i]);
		struct_indices[query_idx] = next_struct_indices[train_idx] = structure.size() - 1;
	}
}

void init_structure(
	cv::Mat K,
	std::vector<std::vector<cv::KeyPoint>>& key_points_for_all, 
	std::vector<std::vector<cv::Vec3b>>& colors_for_all,
	std::vector<std::vector<cv::DMatch>>& matches_for_all,
	std::vector<cv::Point3f>& structure,
	std::vector<std::vector<int>>& correspond_struct_idx,
	std::vector<cv::Vec3b>& colors,
	std::vector<cv::Mat>& rotations,
	std::vector<cv::Mat>& motions
	)
{
	//计算头两幅图像之间的变换矩阵
	std::vector<cv::Point2f> p1, p2;
	std::vector<cv::Vec3b> c2;
	cv::Mat R, T;	//旋转矩阵和平移向量
	cv::Mat mask;	//mask中大于零的点代表匹配点，等于零代表失配点
	get_matched_points(key_points_for_all[0], key_points_for_all[1], matches_for_all[0], p1, p2);
	get_matched_colors(colors_for_all[0], colors_for_all[1], matches_for_all[0], colors, c2);
	find_transform(K, p1, p2, R, T, mask);

	//对头两幅图像进行三维重建
	maskout_points(p1, mask);
	maskout_points(p2, mask);
	maskout_colors(colors, mask);

	cv::Mat R0 = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat T0 = cv::Mat::zeros(3, 1, CV_64FC1);
	reconstruct(K, R0, T0, R, T, p1, p2, structure);
	//保存变换矩阵
	rotations = { R0, R };
	motions = { T0, T };

	//将correspond_struct_idx的大小初始化为与key_points_for_all完全一致
	correspond_struct_idx.clear();
	correspond_struct_idx.resize(key_points_for_all.size());
	for (int i = 0; i < key_points_for_all.size(); ++i)
	{
		correspond_struct_idx[i].resize(key_points_for_all[i].size(), -1);
	}
	
	//填写头两幅图像的结构索引
	int idx = 0;
	std::vector<cv::DMatch>& matches = matches_for_all[0];
	for (int i = 0; i < matches.size(); ++i)
	{
		if (mask.at<uchar>(i) == 0)
			continue;

		correspond_struct_idx[0][matches[i].queryIdx] = idx;
		correspond_struct_idx[1][matches[i].trainIdx] = idx;
		++idx;
	}
}

// int main()
// {
// 	vector<string> img_names;
// 	// get_file_names("images", img_names);
// 	img_names.push_back("/home/wz/tmp/threeDreconstruction/images/0001.png");
// 	img_names.push_back("/home/wz/tmp/threeDreconstruction/images/0002.png");
// 	img_names.push_back("/home/wz/tmp/threeDreconstruction/images/0003.png");

// 	//本征矩阵
// 	Mat K(Matx33d(
// 		2759.48, 0, 1520.69,
// 		0, 2764.16, 1006.81,
// 		0, 0, 1));
//     std::cout << "test\n";
// 	vector<vector<KeyPoint>> key_points_for_all;
// 	vector<Mat> descriptor_for_all;
// 	vector<vector<Vec3b>> colors_for_all;
// 	vector<vector<DMatch>> matches_for_all;
// 	//提取所有图像的特征
// 	extract_features(img_names, key_points_for_all, descriptor_for_all, colors_for_all);
// 	//对所有图像进行顺次的特征匹配
// 	match_features(descriptor_for_all, matches_for_all);
//     std::cout << "test\n";
// 	vector<Point3f> structure;
// 	vector<vector<int>> correspond_struct_idx; //保存第i副图像中第j个特征点对应的structure中点的索引
// 	vector<Vec3b> colors;
// 	vector<Mat> rotations;
// 	vector<Mat> motions;

// 	//初始化结构（三维点云）
// 	init_structure(
// 		K,
// 		key_points_for_all,
// 		colors_for_all,
// 		matches_for_all,
// 		structure,
// 		correspond_struct_idx,
// 		colors,
// 		rotations,
// 		motions
// 		);

// 	//增量方式重建剩余的图像
// 	for (int i = 1; i < matches_for_all.size(); ++i)
// 	{
// 		vector<Point3f> object_points;
// 		vector<Point2f> image_points;
// 		Mat r, R, T;
// 		//Mat mask;

// 		//获取第i幅图像中匹配点对应的三维点，以及在第i+1幅图像中对应的像素点
// 		get_objpoints_and_imgpoints(
// 			matches_for_all[i], 
// 			correspond_struct_idx[i], 
// 			structure,
// 			key_points_for_all[i+1], 
// 			object_points,
// 			image_points
// 			);

// 		//求解变换矩阵
// 		solvePnPRansac(object_points, image_points, K, noArray(), r, T);
// 		//将旋转向量转换为旋转矩阵
// 		Rodrigues(r, R);
// 		//保存变换矩阵
// 		rotations.push_back(R);
// 		motions.push_back(T);

// 		vector<Point2f> p1, p2;
// 		vector<Vec3b> c1, c2;
// 		get_matched_points(key_points_for_all[i], key_points_for_all[i + 1], matches_for_all[i], p1, p2);
// 		get_matched_colors(colors_for_all[i], colors_for_all[i + 1], matches_for_all[i], c1, c2);

// 		//根据之前求得的R，T进行三维重建
// 		vector<Point3f> next_structure;
// 		reconstruct(K, rotations[i], motions[i], R, T, p1, p2, next_structure);

// 		//将新的重建结果与之前的融合
// 		fusion_structure(
// 			matches_for_all[i], 
// 			correspond_struct_idx[i], 
// 			correspond_struct_idx[i + 1],
// 			structure, 
// 			next_structure,
// 			colors,
// 			c1
// 			);
// 	}
// 	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
// 	for (size_t i = 0; i < structure.size(); i++)
// 	{
// 		double x = structure[i].x;
// 		double y = structure[i].y;
// 		double z = structure[i].z;
// 		cloud->points.push_back(pcl::PointXYZ(x, y ,z));
// 	}
// 	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z");
// 	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
//     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); 
// 	 while (!viewer->wasStopped())
//     {
//         viewer->spinOnce(100);
//         boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//     }
// 	pcl::visualization::CloudViewer viewer("pcd viewer");
// 	viewer.showCloud(cloud);
// 	while (true)
// 	{
// 		/* code */
// 	}
	
// 	//保存
// 	save_structure("../Viewer/structure.yml", rotations, motions, structure, colors);
// 	return 0;
// }
}
}