#ifndef TDDIAO_PCL_UTILS
#define TDDIAO_PCL_UTILS

#include "types.hpp"
#include <sensor_msgs/PointCloud2.h>

// #include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/filters/radius_outlier_removal.h>
// #include <pcl/filters/impl/extract_indices.hpp>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/surface/mls.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/surface/gp3.h>
// #include <pcl/surface/poisson.h>

namespace tergeo{
namespace visualodometry {

// struct FrameBuffer {
// 	int id;
// 	double stamp;
//     double mean_x = 0;
//     double mean_y = 0;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr points;
// 	// std::vector<pcl::PointXYZ> points = {};
//     FrameBuffer() {}
//     FrameBuffer(int id_, double stamp_):id(id_), stamp(stamp_) {}

//     void calGemoetry() {
//         int pt_size = this->points->points.size();
//         if (pt_size == 0) {
//             mean_x = -1000;
//             mean_y = -1000;
//             return;
//         }
//         for(int i = 0; i < pt_size; ++i){
//             mean_x += this->points->points[i].x;
//             mean_y += this->points->points[i].y;
//         }
//         mean_x /= pt_size;
//         mean_y /= pt_size;
//         return;
//     }
// };
// struct FrameBufferMap {
// 	int buffer_size = 5;
// 	// std::pair<std::map<int, FrameBuffer>::iterator, bool> start_it;
// 	std::pair<std::map<int, FrameBuffer>::iterator, bool> end_it;
// 	// std::map<int, FrameBuffer> obstales = {};
//     std::vector<FrameBuffer> obstales = {};
//     //?baocuo
//     // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud{new pcl::PointCloud<pcl::PointXYZ>};
//     FrameBufferMap() {}
//     FrameBufferMap(int b_size):buffer_size(b_size) {
//         // pcl_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
//     }
// 	void insert(FrameBuffer fb) {
//         //yong make_pair baocuo?
//         obstales.push_back(fb);
//         // end_it = this->obstales.insert(std::pair<int, FrameBuffer>(fb.id, fb));
//         if (this->obstales.size() == 1) {
//             // start_it = end_it;
//         } else if (this->obstales.size() > buffer_size) {
//             // start_it.first->second.points->clear();
//             // delete start_it.first->second.points;
//             // std::cout << "delete\n";
//             // std::cout << "delete " << this->obstales.begin()->stamp << "\n";
//             this->obstales.erase(this->obstales.begin());
//         }
// 	}
//     pcl::PointCloud<pcl::PointXYZ>::Ptr getPCLCloud(bool transformed = false) {
//         pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         total_cloud->clear();
//         if (!transformed) {
//             for (auto it = obstales.begin(); it != obstales.end(); it++) {
//                 *total_cloud += *(it->points);
//             }
//             return total_cloud;
//         }
//         int base_id = obstales.size();
//         for (int i = obstales.size() - 1; i > -1; --i)         {
//             if (!(this->obstales[base_id].mean_x == -1000 && this->obstales[base_id].mean_y == -1000)) break;
//             base_id -= 1;
//         }
//         if (base_id == -1) {
//             return total_cloud;
//         }
//         for (size_t i = 0; i < base_id; ++i) {
//             pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//             Eigen::Matrix4f tranfsorm_ = Eigen::Matrix4f::Identity();
//             tranfsorm_(0, 3) = this->obstales[base_id].mean_x - this->obstales[i].mean_x;  
//             tranfsorm_(1, 3) = this->obstales[base_id].mean_y - this->obstales[i].mean_y;
//             pcl::transformPointCloud(*(this->obstales[i].points), *temp_cloud, tranfsorm_);
//             *total_cloud += *temp_cloud;
//         }
//         return total_cloud;
//     }

// };

// enum PCL_COMPARE {
//     PX,//X
//     PY,//Y
//     PZ,//Z
//     PR,//R
//     OPR,//R
//     OPX,//X
//     OPY,//Y
//     OPZ//Z
// };

// typedef bool (*FunctionPtr)(double x, double y, double z, double thres);

// class filterFuncMapClass {
// public:
//     static inline bool filtPX(double x, double y, double z, double thres);
//     static inline bool filtPY(double x, double y, double z, double thres);
//     static inline bool filtPZ(double x, double y, double z, double thres);
//     static inline bool filtPR(double x, double y, double z, double thres);
//     static inline bool filtOPX(double x, double y, double z, double thres);
//     static inline bool filtOPY(double x, double y, double z, double thres);
//     static inline bool filtOPZ(double x, double y, double z, double thres);
//     static inline bool filtOPR(double x, double y, double z, double thres);
// };

// //   x             y             z    点索引与grid索引的对应关系
// typedef std::map<int, std::map<int, std::map<int, std::pair<std::vector<int>, std::vector<pcl::PointXYZ>>>>> _GridPCloud;

// struct GridPCloud {
//     double resolution;
//     _GridPCloud grid = {};
//     pcl::PointCloud<pcl::PointXYZ>::Ptr points = nullptr;
//     //存储点索引与grid索引的对应关系
//     // std::vector<std::pair<int, std::vector<int>>> indices = {};
//     GridPCloud() {
//         points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>); 
//     }
//     ~GridPCloud() {}
//     GridPCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud, double resolution) {
//         this->constructGrid(pcloud, resolution);
//     }
//     void constructGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud, double resolution);
//     void clear() {
//         this->grid.clear();
//         // this->indices.clear();
//         if (points) {
//             this->points->points.clear();
//         }
//     }
//     bool empty() {
//         bool valid = (grid.empty() == points->points.empty());
//         // valid = valid && (grid.empty() == indices.empty());
//         if (!valid) {
//             std::cout << "invalid grid map, return grid.empty()!\n";
//         }
//         return grid.empty();
//     }
//     void update();
//     cv::Point3f getNearstPoint(const cv::Point3f &pt, double &dis_to_self, bool precise = false) {
//         if (grid.empty()) {
//             dis_to_self = std::numeric_limits<double>::max();
//             return cv::Point3f(-1, -1, -1);
//         }
//         double x = pt.x;
//         double y = pt.y;
//         double z = pt.z;
//         double gx, gy, gz;
//         int xid, yid, zid;
//         bool isfind = false;
//         double dis = std::numeric_limits<double>::max();
//         for (auto xit = grid.begin(); xit != grid.end(); xit++) {
//             gx = xit->first;
//             for (auto yit = grid[gx].begin(); yit != grid[gx].end(); yit++) {
//                 gy = yit->first;
//                 for (auto zit = grid[gx][gy].begin(); zit != grid[gx][gy].end(); zit++) {
//                     gz = zit->first;
//                     double temp_dis = (x - gx) * (x - gx) + (y - gy) * (y - gy) + (z - gz) * (z - gz);
//                     if (temp_dis < dis) {
//                         dis = temp_dis;
//                         xid = gx; yid = gy; zid = gz; 
//                         isfind = true;
//                     }
//                 }
//             }
//         }
//         if (precise) {
//             dis_to_self = std::sqrt(dis);
//             return cv::Point3f(xid, yid, zid);
//         } else {
//             cv::Point3f cvpt;
//             dis = std::numeric_limits<double>::max();
//             for (int i = 0; i < grid[xid][yid][zid].second.size(); ++i) {
//                 const pcl::PointXYZ &gpt = grid[xid][yid][zid].second[i];
//                 double temp_dis = (x - gpt.x) * (x - gpt.x) + (y - gpt.y) * (y - gpt.y) + (z - gpt.z) * (z - gpt.z);
//                 if (temp_dis < dis) {
//                     dis = temp_dis;
//                     cvpt.x = gpt.x;
//                     cvpt.y = gpt.y;
//                     cvpt.z = gpt.z;
//                 }
//             }
//             dis_to_self = dis;
//             return cvpt;
//         }
//         return cv::Point3f();
        
//     }

// };

// typedef std::vector<std::pair<double, PCL_COMPARE>> PclFilter;

// int *rand_rgb();
	
// pcl::PolygonMesh pointCloudGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud) ;

// std::vector<pcl::PointIndices> euclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud, double thres);

// void preProcess(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud);

// void featureExtraction(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);

// // CubeResult getCube(
// //     pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);

// //只要小于极值的部分
// //                    极值     比较对象
// void filterBYCondition(
//     pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud,
//     const PclFilter &thres = {});

// //先框出预选集合
// //区域生长，遇到已经存在的就不生长，
// void getVehiclePointCloud(
//     pcl::PointXYZ point,
//     pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud, double resolution);

// // void constructGridPCloud(
// //     pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud,
// //     GridPCloud &pointcloud_grid_map, double resolution);

// pcl::PointCloud<pcl::PointXYZ>::Ptr gridCloud2PCLCloud(GridPCloud grid_cloud);pcl::PointCloud<pcl::PointXYZ>::Ptr gridCloud2PCLCloud(GridPCloud grid_cloud);
// sensor_msgs::PointCloud2 toColorPCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcloud, cv::Scalar color);

}
}
#endif