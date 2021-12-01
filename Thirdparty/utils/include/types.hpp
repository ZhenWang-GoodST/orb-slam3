#ifndef TERGEO_VISUAL_ODOMETRY_TYPES
#define TERGEO_VISUAL_ODOMETRY_TYPES

#include <iostream>
// #include <ctime>
#include <thread>
#include <mutex>
#include <chrono>
#include <fstream> 
#include <iomanip>

#include <algorithm>
#include <numeric>
#include <set>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
// #include <opencv2/core/eigen.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include "pcl/io/pcd_io.h"
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#include <Eigen/Dense>
namespace tergeo{
namespace visualodometry {

static cv::Scalar red = cv::Scalar(0, 0, 255);
static cv::Scalar green = cv::Scalar(0, 255, 0);
static cv::Scalar blue = cv::Scalar(255, 0, 0);

struct Match {
    double _coff = 0;
    // counter-clock-wise at degrees
    double _angle = 0;
    cv::Point2f _position;
    cv::Mat _coff_mat;
    // cv::getRotationMatrix2D counter_clock_wise degrees
    // Eigen::Rotation2D 2D counter clock wise radian.
    // rotation martrix set to counter_clock_wise
    Eigen::Matrix2d _R, inver_R;
    Eigen::Vector2d _T, inver_T;
    Match() {}
    Match(cv::Point p, double an, double coff):
        _position(p), _angle(an), _coff(coff){}
    void setTR(double an, double tx, double ty) {
        _R = Eigen::Rotation2D<double>(an);
        bool invertible = false;
        _R.computeInverseWithCheck(inver_R, invertible);
        if (!invertible) {
            std::cout << "error\n";
        }
        _T << tx, ty;
        inver_T = _T;   
    }
    friend std::ostream &operator<<(std::ostream &out, Match &A);
};

struct FeaturePoint {
    int _id;
    double _value;
    double stddev_ori;//原图哈里斯算子响应值
    double stddev_dif;//旋转90°相乘与原图相乘做差的标准差
    cv::Point2d _pt;
    FeaturePoint() {}
    FeaturePoint(int id, double v, cv::Point2d pt):_id(id), _value(v), _pt(pt){}
};

auto cmp=[](FeaturePoint l, FeaturePoint r) {return l._value < r._value;};
auto cmpl=[](FeaturePoint l, FeaturePoint r) {return l._value > r._value;};
using FeatureQueue = std::priority_queue<
        FeaturePoint, std::vector<FeaturePoint>, decltype(cmp)>;
using FeatureLQueue = std::priority_queue<
        FeaturePoint, std::vector<FeaturePoint>, decltype(cmpl)>;

struct MatchPoint {
    FeaturePoint _p1;
    FeaturePoint _p2;
    double _distance;
    MatchPoint(FeaturePoint p1, FeaturePoint p2, double dis):
        _p1(p1), _p2(p2), _distance(dis) {}
};

struct PairInfo {
    int id = -1;
    double dis_in_HT = -1;//harris算子和模板匹配点位距离差值
    double stddev_ori = -1;
    double stddev_dif = -1;
    PairInfo(){}
    PairInfo(int i, double dis, double v1, double v2):
        id(i), dis_in_HT(dis), stddev_ori(v1), stddev_dif(v2) {}
};

struct SortOriStddev {
    bool operator()(PairInfo &p1, PairInfo &p2) const {
        return p1.stddev_ori < p2.stddev_ori;
    }
};
struct SortDifStddev {
    bool operator()(PairInfo &p1, PairInfo &p2) const {
        return p1.stddev_dif < p2.stddev_dif;
    }
};

typedef std::pair<int, PairInfo> PairInfoPair;

enum QueryPtType {
    NO_QUERY = 0,
    QUERY_BY_DEPRECATED = 1,//用于匹配的点
    QUERY_BY_TEMPLATE = 2//用于初始化的点
    // QUERY_BY
};

struct MatchPair {
    int _id;
    int _left_image_id;
    int _right_image_id;
    double min_ori = 1, max_ori = 0, min_dif = 1, max_dif = 0;
    std::vector<std::pair<int, int>> _pairs;
    std::vector<int> _deprecated_pts = {};//will be updated to pairs
    std::vector<int> _label_for_template_filter = {};//used for initilize
    std::vector<PairInfo> _pair_info_vec = {};
    std::map<int, PairInfo> _pair_info_map;
    std::map<int, cv::Point3f> _structures;
    //reprojection error
    std::vector<double> reproj_error = {};
    MatchPair() {}
    MatchPair(int id, int lid, int rid):
        _id(id), _left_image_id(lid), _right_image_id(rid){}
    MatchPair(int id, int lid, int rid, 
        std::vector<std::pair<int, int>> pair):
        _id(id), _left_image_id(lid), _right_image_id(rid), _pairs(pair){}

    std::vector<int> getQuery(int query_type) {
        switch (query_type)
        {
        case QueryPtType::NO_QUERY:
            return std::vector<int>(_pairs.size(), 1);
            break;
        case QueryPtType::QUERY_BY_DEPRECATED:
            return this->_deprecated_pts;
        case QueryPtType::QUERY_BY_TEMPLATE:
            return this->_label_for_template_filter;
        
        default:
            std::cout << "error query type\n";
            break;
        }
    }
    void updateQuery(std::vector<int> query, int query_type) {
        switch (query_type)
        {
        case QueryPtType::QUERY_BY_DEPRECATED:
            for (int i = 0; i < this->_deprecated_pts.size(); ++i) {
                if (query[i] == 0) {
                   this->_deprecated_pts[i] = 0; 
                }
            }
            break;
        case QueryPtType::QUERY_BY_TEMPLATE:
            for (int i = 0; i < this->_deprecated_pts.size(); ++i) {
                if (query[i] == 0) {
                   this->_label_for_template_filter[i] = 0; 
                }
            }
            break;
        default:
            std::cout << "error query type\n";
            break;
        }
    }
    // std::vector<int> _left_points = {};
    // std::vector<int> _right_points = {};
};

struct FeaturePoints {
    int _image_id;
    std::vector<FeaturePoint> _pts = {};
    FeaturePoints(int id, std::vector<FeaturePoint> pts):_image_id(id), _pts(pts){}
};
auto cmpm=[](MatchPoint l, MatchPoint r) {return l._distance < r._distance;};
using MatchQueue = std::priority_queue<
        MatchPoint, std::vector<MatchPoint>, decltype(cmpm)>;

//舍弃
//--------------------------------keyframe id---point id-----------
// using FeaturePointsMap = std::map<int, std::map<int, FeaturePoint>>;

//舍弃
using MatchPointPair = std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>>;
using MatchPoints = std::vector<MatchPointPair>;

//----------------------------匹配id-匹配点对
using MatchPairsMap = std::map<int, MatchPair>;
// using MatchPoint = std::pair<FeaturePoint, FeaturePoint>;

struct KeyFrame {
    //medoubdta data
    double time;
    //推算当前帧位姿
    //由里程计提供的距离信息
    double odometry;
    //由惯导提供的角度
    double angle;
    //由点云提供的平面信息
    std::vector<std::vector<cv::Point2d>> planes = {}; 
    int _id;
    bool _is_initilize_frame = false;
    cv::Mat _R;
    cv::Mat _T;
    cv::Point3f _pose;
    cv::Point3f _rpy;
    cv::Mat _image;
    //存储匹配上序列的id
    std::vector<int> _matchId = {};
    //存储影像提取二维特征点,从提取的map序列放入该map
    std::map<int, FeaturePoint> _featurePts = {};
    //存储图像点对应的三维空间点
    //-------与图像特征点统一的id----------------
    std::map<int, cv::Point3f> _threeDPts = {};
    KeyFrame() {}
    KeyFrame(int id, cv::Point3f pose, cv::Point3f rpy, cv::Mat image):
        _id(id), _pose(pose), _rpy(rpy), _image(image) {}
    
    std::vector<cv::Point2d> getCVPtsVec() {
        std::vector<cv::Point2d> pt_vec = {};
        for (auto it = _featurePts.begin(); it != _featurePts.end(); ++it) {
            pt_vec.push_back(it->second._pt);
        }
        return pt_vec;
    }
};

}
}

#endif