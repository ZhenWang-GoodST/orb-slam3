#ifndef TERGEO_VISUAL_ODOMETRY_MATCHER
#define TERGEO_VISUAL_ODOMETRY_MATCHER
// #include <iostream>
// #include <opencv2/opencv.hpp>
// // #include <ctime>
// #include <thread>
// #include <mutex>
// #include <chrono>
// #include <fstream> 
// #include <direct.h>


// #include <ros/ros.h>
// #include <sensor_msgs/Imu.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <Eigen/Core>
// #include <Eigen/Dense>

#include "sfm.h"
#include "types.hpp"

namespace tergeo {

namespace visualodometry {
//创新点,需要整合sift,harris特征描述符计算
//第一级:1. 模板匹配(平面情况); 2. sift特征点匹配(任意情况),得到粗匹配关系
//第二级:harris算子小模板匹配筛选得到精匹配,求得两帧RT
//第三级:搜索sift尺度内harris特征点,根据精匹配RT将其转换到左右片,
//小模板,筛选得到所有符合要求的精匹配


//下一步计划
//第一级:1.加入降采样2.加入深度信息,提取子平面区域模板匹配
//第一级，大尺度进行粗匹配，小尺度精匹配
//第二级:第二级判断不同级别的匹配点个数，调整query参数，进行误差传播，指示何时重新初始化
//第三级:1.计算第二级提取的点在sift各个尺度下得分值,作为参考向量
//计算所有点在sift下各个尺度得分值,与参考向量计算相似性,排除尺度不符点
//2.在排除之后进行左到右,右到左的第二级匹配:
//方差之比>阈值,min < ori <max, min < dif <max

//提取harris特征点取前一半的极值，是否有显著下降的，或者缩小局部非极大值抑制范围


//由sift算子尺度提供候选harris区域,这样去掉平面点
//sift尺度范围里找左右harris特征点,
//根据1.单应矩阵; 2.本征矩阵RT求得相对位姿; 3.solvePnP求得的转换关系
//将左片转换到右片,计算模板响应值
//右片转换到左片,计算响应值

//根据harris特征图筛选平面点

//harris 算子多级匹配，利用集群点信息，计算几何相似度
class Matcher {
private:
    /* data */
public:
    cv::Mat _K;
    cv::Mat _image, _keypoints_image;
    cv::Mat _last_image, _current_image;
    int _left_image_id, _right_image_id;
    int _keyframe_count, _image_cout;
    double _f;
    double _angle, _distance;

    //特征提取
    cv::Mat _harris_map;
    //
    cv::Mat RT;//H or E
    
    std::vector<cv::Mat> _descriptor_for_all;
    std::vector<std::vector<cv::Vec3b>> _colors_for_all;
    std::vector<std::vector<cv::KeyPoint>> _key_points_for_all;
    cv::Mat _descriptors, _last_descriptors;
    std::vector<cv::KeyPoint> _key_points;
    std::vector<cv::Vec3b> _colors;
    // record the translate from left to right at pixel-coordinate
    // and the origin is in the center of the image
    // 影像坐标系，右手系， 前方为x，左方为y
    // 像素坐标系→影像坐标系
    Match match, submatch;

    // 3d reconstruction
    bool isinitilized = false;
    cv::Point3f _global_odometry;
    //存储第i张相片的特征点序列,2D
    // FeaturePointsMap featurePtsMap = {};
    //舍弃 存储匹配点对
    MatchPoints matchPts = {};
    //存储匹配点对
    int _match_pairId = 0;
    //匹配id,
    //匹配左右影像id,
    //匹配左右点对,
    //匹配三维点坐标
    //匹配选择平移矩阵
    MatchPairsMap matchPairs = {};
    // std::vector<FeaturePoints> FPTS = {};
    //3D点存在关键帧里面

    //舍弃
    // std::vector<std::vector<cv::Point3f>> structures = {};
    // std::vector<std::vector<cv::Point3f>> threeDPoints = {};
    std::vector<cv::Mat> rotations = {};
    std::vector<cv::Mat> motions = {};
    std::map<int, KeyFrame> _keyframes = {};

    Matcher(/* args */) {}
    ~Matcher() {}
    void initialize() {
        pcl_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }
    void load(
        const cv::Mat &K,
        // const cv::Mat &last_image, 
        const cv::Mat &current_image,
        double angle, double distance, cv::Point3f odometry);
    int featureMatch(cv::Mat &R, cv::Mat &T, cv::Mat &matched_image);
    
    int templateMatch(cv::Mat &R, cv::Mat &T, 
    cv::Mat &TMatched_image, Match &result);
    
    void calAngle();
    
    cv::Mat getPredictionPatch(const cv::Mat image, const cv::Rect &rect);
    
    Match baseTemplate(const cv::Mat &image, 
        const cv::Mat &templ, cv::Mat &TMatched_image, double initial_angle = 0);
    
    cv::Mat extractPatch(cv::Mat image);
    
    std::vector<cv::Rect2d> extractPatchs(cv::Mat image, double step, double width, double height);

    void singleMatch(const cv::Mat &image, 
        const cv::Mat &templ, cv::Mat &TMatched_image, Match match);

    cv::Mat getInnerPatch(const cv::Mat &image, const cv::Mat &templ);
    
    // extract points
    void extractPoints(const cv::Mat &image, FeatureLQueue &max_feature_points, 
        std::map<int, FeaturePoint> &feature_points, int step);
    //x - cols, y - rows
    void getMatchedPoints(double x, double y, double &out_x, double &);
    
    // calcuate E-martrix
    bool getRT(const std::vector<cv::Point2f> &p1,
        const std::vector<cv::Point2f> &p2, cv::Mat &R, cv::Mat &T, cv::Mat &RT, cv::Mat &mask, int method = 0, int mode = 0);
    //match points
    bool exceedImage(double x, double y, const cv::Mat &image, bool translate = true);
    bool exceedImage(const cv::Rect &rect, const cv::Mat &image, bool translate = true);
    bool getMatchPoints(
        const Match &match,
        std::map<int, FeaturePoint> &leftPts,
        std::map<int, FeaturePoint> &rightPts,
        MatchQueue &match_queue,
        MatchPair &matchpair);


    void getMatchedPointsByTemplateL2R(MatchPair &matchpair,
        std::map<int, FeaturePoint> &pts1, std::map<int, FeaturePoint> &pts2,
        const cv::Mat &image1, const cv::Mat &image2, std::vector<int> mask, 
        int mode, int predict_mode);
    
    //mode = 0,使用模板匹配
    //mode = 1,使用单应矩阵匹配
    //mode = 3,使用sift大尺度匹配
    void wrapGetMatchedPointsByTemplateL2R(int predict_mode);

    void filtMatchPoints(int matchId, int method) {
        std::vector<int> discard_match_id(matchPairs[matchId]._pairs.size(), 1);
        matchPairs[_match_pairId]._label_for_template_filter = discard_match_id;
        if (method == 0) {
            filtByTemplate(matchId, matchPairs[_match_pairId]._label_for_template_filter);
        } else if(method == 1) {
            filtByReprojectionError(matchId, matchPairs[_match_pairId]._label_for_template_filter);
        }
        auto &matchpair = matchPairs[_match_pairId]._pairs;
        drawMatchPoints(_show_image, matchPairs[_match_pairId]._label_for_template_filter, matchpair);
        cv::imshow("matched", _show_image);
        // cv::waitKey();
        //erase outlier
        if (discard_match_id.empty()) return;
        // for (int i = discard_match_id.size() - 1; i >= 0; i--) {
        //     matchpair.erase(matchpair.begin() + discard_match_id[i]);
        // }
    }

    void filtByTemplate(int matchId, std::vector<int> &discard_match_id);

    void filtByReprojectionError(int matchId, std::vector<int> &discard_match_id);
    
    
    std::vector<cv::KeyPoint> getCVKeyPts(int image_id, std::vector<int> mask = {});

    
    void initializeCoordinate();

    void solvePnP();

    void solveHomography();

    //definition for debug
    int _point_count = 0;
    ros::Publisher structure_publisher;
    sensor_msgs::PointCloud2 structure_pointcloud;
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud; 
    cv::Vec3b blue = cv::Vec3b(255, 0, 0);
    cv::Vec3b red = cv::Vec3b(0, 0, 255);
    cv::Vec3b green = cv::Vec3b(0, 255, 0);
    cv::Size2d size20 = cv::Size2d(20, 20);
    cv::Size2d size30 = cv::Size2d(30, 30);
    cv::Size2d size40 = cv::Size2d(40, 40);
    cv::Size2d size50 = cv::Size2d(50, 50);
    cv::Size2d size60 = cv::Size2d(60, 60);
    cv::Mat _show_image;
    cv::Mat image_left;
    cv::Mat image_right;
    ros::Time _time;
    void drawMatchPoints(cv::Mat &show_image, 
        const std::vector<int> &ids, 
        const std::vector<std::pair<int, int>> &pair);
};

struct Result {
    int _id;
    double _distance;
    tergeo::visualodometry::Match _match;
    double left_image_id, right_image_id;
    Result(int id, double dis, tergeo::visualodometry::Match match,
        double left, double right):_id(id), _distance(dis), _match(match), 
        left_image_id(left), right_image_id(right){}

    friend std::ostream &operator<<(std::ostream &out, Result &A);
    friend std::ostream &operator<<(std::ofstream &out, Result &A);
};

}
}

#endif
