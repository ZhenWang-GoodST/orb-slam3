#ifndef TERGEO_VISUAL_ODOMETRY_PHOTOGRAMMETRY
#define TERGEO_VISUAL_ODOMETRY_PHOTOGRAMMETRY

#include "types.hpp"

namespace tergeo{
namespace visualodometry {


bool relativeOrientation(
    std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2,
    Eigen::MatrixXd &R, Eigen::MatrixXd &T, Eigen::MatrixXd K,
    double &fai1, double &omg1, double &kap1,
    double &fai2, double &omg2, double &kap2);


//前方交会，输入为计算机视觉的Ｒ，Ｔ，二维测量点
bool forwardIntersection(
    const cv::Mat &cR1, const cv::Mat &cT1, const cv::Mat &cR2, const cv::Mat &cT2, const cv::Mat &K,
    const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2,
    std::vector<cv::Point3f> &ThDPts, int mode = 0);

bool FINProjection(
    const cv::Mat &R1, const cv::Mat &T1, const cv::Mat &R2, const cv::Mat &T2, const cv::Mat &K,
    const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2,
    std::vector<cv::Point3f> &ThDPts);




}
}


#endif