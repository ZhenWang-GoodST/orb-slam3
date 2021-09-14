#ifndef TERGEO_VISUAL_ODOMETRY_COMPUTERVISION
#define TERGEO_VISUAL_ODOMETRY_COMPUTERVISION

#include "types.hpp"
#include "opencv_utils.h"

namespace tergeo{
namespace visualodometry {

cv::Point2d predicteHPoint(const cv::Point2d &pt, const cv::Mat &H, int mode = 0);

bool reprojectionError(
    const std::vector<cv::Point3f> &pt3d,
    const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2,
    Eigen::MatrixXd &R1, Eigen::MatrixXd &T1, 
    Eigen::MatrixXd &R2, Eigen::MatrixXd &T2,
    Eigen::MatrixXd &K, double f, 
    std::vector<double> &perror1, std::vector<double> &perror2);

//? &Mat 变成可选
bool getRTByEssential(cv::Mat& K, const std::vector<cv::Point2f>& p1, const std::vector<cv::Point2f>& p2, cv::Mat& R, cv::Mat& T, cv::Mat& mask);

bool getRTByHomography(cv::Mat& K, const std::vector<cv::Point2f>& p1, 
            const std::vector<cv::Point2f>& p2, std::vector<cv::Point3f> D3pts, 
            cv::Mat& R, cv::Mat& T, cv::Mat &H, cv::Mat& mask, int mode = 0);

bool ReconstructH(std::vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
            cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

bool HReprojectionError(
        std::vector<cv::Point2f> p1, std::vector<cv::Point2f> p2,
        const cv::Mat &H, std::vector<cv::Point2f> &HP2, std::vector<int> &query, std::vector<double> &herror, double thres = 2);

int CheckRT(const cv::Mat &R, const cv::Mat &t, const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2,
            const std::vector<std::pair<int, int>> &vMatches12, std::vector<bool> &vbMatchesInliers,
            const cv::Mat &K, std::vector<cv::Point3f> &vP3D, float th2, std::vector<bool> &vbGood, float parallax);


void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

void checkInterAngle(
    const cv::Mat &_R1, cv::Mat &_T1, const cv::Mat &_R2, cv::Mat &_T2,
    const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2,
    const cv::Mat &_K, std::vector<double> &angle_vec);

}



}


#endif