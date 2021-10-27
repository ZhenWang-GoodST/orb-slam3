#ifndef TERGEO_VISUAL_ODOMETRY_OPENCV_UTILS
#define TERGEO_VISUAL_ODOMETRY_OPENCV_UTILS

#include "types.hpp"

namespace tergeo{
namespace visualodometry {

void drawMatchPts(
    const cv::Mat &left, const cv::Mat &right, cv::Mat &show_image,
    const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2,
    cv::Scalar color = cv::Scalar(0, 255, 0), bool update = true);

void drawKeyPts(
    cv::Mat &image, const std::vector<cv::KeyPoint> &pts, int size, const  cv::Scalar color = cv::Scalar(0, 255, 0));

void drawMatchPts(
    const cv::Mat &left, const cv::Mat &right, cv::Mat &show_image,
    const std::vector<cv::KeyPoint> &p1, const std::vector<cv::KeyPoint> &p2,
    const std::vector<int> &indices, cv::Scalar color, bool update);

void drawMatchPtsInCircle(
    const cv::Mat &image, const std::vector<cv::Point2f> &pts, const std::vector<double> &radius = {}, std::vector<int> mask = {}, cv::Scalar color = cv::Scalar(0, 0, 255));

template <typename T>
void printCVPoint(std::vector<T> pts1, std::vector<T> pts2) {
    // std::cout << std::setprecision(5) << "\n";
    for (int i = 0; i < pts1.size(); ++i) {
        std::cout << i << " : " << pts1[i] << " " << pts2[i] << "\n"; 
    }
    std::cout << "\n";
}

void drawCube(cv::Mat &image, const std::vector<cv::Point2f> cube, cv::Scalar color = cv::Scalar(0, 255, 0));

void adaptiveThreshold(const cv::Mat &hist, double thresh, int &minId, int &maxId);

cv::Mat stretch(const cv::Mat& image,int minvalue = 0, int maxvalue = 255);

cv::Mat stretch(const cv::Mat& image, int minvalue = 0, int maxvalue = 255, bool makeFloat = false);
void singleMatch(const cv::Mat &image, 
    const cv::Mat &templ, cv::Mat &TMatched_image, Match match);

cv::Mat getInnerPatch(const cv::Mat &image, const cv::Mat &templ);

Match baseTemplate(const cv::Mat &image, 
        const cv::Mat &templ, cv::Mat &TMatched_image, double initial_angle = 0);

}
}
namespace cv_wz {
using namespace cv;
    int decomposeHomographyMat(InputArray _H,
                        InputArray _K,
                        OutputArrayOfArrays _rotations,
                        OutputArrayOfArrays _translations,
                        OutputArrayOfArrays _normals, int mode);

}


#endif