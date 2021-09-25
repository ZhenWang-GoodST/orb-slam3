#include "opencv_utils.h"

namespace tergeo {
namespace visualodometry {

void drawMatchPts(
    const cv::Mat &left, const cv::Mat &right, cv::Mat &show_image,
    const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2,
    cv::Scalar color, bool update) {
    
    int pt_size = p1.size();
    if (update) {
        show_image = cv::Mat::zeros(cv::Size(left.cols + right.cols, left.rows), left.type());
        cv::Rect rect1(cv::Point2f(0, 0), left.size());
        cv::Rect rect2(cv::Point2f(left.cols, 0), left.size());
        left.copyTo(show_image(rect1));
        right.copyTo(show_image(rect2));
    }
    for (int i = 0; i < pt_size; ++i) {
        cv::Point2d pt1 = p1[i];
        cv::Point2d pt2 = p2[i] + cv::Point2f(left.cols, 0);
    //    if (mask[i] == 0) {
    //         color = cv::Scalar(0, 0, 255);
    //         if (iscontinue) continue;
    //     } else if (mask[i] == 1){
    //         color = cv::Scalar(0, 255, 0);
    //     } 
        cv::line(show_image, pt1, pt2, color);
        // cv::circle(show_image, pt1, 20, cv::Scalar(0, 255, 0));
        // cv::imshow("matched", show_image);
        // cv::waitKey();
    }
}

void drawMatchPts(
    const cv::Mat &left, const cv::Mat &right, cv::Mat &show_image,
    const std::vector<cv::KeyPoint> &p1, const std::vector<cv::KeyPoint> &p2,
    const std::vector<int> &indices, cv::Scalar color, bool update) {
    
    int pt_size = p1.size();
    if (update) {
        show_image = cv::Mat::zeros(cv::Size(left.cols + right.cols, left.rows), left.type());
        cv::Rect rect1(cv::Point2f(0, 0), left.size());
        cv::Rect rect2(cv::Point2f(left.cols, 0), left.size());
        left.copyTo(show_image(rect1));
        right.copyTo(show_image(rect2));
        cv::cvtColor(show_image, show_image, cv::COLOR_GRAY2BGR);
    }
    // std::cout << show_image.size() << "\n";
    int size = 5;
    for (int i = 0; i < pt_size; ++i) {
        if (indices[i] < 0) continue;
        cv::Point2f pt1 = p1[i].pt;
        cv::Point2f pt2 = p2[indices[i]].pt + cv::Point2f(left.cols, 0);
    //    if (mask[i] == 0) {
    //         color = cv::Scalar(0, 0, 255);
    //         if (iscontinue) continue;
    //     } else if (mask[i] == 1){
    //         color = cv::Scalar(0, 255, 0);
    //     } 
        cv::line(show_image, pt1, pt2, cv::Scalar(0, 0, 0));
        cv::rectangle(show_image, cv::Rect(pt1.x - size, pt1.y - size, size * 2, size * 2), color);
        cv::rectangle(show_image, cv::Rect(pt2.x - size, pt2.y - size, size * 2, size * 2), color);
        // cv::circle(show_image, pt1, p1[i].size, cv::Scalar(0, 255, 0));
        // cv::imshow("matched", show_image);
        // cv::waitKey();
    }
}

void drawKeyPts(
    cv::Mat &image, const std::vector<cv::KeyPoint> &pts, int size, const  cv::Scalar color) {
    for (int i = 0; i < pts.size(); ++i) {
        cv::rectangle(image, cv::Rect(pts[i].pt.x - size, pts[i].pt.y - size, size * 2, size * 2), color);
        // cv::imshow("key", image);
        // cv::waitKey();
    }
}

void drawMatchPtsInCircle(
    const cv::Mat &image, const std::vector<cv::Point2f> &pts, const std::vector<double> &radius, std::vector<int> mask, cv::Scalar color) {
    double _radius = 5;
    for (int i = 0; i < pts.size(); ++i) {
        cv::Point2d pt = pts[i] + cv::Point2f(image.cols / 2, 0);
        if (!radius.empty())
        _radius = std::abs(radius[i] * 20);
        cv::circle(image, pt, _radius, color);
    }
}




void drawCube(cv::Mat &image, const std::vector<cv::Point2f> cube, cv::Scalar color) {
    for (int i = 0; i < 4; ++i){
        int next_id = (i + 1) % 4;
        cv::line(image, cube[i], cube[next_id], cv::Scalar(0, 255, 0));
    }
    for (int i = 4; i < 8; ++i){
        int next_id = i + 1 == 8 ? 4 : i + 1;
        cv::line(image, cube[i], cube[next_id], cv::Scalar(0, 255, 0));
    }
    cv::line(image, cube[0], cube[4], cv::Scalar(0, 255, 0));
    cv::line(image, cube[1], cube[5], cv::Scalar(0, 255, 0));
    cv::line(image, cube[2], cube[6], cv::Scalar(0, 255, 0));
    cv::line(image, cube[3], cube[7], cv::Scalar(0, 255, 0));
}


}
}