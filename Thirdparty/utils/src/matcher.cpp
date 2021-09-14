#include <iostream>
#include <opencv2/opencv.hpp>
// #include <ctime>
#include <thread>
#include <mutex>
#include <chrono>
#include <fstream> 
#include <queue>
#include <limits>
// #include <direct.h>


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// #include <pcl_conversions/pcl_conversions.h>
#include "sfm.h"
#include "matcher.h"
#include "coverage_math.hpp"
#include "coverage_utils.h"
#include "computervision.h"
#include "typeconvertor.hpp"
#include "opencv_utils.h"
#include "photogrammetry.h"
#include "vo_type_utils.h"

namespace tergeo::visualodometry {

std::ostream &operator<<(std::ostream &out, Match &A) {
    out << "angle : " << A._angle << "\n"; 
    out << "_coff : " << A._coff << "\n"; 
    out << "Translate : " << A._T(0) << " " << A._T(1) << "\n"; 
    out << "R : \n" << A._R << "\n";
    return out;
}

void Matcher::load(
        const cv::Mat &K,
        // const cv::Mat &last_image, 
        const cv::Mat &current_image,
        double angle, double distance, cv::Point3f odometry
        // const cv::Mat &show_image,
        // const std::vector<cv::Mat> &descriptor_for_all,
        // const std::vector<std::vector<cv::Vec3b>> &colors_for_all,
        // const std::vector<std::vector<cv::KeyPoint>> &key_points_for_all
        ) {
    _K.release();
    _K = cv::Mat(cv::Matx33d(
		625.89325,    0.     ,  309.97394,
            0.     ,  626.76263,  256.35041,
            0.     ,    0.     ,    1.));
    // _K = cv::Mat(cv::Matx33d(
	// 	615.60882,    0.     ,  313.11862,
    //         0.     ,  615.54421,  255.36951,
    //         0.     ,    0.     ,    1.));
    // _K = cv::Mat(cv::Matx33d(
	// 	608.243,    0.     ,  324.346,
    //         0.     ,  606.882,  248.494,
    //         0.     ,    0.     ,    1.));
    _f = _K.at<double>(0, 0);
    _angle = angle;
    _distance = distance;
    _global_odometry = odometry;
    int id = 0;
    std::cout << _keyframes.empty() << "\n";
    if (!_keyframes.empty()) {//非空
        auto it = _keyframes.end();
        it--;
        // std::cout << it->first << " image \n";
        // std::cout << it->second._image.empty()<< "\n";
        _left_image_id = it->second._id;
        id = it->second._id + 1;
        _last_image = it->second._image.clone();
        image_left = _last_image.clone();
        // std::cout << "get image " << _last_image.empty() << "\n";
        _right_image_id = id;
    } else {
        _left_image_id = -1;
    }
    _keyframes[id] = KeyFrame(id, cv::Point3f(), cv::Point3f(), current_image);
    _current_image = current_image.clone();
    image_right = _current_image.clone();
    bool formatch = false;
    std::cout << "load " << "\n";
}

int Matcher::featureMatch(cv::Mat &R, cv::Mat &T,cv::Mat &matched_image) {
    _descriptor_for_all.clear();
    _descriptor_for_all.push_back(_descriptors);
    _key_points_for_all.clear();
    _key_points_for_all.push_back(_key_points);
    extract_features(_current_image, _descriptors, _colors, _key_points);
    _descriptor_for_all.push_back(_descriptors);
    _key_points_for_all.push_back(_key_points);
    _keyframe_count += 1;
    _keypoints_image.release();
    _keypoints_image = _current_image.clone();
    for (int i = 0; i < _key_points.size(); ++i) {
        cv::circle(_keypoints_image, 
        _key_points[i].pt, 
        _key_points[i].size, 
        cv::Vec3b(0, 255, 0));
    }

    // for show
    cv::imshow("keypoints", _keypoints_image);
    cv::waitKey();
    std::cout << "match\n";

    //match 
    if(_image_cout < 3) return 0;
    std::vector<cv::DMatch> matches = {};
    match_features(_descriptor_for_all[0], _descriptor_for_all[1], matches);
    std::vector<cv::Point2f> p1, p2;
    cv::Mat mask;
    get_matched_points(
        _key_points_for_all[0], 
        _key_points_for_all[1], matches, p1, p2);
    if (p1.empty() || p2.empty()) return 0;
    bool isfind_transform = find_transform(_K, p1, p2, R, T, mask);
    if (!isfind_transform) return 0;
    std::cout << "find_transform: " << find_transform << "\n";
    // std::cout << _last_image.empty() << " " << _current_image.empty() << "\n";
    cv::drawMatches(_last_image, _key_points_for_all[0], 
        _current_image, _key_points_for_all[1], matches, matched_image,
        cv::Scalar::all(-1), cv::Scalar::all(-1), 
        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    maskout_points(p1, mask);
    maskout_points(p2, mask);
    return 1;
}

//by histogram
void Matcher::calAngle() {

}


//by angle and distance
cv::Mat Matcher::getPredictionPatch(const cv::Mat image, const cv::Rect &rect) {

    return cv::Mat::zeros(image.size(), image.type());
}

void Matcher::singleMatch(const cv::Mat &image, 
    const cv::Mat &templ, cv::Mat &TMatched_image, Match match) {
    
    cv::Size dst_size;
    dst_size.width = 
        std::sqrt(image.cols * image.cols + image.rows * image.rows);
    dst_size.height = dst_size.width;
    cv::Mat tmp_dst1 = cv::Mat::zeros(dst_size, CV_8UC3);
    cv::Mat tmp_dst2 = cv::Mat::zeros(dst_size, CV_8UC3);
    cv::Point ori1, ori2;
    ori1.x = dst_size.width / 2 - image.cols / 2;
    ori1.y = dst_size.height / 2 - image.rows / 2;
    ori2.x = dst_size.width / 2 - templ.cols / 2;
    ori2.y = dst_size.height / 2 - templ.rows / 2;
    cv::Rect rect1(ori1, image.size()), rect2(ori2, templ.size());
    image.copyTo(tmp_dst1(rect1));
    templ.copyTo(tmp_dst2(rect2));
    cv::Point2d center;
    center.x = tmp_dst1.cols / 2;
    center.y = tmp_dst1.rows / 2;
    cv::Mat rotate1, rotate2, rotate3, mask;
    cv::Mat t_mask = cv::Mat::zeros(dst_size, CV_8U);
    // t_mask.setTo(255);
    t_mask(rect1).setTo(255);
    // std::cout << dst_size << "\n" << image.size() << "\n";
    // image
    cv::Mat rotate_matrix = cv::getRotationMatrix2D(center, - match._angle, 1.0);
    cv::Mat translate_martrix = cv::Mat::zeros(2, 3, CV_32FC1);
    translate_martrix.at<float>(0, 0) = 1;
    translate_martrix.at<float>(1, 1) = 1;
    translate_martrix.at<float>(0, 2) = - match._T.x();
    translate_martrix.at<float>(1, 2) = - match._T.y();
    // std::cout << rotate_matrix << "\n";
    // std::cout << match._T.x() << " " << match._T.y() << "---\n";
    cv::warpAffine(tmp_dst1, rotate1, rotate_matrix, dst_size);
    // cv::warpAffine(tmp_dst2, rotate1, rotate_matrix, dst_size);
    // cv::warpAffine(rotate1, rotate2, translate_martrix, dst_size);
    // cv::warpAffine(t_mask, mask, rotate_matrix, dst_size);
    // cv::bitwise_or(tmp_dst1, rotate2, TMatched_image, mask);
    TMatched_image = rotate1.clone();
    templ.copyTo(TMatched_image(cv::Rect(match._position, templ.size())));
    // cv::Mat result;
    // cv::matchTemplate(dst, tmpt, result, cv::TemplateMatchModes::TM_CCORR_NORMED);
    // double minVal, maxVal;
    // cv::Point minLoc, maxLoc;
    // cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
    // std::cout << maxVal << "\n";
    cv::rectangle(TMatched_image, cv::Rect(match._position, templ.size()), cv::Vec3b(0, 255, 0));
    // cv::imshow("match1", mask);
    // std::cout << match._angle << "\n";
    // cv::add
    // cv::imshow("match", rotate1);
    // std::cout << 1 << "\n";
    // cv::waitKey();
    cv::imshow("result", match._coff_mat);
    // std::cout << 2 << "\n";
    // cv::waitKey();
    cv::imshow("match", TMatched_image);
    std::cout << 3 << "\n";
    // cv::waitKey();
}


//需要改变为旋转模板，同时旋转掩膜，进行模板匹配
//需要截取真正计算了相似性计算的区域
Match Matcher::baseTemplate(const cv::Mat &image, 
        const cv::Mat &templ, cv::Mat &TMatched_image, double initial_angle) {
    
    auto cmp = [](Match l, Match r) { return l._coff < r._coff;};
    std::priority_queue<
        Match, std::vector<Match>, decltype(cmp)> match_queue(cmp);
    
    // std::cout << templ.size() << "\n";
    int cols = templ.cols;
    int rows = templ.rows;
    cv::Size dst_size;
    dst_size.width = 
        std::sqrt(image.cols * image.cols + image.rows * image.rows);
    dst_size.height = dst_size.width;
    cv::Mat tmp_tran_templ;
    cv::Mat translate_templ = cv::Mat::zeros(dst_size, CV_8UC3);
    cv::Point ori1, ori2;
    ori1.x = dst_size.width / 2 - image.cols / 2;
    ori1.y = dst_size.height / 2 - image.rows / 2;
    ori2.x = dst_size.width / 2 - templ.cols / 2;
    ori2.y = dst_size.height / 2 - templ.rows / 2;
    cv::Rect rect1(ori1, image.size()), rect2(ori2, templ.size());
    // std::cout << "test\n"; 
    templ.copyTo(translate_templ(rect2));
    tmp_tran_templ = translate_templ.clone();
    // cv::cvtColor(rotate_templ, rotate_templ, CV_BGR2GRAY);
    cv::Mat match_templ;
    match_templ = templ.clone();
    // cv::cvtColor(templ, match_templ, CV_BGR2GRAY);
    
    for (double i = -5; i < 5; i=i + 1) {
        cv::Mat match_image, rotate_template, image_plus, mask;
        double template_angle = - initial_angle + i;
        cv::Mat tmp_match_image = cv::Mat::zeros(dst_size, CV_8UC3);
        image.copyTo(tmp_match_image(rect1));
        
        cv::Point2d center;
        center.x = tmp_match_image.cols / 2;
        center.y = tmp_match_image.rows / 2;
        cv::Mat rotate_matrix = cv::getRotationMatrix2D(center, template_angle, 1.0);
        cv::warpAffine(tmp_match_image, match_image, rotate_matrix, dst_size);
        // std::cout << match_image.size() << "\n";
        // cv::cvtColor(match_image, match_image, CV_BGR2GRAY);
        cv::Mat result;
        // std::cout << "match time: " << ros::Time::now().toSec() - _time.toSec() << "\n";    
        // cv::matchTemplate(match_image, match_templ, result, cv::TemplateMatchModes::TM_CCORR_NORMED);
        cv::matchTemplate(match_image, match_templ, result, cv::TemplateMatchModes::TM_CCOEFF_NORMED);
        // std::cout << "match once time: " << ros::Time::now().toSec() - _time.toSec() << "\n";    
        
        // int inner_x = match_templ.cols / 2 - match_image.cols / 2 + result.cols / 2;
        // int inner_y = match_templ.rows / 2 - match_image.rows / 2 + result.rows / 2;
        // cv::Rect inner_rect(inner_x, inner_y, match_image.cols - match_templ.cols, match_image.rows - match_templ.rows); 
        // // cv::cvtColor(submatch._coff_mat, color1, cv::COLOR_GRAY2BGR, 3); 
        // result = result(inner_rect);
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);

        //it means the rotate right image -template_angle at the center, and translate left_center pixel
        //will transform the right to left  
        Match match(Match(maxLoc, - template_angle, maxVal));
        cv::Point2f left_center = match._position + cv::Point2f(cols / 2, rows / 2);
        left_center = left_center - cv::Point2f(match_image.cols / 2, match_image.rows / 2);
        match.setTR( - match._angle / 180 * M_PI, left_center.x, left_center.y);
        match._coff_mat = result;
        match_queue.push(match);
        
        //for check
        cv::Mat translate_martrix = cv::Mat::zeros(2, 3, CV_32FC1);
        translate_martrix.at<float>(0, 0) = 1;
        translate_martrix.at<float>(1, 1) = 1;
        translate_martrix.at<float>(0, 2) = match._T.x();
        translate_martrix.at<float>(1, 2) = match._T.y();
        cv::warpAffine(tmp_tran_templ, translate_templ, translate_martrix, dst_size);
        std::cout << left_center << " " << match._T.x() << " " << match._T.y() << "\n";
        
        cv::rectangle(match_image, cv::Rect(maxLoc, match_templ.size()), cv::Vec3b(0, 255, 0));
        // cv::bitwise_or(match_image, translate_templ, image_plus);
        image_plus = match_image.clone();
        templ.copyTo(image_plus(cv::Rect(maxLoc, match_templ.size())));
        cv::imshow("result", result);
        cv::imshow("match", match_image);
        cv::imshow("image_plus", image_plus);
        std::cout<< template_angle << " " << maxVal << " " << maxLoc << "\n";
        // cv::waitKey();
    }
    Match t_match = match_queue.top(); 
    // match = match_queue.top();
    std::cout << "match coff: " << match_queue.top()._coff << " angle " << match_queue.top()._angle << "\n";
    //origin of the following coordinate is in the center of the image
    singleMatch(image, templ, TMatched_image, match_queue.top());
    return t_match;
}

cv::Mat Matcher::getInnerPatch(const cv::Mat &image, const cv::Mat &templ) {
    cv::Mat patch;
    int cols = templ.cols;
    int rows = templ.rows;
    double maxsize = std::min(templ.rows, templ.cols);
    double cos = 0.8 * cols / std::sqrt(cols * cols + rows * rows); 
    double sin = 0.8 * rows / std::sqrt(cols * cols + rows * rows); 
    cv::Size templ_size;
    templ_size.width = maxsize * cos;
    templ_size.height = maxsize * sin;
    cv::Point pt;
    pt.x = (cols - templ_size.width) / 2;
    pt.y = (rows - templ_size.height) / 2;
    return templ(cv::Rect(pt, templ_size));

}


void Matcher::getMatchedPoints(double x, double y, double &out_x, double &out_y) {
    int h_cols = _last_image.cols / 2;
    int h_rows = _last_image.rows / 2;
    Eigen::Vector2d in_XY(x - h_cols, y - h_rows);
    Eigen::Vector2d out_xy = match._R * in_XY; 
    out_x = out_xy(0) + match._T(0) + h_cols;
    out_y = out_xy(1) + match._T(1) + h_rows;
    // cv::circle(_show_image, cv::Point2d(out_x, out_y), 5, red);
    // cv::circle(_last_image, cv::Point2d(x, y), 5, red);
    // cv::imshow("rect",  _last_image);
    // cv::imshow("rotate", _show_image);
    // cv::waitKey();

}


bool Matcher::exceedImage(double x, double y, const cv::Mat &image, bool translate) {
    double out_x, out_y;
    if (translate) {
        getMatchedPoints(x, y, out_x, out_y);
    } else {
        out_x = x;
        out_y = y;
    }
    if (out_x < 0 || out_x > image.cols || 
        out_y < 0 || out_y > image.rows) return true; 
    return false;
}


bool Matcher::exceedImage(const cv::Rect &rect, const cv::Mat &image, bool translate) {
    
    double minX, maxX, minY, maxY; 
    double out_x, out_y;
    std::vector<cv::Point2d> pts = {};
    // std::vector<cv::Point2d> pts1 = {};
    pts.push_back(rect.tl());
    pts.push_back(cv::Point2d(rect.x + rect.width, rect.y));
    pts.push_back(cv::Point2d(rect.x + rect.width, rect.y + rect.height));
    pts.push_back(cv::Point2d(rect.x, rect.y + rect.height));
    // getMatchedPoints(389, 334, out_x, out_y);
    // getMatchedPoints(485, 265, out_x, out_y);
    for (int i = 0; i < 4; ++i) {
        const cv::Point2d &pt = pts[i];
        if (translate) {
            getMatchedPoints(pt.x, pt.y, out_x, out_y);
        } else {
            out_y = pt.y;
            out_x = pt.x;
        }
        if (out_x < 0 || out_x > image.cols || 
            out_y < 0 || out_y > image.rows) return true; 
        // show
        // pts1.push_back(cv::Point2d(out_x, out_y));
    }
    //show
    // _show_image = _current_image.clone();
    // tergeo::planning::algorithm::draw<cv::Point2d>(image, pts1, green);
    // cv::rectangle(_last_image, rect, green);
    // cv::waitKey();
    return false;
}

void Matcher::extractPoints(const cv::Mat &image, FeatureLQueue &max_feature_points, 
    std::map<int, FeaturePoint> &feature_points, int step) {
    
    int feature_count = 0;
    int blockSize = 7;
	int ksize = 5;
	double k = 0.04;
    cv::Mat gray;
    cv::Mat harris_result, norm_harris_result, normScaleDst;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    cv::cornerHarris(gray, harris_result, blockSize, ksize, k);
    cv::Mat binary_harris;
    cv::threshold(harris_result, binary_harris, 0, 1, cv::ThresholdTypes::THRESH_BINARY);
    cv::namedWindow("binary_harris", CV_WINDOW_NORMAL);
    cv::imshow("binary_harris", binary_harris);
    
    //制作掩膜
    cv::Mat meanMat = binary_harris.clone(), stddevMat = binary_harris.clone();
    int block = 20;
	int pwidth = binary_harris.cols / block;
	int pheight = binary_harris.rows / block;
	cv::Size rsize = cv::Size(block, block);
    cv::Scalar mean, stddev;
    cv::Mat mask = cv::Mat::zeros(binary_harris.size(), CV_8U);
    mask.setTo(1);
    for (int i = 0; i < pwidth; ++i) {
		for (int j = 0; j < pheight; ++j) {
			cv::Rect prect(cv::Rect(cv::Point(i * block, j * block), rsize));
			cv::Mat pmat = binary_harris(prect);
			cv::meanStdDev(pmat, mean, stddev);
			meanMat(prect).setTo(mean);
			stddevMat(prect).setTo(stddev);
            double aver_mean = (mean[0] + mean[1] + mean[1]) / 3;
            double aver_stddev = (stddev[0] + stddev[1] + stddev[1]) / 3;
            
            if (aver_stddev < 0.01 && aver_mean < 0.5) {
                mask(prect).setTo(255);
            }
            
			// std::cout << pmat << "\n";
			// std::cout << "i: " << i << " j: " << j 
			// << " \nmean:\n" << mean << " \nstddev:\n" << stddev << "\n";
		}
	}
    // cv::imshow("mask", mask);
    // cv::imshow("meanMat", meanMat);
    // cv::imshow("stddevMat", stddevMat);
    // cv::waitKey();

    // cv::waitKey();
    FeatureQueue point_queue0(cmp);
    int step0 = _current_image.cols / 8;
    int filter_size = blockSize;
    filter_size = image.cols / 24;
    int max_feature_point_count = 10;
    int visited_length = harris_result.cols * harris_result.rows;
    bool visited[visited_length];
    memset(visited, false, sizeof(bool) * visited_length);
    int count_skip = 0;
    // std::cout << harris_result << "\n";
    for (int row = blockSize; row  < image.rows - blockSize; ++row ) {
        for (int col = blockSize; col < image.cols - blockSize; ++col) {
            float R = harris_result.at<float>(row, col);
            if (mask.at<uchar>(row, col) == 255) {
                count_skip += 1;
                continue;//去除平面点
            } 
            if (max_feature_points.size() < max_feature_point_count) {
                max_feature_points.push(FeaturePoint(0, R, cv::Point2d(col, row)));
            } else if(max_feature_points.top()._value < R) {
                max_feature_points.pop();
                max_feature_points.push(FeaturePoint(0, R, cv::Point2d(col, row)));
            }
            bool maxInNeighbour = true;
            int rcid = row * harris_result.cols + col;
            //保存了上一次小于中心row，col的点，这些点一定不是局部极大值点
            if (visited[rcid]) continue;
            for (int i = - filter_size / 2; i < filter_size / 2; ++i) {// row
                for (int j = - filter_size / 2; j < filter_size / 2; ++j) {// col
                    int irow = i + row;
                    int jcol = j + col;
                    if (irow < 0 || irow >= harris_result.rows ||
                        jcol < 0 || jcol >= harris_result.cols) continue;
                    int ijindex = irow * harris_result.cols + jcol;
                    if (harris_result.at<float>(irow, jcol) > R) {
                        maxInNeighbour = false;
                        break;
                    }
                    //这些点小于等于中心点，已经访问过，下一次不必再访问
                    visited[ijindex] = true;
                }
                if (!maxInNeighbour) break;
            }
            if (maxInNeighbour) {
                feature_points[feature_count] = FeaturePoint(feature_count, R, cv::Point2d(col, row));
                feature_count = feature_count + 1;
                // cv::rectangle(image_right, 
                //     cv::Rect2d(col - filter_size / 2, 
                //     row - filter_size / 2, filter_size, filter_size),
                //     green);
                cv::circle(image_right, cv::Point2d(col, row), 5, red);
                cv::imshow("right", image_right);
            }
        }
    }
    cv::waitKey();
    // std::cout << "done\n";
    std::cout << count_skip << "\n";
}

bool Matcher::getMatchPoints(
    const Match &match,
    std::map<int, FeaturePoint> &leftPts,
    std::map<int, FeaturePoint> &rightPts,
    MatchQueue &match_queue,
    MatchPair &matchpair) {
    
    // match
    // std::map<int, std::vector<FeaturePoint>> key_points = {};
    // FeatureLQueue left_max_feature_points(cmpl), right_max_feature_points(cmpl);
    image_left = _last_image.clone();
    image_right = _current_image.clone();
    // cv::namedWindow("left", cv::WINDOW_NORMAL);
    cv::namedWindow("right", cv::WINDOW_NORMAL);
    _show_image.release();
    _show_image = cv::Mat::zeros(
        std::max(image_left.rows, image_right.rows), image_left.cols + image_right.cols, image_left.type());
    image_left.copyTo(_show_image(cv::Rect(cv::Point(0, 0), image_left.size())));
    image_right.copyTo(_show_image(cv::Rect(cv::Point(image_left.cols, 0), image_right.size())));
    // store the best five-points and cal the E-martrix from it
    // then predict the precise pixel coordinate to get more accurate
    // std::cout <<" test2\n";
    // matched points
    FeaturePoint matched_point;
    int best_match_num = 8;

    //draw right
    for (auto rit = rightPts.begin(); rit != rightPts.end(); ++rit) {
        const FeaturePoint &pt2 = rit->second;
        cv::circle(_show_image, cv::Point2d(pt2._pt.x + image_left.cols, pt2._pt.y), 5, green);
    }
    for (auto lit = leftPts.begin(); lit != leftPts.end(); ++lit) {
        const FeaturePoint &pt1 = lit->second;
        double out_x, out_y;
        getMatchedPoints(pt1._pt.x, pt1._pt.y, out_x, out_y);
        cv::circle(_show_image, pt1._pt, 5, green);
        cv::circle(_show_image, cv::Point2d(out_x + image_left.cols, out_y), 5, red);
        // search nearst pt in right 
        double dis = std::numeric_limits<double>::max();
        for (auto rit = rightPts.begin(); rit != rightPts.end(); ++rit) {
            const FeaturePoint &pt2 = rit->second;
            double dx = out_x - pt2._pt.x;
            double dy = out_y - pt2._pt.y;
            double tmp_dis = dx * dx + dy * dy;
            if (dis > tmp_dis) {
                // std::cout << "one\n";
                dis = tmp_dis;
                matched_point  = pt2;
            }
        }
        if(match_queue.size() < best_match_num) {
            match_queue.push(MatchPoint(pt1, matched_point, dis));
        } else if(match_queue.size() == 5 && match_queue.top()._distance > dis) {
            match_queue.pop();
            match_queue.push(MatchPoint(pt1, matched_point, dis));
        }
        // std::cout << std::sqrt(dis) << "\n";
        if (dis < 36) {
        cv::Vec3b color(128, rand() % 255, rand() % 255);
        cv::line(_show_image, pt1._pt, 
            cv::Point2d(matched_point._pt.x + image_left.cols, 
                matched_point._pt.y), color);
            matchpair._pairs.push_back(std::make_pair(pt1._id, matched_point._id));
            // match_points.push_back(MatchPoint(pt1, matched_point, dis));
        }
    }
    // get cv::Point
    std::cout << match_queue.size() << " match size\n";
    // while (!match_queue.empty()) {
    //     p1.push_back(match_queue.top()._p1._pt);
    //     p2.push_back(match_queue.top()._p2._pt);
    //     match_queue.pop();
    // }
    cv::namedWindow("matched", cv::WINDOW_NORMAL);
    cv::imshow("matched_pair", _show_image);
    cv::waitKey(1);
    if (matchpair._pairs.size() < 5) {
        std::cout << "too few feature points\n";
        return false;
    } else {
        return true;
    }
}

bool Matcher::getRT(const std::vector<cv::Point2f> &p1,
        const std::vector<cv::Point2f> &p2, cv::Mat &R, 
        cv::Mat &T, cv::Mat &RT, cv::Mat &mask, int method, int mode) {
    std::vector<cv::Point3f> D3pts = {};
    if(method == 0) {//essential matrix
        return getRTByEssential(_K, p1, p2, R, T, mask);
    } else if (method == 1){//homography
        return getRTByHomography(_K, p1, p2, D3pts, R, T, RT, mask, mode);
    }
}


std::vector<cv::KeyPoint> Matcher::getCVKeyPts(int image_id, std::vector<int> mask) {
    const auto &featuresPts = _keyframes[image_id]._featurePts;
    int pt_size = featuresPts.size();
    std::vector<cv::KeyPoint> keyPts = {};
    for (auto it = featuresPts.begin(); it != featuresPts.end(); it++) {
        keyPts.push_back(cv::KeyPoint(it->second._pt, 1));
    }
    return keyPts;
}
int Matcher::templateMatch(cv::Mat &R, cv::Mat &T, 
    cv::Mat &TMatched_image, Match &result) {
    // _last_image = cv::imread("/home/wz/VO-LOAM/data/visual_odometry/1621488794.668933/left_image.png");
    // _current_image = cv::imread("/home/wz/VO-LOAM/data/visual_odometry/1621488794.668933/right_image.png");
    _show_image = _current_image.clone();
    std::cout << "_image_cout: " << _image_cout << "\n";
    std::cout << _current_image.empty() << " " << _last_image.empty() << "\n";
    //先提取特征点，再最新的_current_image提取
    //如果有两张影像就执行模板匹配，再对特征点匹配
    //
    int step0 = _current_image.cols / 8;
    int width = step0;
    int height = step0;
    int points_size = 5;
    FeatureLQueue left_max_feature_points(cmpl), right_max_feature_points(cmpl);
    std::map<int, FeaturePoint> right_feature_points;
    // std::cout << "extractPoints\n";
    std::cout << "before extract: " << ros::Time::now().toSec() - _time.toSec() << "\n";    
    extractPoints(_current_image, 
        right_max_feature_points, 
        right_feature_points, step0);
    return 1;
    // std::cout << "extractPoints done\n";
    std::cout << "extract time: " << ros::Time::now().toSec() - _time.toSec() << "\n";    
    // int size = featurePtsMap.size();
    // int image_id = _keyframes.end()
    _keyframes[_right_image_id]._featurePts = right_feature_points;
    // if(isinitilized())
    if (_image_cout == 0 && _keyframes.size() == 1) {
        std::cout << "extract feature points without template\n";
        return 0;
    }
    //模板匹配
    calAngle();
    //get distance
    // std::cout << "getInnerPatch\n";
    cv::Mat inner_patch = getInnerPatch(_current_image, _last_image);
    // std::cout << "getInnerPatch time: " << ros::Time::now().toSec() - _time.toSec() << "\n";    
    // std::cout << "baseTemplate\n";
    match = baseTemplate(_current_image, inner_patch, TMatched_image, _angle / M_PI * 180);
    // std::cout << "Template time: " << ros::Time::now().toSec() - _time.toSec() << "\n";    
    // std::cout << "baseTemplate done\n";
    //模板匹配完毕，开始匹配同名点
    //参数是两幅影像的R，T和两组特征点
    MatchQueue match_queue(cmpm);
    // std::vector<cv::Point2f> p1, p2;
    if(matchPairs.empty()) {
        _match_pairId = 0;
    } else {
        auto it = matchPairs.end();
        it--;
        _match_pairId = it->second._id + 1;
    }
    //第二次进入，有了两张影像，取出左片特征点
    std::map<int, FeaturePoint> &left_feature_points = _keyframes[_left_image_id]._featurePts;
    MatchPair matchpair(_match_pairId, _left_image_id, _right_image_id);
    bool matched = getMatchPoints(match,
        left_feature_points,
        right_feature_points,
        match_queue, matchpair);
    matchPairs[_match_pairId] = matchpair;
    if (!matched) {
        return 0;
    }
    // get match points in matchpair
    int left_id = matchPairs[_match_pairId]._left_image_id;
    int right_id = matchPairs[_match_pairId]._right_image_id;
    std::cout << "size: " << left_id << " " << _keyframes[left_id]._featurePts.size() << "\n";
    std::cout << "size: " << right_id << " " << _keyframes[right_id]._featurePts.size() << "\n";
    std::cout << "size: " << matchPairs[_match_pairId]._pairs.size() << "\n";
    // std::vector<cv::Point2f> pts1 = {}, pts2 = {};
    // getMatchCVPts(matchpair, pts1, pts2);
    result = match;
    std::cout << "match done\n";
    //根据模板匹配结果得到精匹配特征点,并且删除不符合的点
    filtMatchPoints(_match_pairId, 0);
    //由sift算子尺度提供候选harris区域,这样去掉平面点
    //sift尺度范围里找左右harris特征点,
    //根据1.单应矩阵; 2.本征矩阵RT求得相对位姿; 3.solvePnP求得的转换关系
    //将左片转换到右片,计算模板响应值
    //右片转换到左片,计算响应值
    // filtMatchPoints(_match_pairId, 1);

    //偶尔出现内存分配失败的错误
	// cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create(20, 5, 0.005, 1.6);
	// cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create(20, 3, 0.005, 1.6);
	
    //test sift
    // cv::Mat image = _last_image.clone(), oimage = _last_image.clone();
    // std::vector<cv::KeyPoint> key_points = getCVKeyPts(matchPairs[_match_pairId]._left_image_id);
    // cv::Mat descriptors;
    // cv::drawKeypoints(image, key_points, oimage, green);
    // cv::imshow("keypoints_before", oimage);
    // cv::waitKey();
    // sift->detectAndCompute(image, cv::noArray(), key_points, descriptors, true);
    // for show
    // cv::imshow("keypoints_after", oimage);
    // cv::waitKey();

    double pixel_distance = std::sqrt(match._T(0) * match._T(0) + match._T(1) * match._T(1));  
    if (pixel_distance < 10) {
        std::cout << _distance << "\n";
        std::cout << "too short distance, don't get refined matches\n";
        return 1;
    }
    //寻找极值点
    auto &matchpair_ = matchPairs[_match_pairId];
    std::vector<PairInfo> _pair_info_vec;
    for (int i = 0; i < matchpair_._pair_info_vec.size(); i++) {
        if (matchpair_._pair_info_vec[i].stddev_dif <=0 || 
            matchpair_._pair_info_vec[i].stddev_ori <=0) continue;
        _pair_info_vec.push_back(matchpair_._pair_info_vec[i]);
    }
    // _pair_info_vec.erase(std::remove_if(_pair_info_vec.begin(), _pair_info_vec.end(), [](PairInfo p)
    //     {return (p.stddev_dif <= 0 && p.stddev_ori <= 0);}), _pair_info_vec.end());
    // std::remove_if(_pair_info_vec.begin(), _pair_info_vec.end(), [](PairInfo p)
    //     {return (p.stddev_dif <= 0 && p.stddev_ori <= 0);});
    double min_ori = 1, max_ori = 0, min_dif = 1, max_dif = 0;
    std::sort(_pair_info_vec.begin(), _pair_info_vec.end(), SortOriStddev());
    matchpair_.min_ori = _pair_info_vec.front().stddev_ori;
    matchpair_.max_ori = _pair_info_vec.back().stddev_ori;
    std::sort(_pair_info_vec.begin(), _pair_info_vec.end(), SortDifStddev());
    matchpair_.min_dif = _pair_info_vec.front().stddev_dif;
    matchpair_.max_dif = _pair_info_vec.back().stddev_dif;
    //如果找到了单应变换/本征矩阵，使用单应变换/本征矩阵搜索寻找特征点
    //如果没有找到，就用模板匹配寻找对应点
    cv::Mat mask;
    std::vector<cv::Point2f> pts1 = {}, pts2 = {};
    std::vector<int> query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_TEMPLATE);
    getMatchedCVPts(matchPairs[_match_pairId], _keyframes[_left_image_id], _keyframes[_right_image_id], pts1, pts2, query);
    //本征矩阵是模板匹配后的点，可以直接用全部特征点，用RANSAC
    cv::namedWindow("matched_template", cv::WINDOW_NORMAL);
    cv::imshow("show_image", TMatched_image);
    if (pts1.size() != pts2.size() || pts1.size() < 4) {
        std::cout << "too few match points, use SIFT or Template Match\n";
        //鉴于特征点太少,说明之前筛选太严格,需要将极值确定的区间扩大
        matchpair_.min_ori = matchpair_.min_ori * 0.8;
        matchpair_.max_dif = matchpair_.max_dif * 1.2;
        wrapGetMatchedPointsByTemplateL2R(0);
    } else {
        bool isgetRT = getRT(pts1, pts2, _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, RT, mask, 1, 1);
        if (isgetRT) {
            wrapGetMatchedPointsByTemplateL2R(1);
        } else {//仍然使用模板匹配
            matchpair_.min_ori = matchpair_.min_ori * 0.8;
            matchpair_.max_dif = matchpair_.max_dif * 1.2;
            wrapGetMatchedPointsByTemplateL2R(0);
        }
    }
    std::vector<cv::Point2f> p1, p2;
    query = matchpair_.getQuery(QueryPtType::QUERY_BY_DEPRECATED);
    getMatchedCVPts(matchpair_, _keyframes[_left_image_id], _keyframes[_right_image_id], p1, p2, query);
    drawMatchPts(_last_image, _current_image, _show_image, p1, p2, red);
    query = matchpair_.getQuery(QueryPtType::QUERY_BY_TEMPLATE);
    getMatchedCVPts(matchpair_, _keyframes[_left_image_id], _keyframes[_right_image_id], p1, p2, query);
    drawMatchPts(_last_image, _current_image, _show_image, p1, p2, green, false);
    cv::imshow("matched_template", _show_image);
    cv::waitKey();
    return 1;
}

void Matcher::wrapGetMatchedPointsByTemplateL2R(int predict_mode) {
    auto &matchpair_ = matchPairs[_match_pairId];
    int left_id = matchpair_._left_image_id;
    int right_id = matchpair_._right_image_id;
    auto &lfeaturePts = _keyframes[left_id]._featurePts;
    auto &rfeaturePts = _keyframes[right_id]._featurePts;
    std::vector<int> mask_left(lfeaturePts.size(), 1);
    std::vector<int> mask_right(rfeaturePts.size(), 1);
    //留下上一步未匹配上的点
    for (int i = 0; i < matchpair_._pairs.size(); ++i) {
        int left_id, right_id;
        // if (mask_template[i] == 1 || matchpair._deprecated_pts[i] == 0) {
            left_id = matchpair_._pairs[i].first;
            right_id = matchpair_._pairs[i].second;
            mask_left[left_id] = 0;
            mask_right[right_id] = 0;
        // }
    }
    getMatchedPointsByTemplateL2R(matchpair_, lfeaturePts, rfeaturePts, _last_image, _current_image, mask_left, 0, predict_mode);
    double push_length = rfeaturePts.size() - mask_right.size();
    for (int i = 0; i < push_length; ++i) {
        mask_right.push_back(0);
    }
    getMatchedPointsByTemplateL2R(matchpair_, rfeaturePts, lfeaturePts, _current_image, _last_image, mask_right, 1, predict_mode);
}

void Matcher::getMatchedPointsByTemplateL2R(MatchPair &matchpair,
        std::map<int, FeaturePoint> &pts1, std::map<int, FeaturePoint> &pts2,
        const cv::Mat &image1, const cv::Mat &image2, std::vector<int> mask, 
        int mode, int predict_mode) {
    cv::Mat TMatched_image;
    cv::Rect2d rect1, rect2;
    //左片当模板,左片是真值,右片是模板对应值
    // cv::namedWindow("rotate2", cv::WINDOW_NORMAL);
    cv::Mat result_image;
    std::vector<double> error0 = {};
    
    auto cmpPairInfo1 = [](PairInfoPair l, PairInfoPair r){return l.second.stddev_dif > r.second.stddev_dif;};
    auto cmpPairInfo2 = [](PairInfoPair l, PairInfoPair r){return l.second.stddev_ori < r.second.stddev_ori;};
    std::priority_queue<PairInfoPair, std::vector<PairInfoPair>, decltype(cmpPairInfo1)> simi_mat1(cmpPairInfo1);
    std::priority_queue<PairInfoPair, std::vector<PairInfoPair>, decltype(cmpPairInfo2)> simi_mat2(cmpPairInfo2);
    int MPID = matchpair._pairs.size();
    auto p2It = pts2.end();
    p2It--;
    int p2Id = p2It->first + 1;
    cv::Mat show_image = _show_image.clone();

    for (auto it = pts1.begin(); it != pts1.end(); it++) {
        int id = it->first;
        if (mask[id] == 0)continue;
        const cv::Point2d &pt1 = it->second._pt;
        cv::Point2d _tpt2;
        if (predict_mode == 0) {
            double out_x, out_y;
            getMatchedPoints(pt1.x, pt1.y, out_x, out_y);
            _tpt2 = cv::Point2d(out_x, out_y);
        } else if(predict_mode == 1) {
            _tpt2 = predicteHPoint(pt1, RT, mode);
        }
        const cv::Point2d &pt2 = _tpt2;
        std::cout << RT << "\n";
        // show_image = _show_image.clone();
        if (mode == 0) {
            rect1 = cv::Rect2d(cv::Point2d(pt1.x - 20, pt1.y - 20) , size40);
            rect2 = cv::Rect2d(cv::Point2d(pt2.x + image1.cols - 30, pt2.y - 30) , size60);
            // cv::rectangle(show_image, rect1, red);
            // cv::rectangle(show_image, rect2, red);
            rect2.x = rect2.x - image1.cols;
            // cv::circle(show_image, cv::Point2d(pt2.x + image1.cols, pt2.y), 5, blue);
            // cv::line(show_image, pt1, 
            //     cv::Point2d(pt2.x + image_left.cols, pt2.y), blue);
        } else {
            rect1 = cv::Rect2d(cv::Point2d(pt1.x - 20 + image1.cols, pt1.y - 20) , size40);
            rect2 = cv::Rect2d(cv::Point2d(pt2.x - 30, pt2.y - 30) , size60);
            // cv::rectangle(show_image, rect1, red);
            // cv::rectangle(show_image, rect2, red);
            rect1.x = rect1.x - image1.cols;
            // cv::circle(show_image, pt2, 5, blue);
            // cv::line(show_image, cv::Point2d(pt1.x + image_left.cols, pt1.y), 
            //     pt2, blue);
        }
        // cv::imshow("matched", show_image);
        // cv::waitKey();
        // continue;
        // submatch
        if (exceedImage(rect1, image1, false) || 
            exceedImage(rect2, image2, false)) {
            error0.push_back(100);
            continue;
        }
        cv::Mat match_image = image2(rect2);//right
        cv::Mat templ_image = image1(rect1);//left
        // std::cout << match_image.size() << " " << templ_image.size() << "\n";
        submatch = baseTemplate(match_image, templ_image, TMatched_image, 0);
        std::cout << submatch._T << "\n";
        // std::cout << pts1[i] << "\n" << pts2[i] << "\n";
        cv::Size dst_size;
        //translate the left to the right
        double out_x, out_y;
        int h_cols = 20;
        int h_rows = 20;
        Eigen::Vector2d in_XY(0, 0);//原点转换到右边
        Eigen::Vector2d out_xy = submatch._R * in_XY; 
        out_x = out_xy(0) + submatch._T(0) + pt2.x;
        out_y = out_xy(1) + submatch._T(1) + pt2.y;
        // pts3.push_back(cv::Point2f(out_x, out_y));
        //在后续子模板匹配中，距离应该怎么设置？
        double dx = out_x - pt2.x;
        double dy = out_y - pt2.y;
        double dis_in_HT = std::sqrt(dx * dx + dy * dy);
        error0.push_back(dis_in_HT);
        if (predict_mode == 0 && dis_in_HT > 10) {//模板匹配
            continue;
        }
        if (predict_mode == 1 && dis_in_HT > 2) {//单应矩阵
            continue;
        }
        
        
        //for debug
        cv::Mat color, color1, color2;
        int inner_x = 3 + rect1.width / 2 - rect2.width / 2 + submatch._coff_mat.cols / 2;
        int inner_y = 3 + rect1.height / 2 - rect2.height / 2 + submatch._coff_mat.rows / 2;
        // cv::cvtColor(submatch._coff_mat, color1, cv::COLOR_GRAY2BGR, 3); 
        cv::Rect inner_rect(inner_x, inner_y, rect2.width - rect1.width - 6, rect2.height - rect1.height - 6); 
        if (predict_mode == 1) {
            // inner_x = 3 + rect1.width / 2 - rect2.width / 2 + submatch._coff_mat.cols / 2;
            // inner_y = 3 + rect1.height / 2 - rect2.height / 2 + submatch._coff_mat.rows / 2;
            submatch._coff_mat = submatch._coff_mat(inner_rect);
        } else if(predict_mode == 0) {//模板匹配直接定位到中心，距离阈值也应该设置更大一点
            // inner_x = inner_x + submatch._T(0);
            // inner_y = inner_y + submatch._T(1);
        }
        // cv::imshow("inner_rect", submatch._coff_mat(inner_rect));
        // cv::waitKey();
        // continue;
        const cv::Mat &coff_mat0 = submatch._coff_mat;
        double sizeGS = 5;
        cv::Mat coff_mat;
        cv::GaussianBlur(coff_mat0, coff_mat, cv::Size(sizeGS, sizeGS), sizeGS);
        cv::Mat rotate90;
        cv::Mat rotate_matrix = cv::getRotationMatrix2D(cv::Point(coff_mat.cols / 2, coff_mat.rows / 2), 90, 1);
        cv::warpAffine(coff_mat, rotate90, rotate_matrix, coff_mat.size());
        cv::Mat mul_ori = coff_mat.mul(coff_mat);
        cv::Mat mul_90 = coff_mat.mul(rotate90);
        cv::Mat diff_image = mul_90 - mul_ori;
        std::cout << " ttt: " << coff_mat.channels() << "\n";
        // cv::imshow("mul_ori", mul_ori);
        // cv::imshow("mul_90", mul_90);
        // cv::imshow("diff_image", diff_image);
        cv::Scalar mean, stddev;
        double stddev_ori, stddev_dif;
        cv::meanStdDev(coff_mat, mean, stddev);
        stddev_ori = stddev[0];
        cv::meanStdDev(diff_image, mean, stddev);
        stddev_dif = stddev[0];
        // cv::meanStdDev(coff_mat, mean, stddev);
        // cv::waitKey();
        cv::Mat tmp_dst2 = cv::Mat::zeros(show_image.size(), CV_8UC3);
        cv::Point ori1, ori2;
        if (mode == 0) {
            ori2.x = pt1.x - submatch._coff_mat.cols / 2;
            ori2.y = pt1.y - submatch._coff_mat.rows / 2;
        } else {
            ori2.x = pt1.x - submatch._coff_mat.cols / 2 + image1.cols;
            ori2.y = pt1.y - submatch._coff_mat.rows / 2;
        }
        cv::Rect rect3(ori2, submatch._coff_mat.size());
        // templ_image.copyTo(tmp_dst2(rect1));
        submatch._coff_mat.convertTo(color1, CV_32FC3);
        // color1 = (color1 - 0.5) * 2;
        color1.convertTo(color, CV_8UC3, 255.0);
        cv::cvtColor(color, color2, cv::COLOR_GRAY2BGR);
        std::cout << submatch._coff_mat.channels() << " " << templ_image.type() << " "<< color.type() << " " << tmp_dst2.type() <<"\n";
        if (exceedImage(rect3, tmp_dst2, false)) continue;
        color2.copyTo(tmp_dst2(rect3));
        cv::Mat maskt = tmp_dst2.clone();
        maskt.setTo(cv::Vec3i(0, 0, 0));
        maskt(rect3) = cv::Vec3i(255, 255, 255);
        // tmp_dst2(rect3) = color;
        cv::imshow("math", tmp_dst2);
        // std::cout << submatch._coff_mat << "\n\n";
        // std::cout << color << "\n\n";
        // cv::waitKey();

        cv::Mat rotate1, rotate2, rotate3;
        cv::Mat translate_martrix;
        cv::Mat t_mask1, t_mask2, t_mask3;
        if (mode == 0) {
            rotate_matrix = cv::getRotationMatrix2D(pt1, match._angle, 1.0);
            translate_martrix = cv::Mat(cv::Matx23d(
                1, 0, submatch._T.x() + pt2.x - pt1.x + _last_image.cols,
                0, 1, submatch._T.y() + pt2.y - pt1.y));
            cv::warpAffine(tmp_dst2, rotate1, rotate_matrix, dst_size);
            cv::warpAffine(rotate1, rotate2, translate_martrix, dst_size);
            cv::warpAffine(maskt, t_mask1, rotate_matrix, dst_size);
            cv::warpAffine(t_mask1, t_mask2, translate_martrix, dst_size);
        } else {
            translate_martrix = cv::Mat(cv::Matx23d(
                1, 0, submatch._T.x() + pt2.x - pt1.x - _last_image.cols,
                0, 1, submatch._T.y() + pt2.y - pt1.y));
            rotate_matrix = cv::getRotationMatrix2D(pt1, match._angle, 1.0);
            cv::warpAffine(tmp_dst2, rotate1, rotate_matrix, dst_size);
            cv::warpAffine(rotate1, rotate2, translate_martrix, dst_size);
            cv::warpAffine(maskt, t_mask1, rotate_matrix, dst_size);
            cv::warpAffine(t_mask1, t_mask2, translate_martrix, dst_size);
        }
        result_image = show_image.clone();
        cv::bitwise_not(t_mask2, t_mask3);
        cv::bitwise_and(result_image, t_mask3, rotate1, cv::noArray());
        cv::bitwise_or(rotate1, rotate2, show_image, cv::noArray());
        cv::imshow("rotate2", rotate2);
        // image.copyTo(tmp_dst1(rect1));
        // cv::imshow("gray", rotate1);
        // cv::imshow("math", rotate2);
        cv::imshow("matched", show_image);
        // cv::imshow("rotate2", result_image);
        std::cout << "stddev: " << stddev_ori << " " << stddev_dif << " " << stddev_ori / stddev_dif << "\n";
        std::cout << dis_in_HT << "\n";
        if (predict_mode == 0) {
            cv::waitKey();
            int ttt = 1;
        }
        if (stddev_ori < 0.1) {
            continue;
        }
        if (stddev_ori / stddev_dif < 1.5) {
            continue;
        }
        if(stddev_dif > matchpair.max_dif ||  stddev_ori < matchpair.min_ori) continue;
        if(mode == 0) {
            matchpair._pairs.push_back(std::make_pair(it->first, p2Id)); 
        } else {
            matchpair._pairs.push_back(std::make_pair(p2Id, it->first)); 
            // pts1[p1Id] = pt1;
            // pts2[p2Id] = pt2;
        }
        pts2[p2Id] = FeaturePoint(p2Id, 0, pt2);
        // p1Id += 1; 
        p2Id += 1;
        std::cout << p2Id << " : " << pt2 << "\n";
        matchpair._label_for_template_filter.push_back(1);
        matchpair._deprecated_pts.push_back(1);
        if (mode == 0) {
            cv::line(show_image, pt1, cv::Point2d(pt2.x + image1.cols, pt2.y), red);
            rect3 = cv::Rect(cv::Point(pt2.x + _last_image.cols - 20, pt2.y - 20), size40);
        } else {
            cv::line(show_image, pt2, cv::Point2d(pt1.x + image1.cols, pt1.y), red);
            rect3 = cv::Rect(cv::Point(pt1.x + image1.cols - 20, pt1.y - 20), size40);
        }
        
        cv::rectangle(show_image, rect3, blue);
        cv::imshow("matched", show_image);
        if (predict_mode == 0) {
            cv::waitKey();
            int ttt = 1;
        }
        

        simi_mat1.push(std::make_pair(MPID, PairInfo(MPID, dis_in_HT, stddev_ori, stddev_dif)));
        simi_mat2.push(std::make_pair(MPID, PairInfo(MPID, dis_in_HT, stddev_ori, stddev_dif)));
        MPID += 1;
    }
    // std::vector<PairInfo> pairInfoVec = {}; 
    // bool ispush_back = true;
    // double acent_value = 0, last_acent_value = 0;
    // std::vector<int> id_vec = {};
    // int id2_3 = simi_mat2.size() * 1 / 4;
    // std::vector<PairInfo> pairInfoVec1 = {};
    // //方差指标，获取simi_mat1的前n个,不满足的置为0,更严格
    // while (!simi_mat1.empty()) {
    //     // if(discard_match_id[simi_mat1.top().first] == 0) {//先前删除的点
    //     //     simi_mat1.pop();
    //     //     continue;
    //     // }
    //     if (ispush_back) {
    //         pairInfoVec.push_back(simi_mat1.top().second);
    //     } else {
    //         matchpair._label_for_template_filter[simi_mat1.top().first] = 0;
    //     }
    //     simi_mat1.pop();
    //     if (!simi_mat1.empty() || ispush_back) {
    //         last_acent_value = simi_mat1.top().second.stddev_dif - pairInfoVec.front().stddev_dif;
    //         if(acent_value != 0 && pairInfoVec.size() > 3){
    //             if (last_acent_value / acent_value > 3) {
    //                 ispush_back = false;
    //             }
    //         }
    //         if (pairInfoVec.size() > 3) {
    //             acent_value = pairInfoVec[3].stddev_dif - pairInfoVec[0].stddev_dif; 
    //         } else {
    //             acent_value = last_acent_value;
    //         }
    //     }
    // }
    // if (pairInfoVec.size() < 5) return;
  
    // //方差指标，删除simi_mat2的前n个,置为0,更宽松
    // // pairInfoVec1 = pairInfoVec; 
    // pairInfoVec.clear();
    // ispush_back = true;
    // acent_value = 0;
    // last_acent_value = 0;
    // while (!simi_mat2.empty()) {
    //     if(discard_match_id[simi_mat2.top().first] == 0) {//先前删除的点
    //         simi_mat2.pop();
    //         continue;
    //     }
    //     if (ispush_back) {
    //         pairInfoVec.push_back(simi_mat2.top().second);
    //         id_vec.push_back(simi_mat2.top().first);
    //     } else {
    //         discard_match_id[simi_mat2.top().first] = 0;
    //     }
    //     simi_mat2.pop();
    //     while (!simi_mat2.empty()) {//每次取值之前都要先pop掉已经删除的点
    //         if(discard_match_id[simi_mat2.top().first] == 1) {//先前删除的点
    //             break;
    //         }
    //         simi_mat2.pop();
    //     }
    //     if (ispush_back && !simi_mat2.empty()) {
    //         last_acent_value = pairInfoVec.front().stddev_ori - simi_mat2.top().second.stddev_ori;
    //         if(acent_value != 0){
    //             if (last_acent_value > 0.05 && last_acent_value / acent_value > 3) {
    //                 ispush_back = false;
    //             }
    //         }
    //         id2_3 = pairInfoVec.size() * 1 / 3;
    //         if (pairInfoVec.size() > 3) {
    //             acent_value = pairInfoVec[0].stddev_ori - pairInfoVec[3].stddev_ori; 
    //         } else {
    //             acent_value = last_acent_value;
    //         }
    //     }
    // }   
    // cv::imshow("matched", _show_image);
    // // cv::waitKey();
    // auto &pair_info_map = matchpair._pair_info_map;
    // for (int i = 0; i < pts1.size(); ++i) {
    //     if(discard_match_id[i] == 0) continue;
    //     pair_info_map[i] = pair_info_vec[i];

    // }
    // pairInfoVec1.clear();
}


void Matcher::initializeCoordinate() {

    std::vector<cv::Point2f> pts1 = {}, pts2 = {};
    std::vector<int> query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_DEPRECATED);
    getMatchedCVPts(matchPairs[_match_pairId], _keyframes[_left_image_id], _keyframes[_right_image_id], pts1, pts2, query);
    //本征矩阵是模板匹配后的点，可以直接用全部特征点，用RANSAC
    if (pts1.size() != pts2.size() || pts1.size() < 5) {
        std::cout << pts1.size() << "\n";
        std::cout << "too few match points, don't initilize\n";
        return;
    } 
    // _distance = std::sqrt(match._T(0) * match._T(0) + match._T(1) * match._T(1));
    double pixel_distance = std::sqrt(match._T.dot(match._T));
    if (pixel_distance < 10) {
        std::cout << _distance << "\n";
        std::cout << "too short distance\n";
        return;
    }
    cv::Mat mask, RT;
    bool isgetRT = getRT(pts1, pts2, _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, RT, mask, 1, 1);
    std::cout << "RT: " << _keyframes[_right_image_id]._R << " \n\n" << _keyframes[_right_image_id]._T << "\n\n";
    Eigen::MatrixXd K(3, 3), Re(3, 3), Te(3, 1);
    K = cv2Eigen<double>(_K);
    double fai1, omg1, kap1, fai2, omg2, kap2;
    // relativeOrientation(pts1, pts2, Re, Te, K, fai1, omg1, kap1, fai2, omg2, kap2);
    // Te = -Re.inverse() * Te;
    // Re = Re.inverse();
    // eigen2CV(Re, _keyframes[_right_image_id]._R);
    // eigen2CV(Te, _keyframes[_right_image_id]._T);
    // std::cout << RT << "\n";
    // isgetRT = getRT(pts1, pts2, _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, RT, mask, 1, 1);
    // std::cout << "RT: " << _keyframes[_right_image_id]._R << " \n\n" << _keyframes[_right_image_id]._T << "\n\n";
    // std::cout << RT << "\n";
    // isgetRT = getRT(pts1, pts2, _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, RT, mask, 0, 1);
    // std::cout << "RT: " << _keyframes[_right_image_id]._R << " \n\n" << _keyframes[_right_image_id]._T << "\n\n";
    // std::cout << RT << "\n";
    std::vector<double> herror = {};
    // _keyframes[1]._featurePts[11]._pt.x += 1;
    // _keyframes[1]._featurePts[11]._pt.y += 1;
    query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_DEPRECATED);
    getMatchedCVPts(matchPairs[_match_pairId], _keyframes[_left_image_id], _keyframes[_right_image_id], pts1, pts2, query);
    // query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_DEPRECATED);
    std::vector<cv::Point2f> HP2(pts1.size());
    // query = std::vector<int>(pts1.size(), 1);
    // HReprojectionError(pts1, pts2, RT, HP2, query, herror);
    // matchPairs[_match_pairId].updateQuery(query, QueryPtType::QUERY_BY_DEPRECATED);
    cv::Mat filted_image_final, filted_image_template, filted_image_depecated;
    drawMatchPts(_last_image, _current_image, filted_image_final, pts1, pts2);
    drawMatchPtsInCircle(filted_image_final, HP2, herror);
    cv::namedWindow("filted_image_final", CV_WINDOW_NORMAL);
    cv::imshow("filted_image_final", filted_image_final);
    // query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_TEMPLATE);
    // drawMatchPts(_last_image, _current_image, filted_image_template, pts1, pts2, query);
    // drawMatchPtsInCircle(filted_image_template, HP2);
    // cv::imshow("filted_image_template", filted_image_template);
    // query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_DEPRECATED);
    // drawMatchPts(_last_image, _current_image, filted_image_depecated, pts1, pts2, query);
    // drawMatchPtsInCircle(filted_image_depecated, HP2);
    // cv::imshow("filted_image_depecated", filted_image_depecated);
    // cv::waitKey();
    // cv::Mat homography_matrix = cv::findHomography ( pts1, pts1, cv::RANSAC, 3, noArray(), 2000, 0.99 );
    // recoverPose ( homography_matrix, pts1, pts1, R, t, focal_length, principal_point );
    if (!isgetRT) {
        std::cout << "falied to get E martrix\n";
        return;
    }
    double translation = std::sqrt(match._T(0) * match._T(0) + match._T(1) * match._T(1));
    std::cout << "T : " << _keyframes[_right_image_id]._T << "\n";
    cv::Mat R0 = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat T0 = cv::Mat::zeros(3, 1, CV_64FC1);
    // T0.at<double>(0, 0) = _global_odometry.x;
    // T0.at<double>(1, 0) = _global_odometry.y;
    // T0.at<double>(2, 0) = _global_odometry.z;
    _keyframes[_left_image_id]._R = R0;
    _keyframes[_left_image_id]._T = T0;
    cv::Mat &T = _keyframes[_right_image_id]._T; 
    double dis_in_T = std::sqrt(T.dot(T)); 
    double dis_in_plane = std::sqrt(
        T.at<double>(0, 0) * T.at<double>(0, 0) +
        T.at<double>(1, 0) * T.at<double>(1, 0));
    double dis_in_meter = pixel_distance * 0.00468;
    T = T / dis_in_T * dis_in_meter;
    // T.at<double>(2, 0) = 0;
    std::cout <<_keyframes[_right_image_id]._T << "\n";
    // _keyframes[_right_image_id]._T = T0 + _keyframes[_right_image_id]._T;
    // _keyframes[_right_image_id]._T = _keyframes[_right_image_id]._T;
    std::vector<cv::Point3f> structure = {};
    query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_DEPRECATED);
    getMatchedCVPts(matchPairs[_match_pairId], _keyframes[_left_image_id], _keyframes[_right_image_id], pts1, pts2, query);
    std::cout << pts1.size() << " " << pts2.size() << "\n";
    std::vector<double> angle_vec = {};
	// pts1[3].x += 1;
    checkInterAngle(R0, T0, _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, pts1, pts2, _K, angle_vec);
    reconstruct(_K, R0, T0, _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, pts1, pts2, structure);
    // forwardIntersection(R0, T0, _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, _K, pts1, pts2, structure);
    std::vector<double> error1 = {}, error2 = {};
    Eigen::MatrixXd Re0 = cv2Eigen<double>(R0);
    Eigen::MatrixXd Te0 = cv2Eigen<double>(T0);
    Eigen::MatrixXd Re1 = cv2Eigen<double>(_keyframes[_right_image_id]._R);
    Eigen::MatrixXd Te1 = cv2Eigen<double>(_keyframes[_right_image_id]._T);
    Eigen::MatrixXd Ke = cv2Eigen<double>(_K);
    reprojectionError(structure, pts1, pts2, Re0, Te0, Re1, Te1, Ke, _f, error1, error2);
    // drawMatchPtsInCircle(filted_image_depecated, HP2, error1);
    // cv::imshow("filted_image_depecated", filted_image_depecated);
    // cv::waitKey();
    std::cout << "structure: " << structure.size() << "\n";
    std::cout << R0 << " \n" << T0 << "\n";
    std::cout << _keyframes[_right_image_id]._R << " \n" << _keyframes[_right_image_id]._T << "\n";
    const auto &pair = matchPairs[_match_pairId]._pairs;
    int count = 0, query_count = 0;
    //把匹配的结果点生成的三维点写入该影像关键帧对应三维坐标点
    for (auto it = pair.begin(); it != pair.end(); it++) {
        if (query[query_count] == 0) {
            query_count += 1;
            continue;
        } 
        _keyframes[_left_image_id]._threeDPts[it->first] = structure[count];
        _keyframes[_right_image_id]._threeDPts[it->second] = structure[count];
        const auto &pt1 = _keyframes[_left_image_id]._featurePts[it->first]._pt;
        const auto &pt2 = _keyframes[_right_image_id]._featurePts[it->second]._pt; 
        cv::Rect2d rectl(cv::Point2d(pt1.x - 15, pt1.y - 15), size30);
        cv::Rect2d rectr(cv::Point2d(pt2.x + _last_image.cols - 15, pt2.y - 15), size30);
        cv::rectangle(_show_image, rectl, red);
        cv::rectangle(_show_image, rectr, red);
        query_count += 1;
        count += 1;
    }
    cv::imshow("matched", _show_image);
    //求出左右片所有特征点坐标
    
    //可视化
    // sensor_msgs::PointCloud2 tmp_ros_cloud;
    _point_count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    _keyframes[_left_image_id]._is_initilize_frame = true;
    for (auto it = _keyframes.begin(); it != _keyframes.end(); it++) {
        const auto &_threeDPts_in_frame = it->second._threeDPts;
        if (it->second._is_initilize_frame) tmp_pcl_cloud->points.clear();
        for (auto sit = _threeDPts_in_frame.begin(); sit != _threeDPts_in_frame.end(); sit++) {
            double x = sit->second.x;
            double y = sit->second.y;
            double z = sit->second.z;
            _point_count += 1;
            tmp_pcl_cloud->points.push_back(pcl::PointXYZ(x, y ,z));
        }
    }
    // *pcl_cloud += *tmp_pcl_cloud;
    std::cout << _point_count << "\n";
    std::cout << ros_cloud.data.size() << "  cloud size\n";
    pcl::toROSMsg(*tmp_pcl_cloud, ros_cloud);
    // std::cout << cloud->size() << "\n";
    // ros_cloud = ros_cloud + tmp_ros_cloud;
    ros_cloud.header.frame_id = "map";
    ros_cloud.header.stamp = ros::Time::now();
    isinitilized = true;
}

void Matcher::solveHomography() {

    std::vector<cv::Point2f> pts1 = {}, pts2 = {};
    std::vector<int> query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_DEPRECATED);
    getMatchedCVPts(matchPairs[_match_pairId], 
        _keyframes[_left_image_id], 
        _keyframes[_right_image_id], 
        pts1, pts2, query);
    //本征矩阵是模板匹配后的点，可以直接用全部特征点，用RANSAC
    if (pts1.size() != pts2.size() || pts1.size() < 5) {
        std::cout << pts1.size() << "\n";
        std::cout << "too few match points, don't solve\n";
        isinitilized = false;
        return;
    } 
    // _distance = std::sqrt(match._T(0) * match._T(0) + match._T(1) * match._T(1));
    double pixel_distance = sqrt(match._T.dot(match._T));
    if (pixel_distance < 10) {
        std::cout << _distance << "\n";
        std::cout << "too short distance, don't solve\n";
        isinitilized = false;
        return;
    }
    cv::Mat mask, RT;
    cv::Mat tR, tT;
    bool isgetRT = getRT(pts1, pts2, tR, tT, RT, mask, 1, 1);
    std::cout << "RT: " << tR << " \n\n" << tT << "\n\n";
    // Eigen::MatrixXd K(3, 3), Re(3, 3), Te(3, 1);
    // K = cv2Eigen<double>(_K);
    // double fai1, omg1, kap1, fai2, omg2, kap2;
    // relativeOrientation(pts1, pts2, Re, Te, K, fai1, omg1, kap1, fai2, omg2, kap2);
        // std::vector<double> herror = {};
        // query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_DEPRECATED);
        // getMatchedCVPts(matchPairs[_match_pairId], _keyframes[_left_image_id], _keyframes[_right_image_id], pts1, pts2, query);
    // query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_DEPRECATED);
        // std::vector<cv::Point2f> HP2(pts1.size());
    // query = std::vector<int>(pts1.size(), 1);
    // HReprojectionError(pts1, pts2, RT, HP2, query, herror);
    // matchPairs[_match_pairId].updateQuery(query, QueryPtType::QUERY_BY_DEPRECATED);
        // cv::Mat filted_image_final, filted_image_template, filted_image_depecated;
        // drawMatchPts(_last_image, _current_image, filted_image_final, pts1, pts2);
        // drawMatchPtsInCircle(filted_image_final, HP2, herror);
        // cv::namedWindow("filted_image_final", CV_WINDOW_NORMAL);
        // cv::imshow("filted_image_final", filted_image_final);
    // query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_TEMPLATE);
    // drawMatchPts(_last_image, _current_image, filted_image_template, pts1, pts2, query);
    // drawMatchPtsInCircle(filted_image_template, HP2);
    // cv::imshow("filted_image_template", filted_image_template);
    // query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_DEPRECATED);
    // drawMatchPts(_last_image, _current_image, filted_image_depecated, pts1, pts2, query);
    // drawMatchPtsInCircle(filted_image_depecated, HP2);
    // cv::imshow("filted_image_depecated", filted_image_depecated);
    // cv::waitKey();
    if (!isgetRT) {
        std::cout << "falied to get RT martrix, don't solve\n";
        isinitilized = false;
        return;
    }
    std::cout << "T : " << _keyframes[_right_image_id]._T << "\n";
    cv::Mat &T = _keyframes[_right_image_id]._T; 
    cv::Mat &R = _keyframes[_right_image_id]._R; 
    //递推公式，ｔＴ需要根据公共３Ｄ点确定尺度
    double dis_in_T = std::sqrt(tT.dot(tT)); 
    double dis_in_meter = pixel_distance * 0.00468;
    tT = tT / dis_in_T * dis_in_meter;
    R = tR * _keyframes[_left_image_id]._R;
    T = tR * _keyframes[_left_image_id]._T + tT;
    std::cout <<_keyframes[_right_image_id]._T << "\n";
    // _keyframes[_right_image_id]._T = T0 + _keyframes[_right_image_id]._T;
    // _keyframes[_right_image_id]._T = _keyframes[_right_image_id]._T;
    std::vector<cv::Point3f> structure = {};
    query = matchPairs[_match_pairId].getQuery(QueryPtType::QUERY_BY_DEPRECATED);
    getMatchedCVPts(matchPairs[_match_pairId], _keyframes[_left_image_id], _keyframes[_right_image_id], pts1, pts2, query);
    std::cout << pts1.size() << " " << pts2.size() << "\n";
    std::vector<double> angle_vec = {};
	checkInterAngle(
        _keyframes[_left_image_id]._R, _keyframes[_left_image_id]._T, 
        _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, 
        pts1, pts2, _K, angle_vec);
    reconstruct(_K, 
        _keyframes[_left_image_id]._R, _keyframes[_left_image_id]._T, 
        _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, 
        pts1, pts2, structure);
    // forwardIntersection(R0, T0, _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, _K, pts1, pts2, structure);
    std::vector<double> error1 = {}, error2 = {};
    Eigen::MatrixXd Re0 = cv2Eigen<double>(_keyframes[_left_image_id]._R);
    Eigen::MatrixXd Te0 = cv2Eigen<double>(_keyframes[_left_image_id]._T);
    Eigen::MatrixXd Re1 = cv2Eigen<double>(_keyframes[_right_image_id]._R);
    Eigen::MatrixXd Te1 = cv2Eigen<double>(_keyframes[_right_image_id]._T);
    Eigen::MatrixXd Ke = cv2Eigen<double>(_K);
    reprojectionError(structure, pts1, pts2, Re0, Te0, Re1, Te1, Ke, _f, error1, error2);
    // drawMatchPtsInCircle(filted_image_depecated, HP2, error1);
    // cv::imshow("filted_image_depecated", filted_image_depecated);
    // cv::waitKey();
    std::cout << "structure: " << structure.size() << "\n";
    std::cout << _keyframes[_left_image_id]._R << " \n" << _keyframes[_left_image_id]._T << "\n";
    std::cout << _keyframes[_right_image_id]._R << " \n" << _keyframes[_right_image_id]._T << "\n";
    const auto &pair = matchPairs[_match_pairId]._pairs;
    int count = 0, query_count = 0;
    //把匹配的结果点生成的三维点写入该影像关键帧对应三维坐标点
    for (auto it = pair.begin(); it != pair.end(); it++) {
        if (query[query_count] == 0) {
            query_count += 1;
            continue;
        } 
        // _keyframes[_left_image_id]._threeDPts[it->first] = structure[count];
        _keyframes[_right_image_id]._threeDPts[it->second] = structure[count];
        // const auto &pt1 = _keyframes[_left_image_id]._featurePts[it->first]._pt;
        // const auto &pt2 = _keyframes[_right_image_id]._featurePts[it->second]._pt; 
        // cv::Rect2d rectl(cv::Point2d(pt1.x - 15, pt1.y - 15), size30);
        // cv::Rect2d rectr(cv::Point2d(pt2.x + _last_image.cols - 15, pt2.y - 15), size30);
        // cv::rectangle(_show_image, rectl, red);
        // cv::rectangle(_show_image, rectr, red);
        query_count += 1;
        count += 1;
    }
    cv::imshow("matched", _show_image);
    //求出左右片所有特征点坐标
    
    //可视化
    // sensor_msgs::PointCloud2 tmp_ros_cloud;
    _point_count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    // _keyframes[_left_image_id]._is_initilize_frame = true;
    for (auto it = _keyframes.begin(); it != _keyframes.end(); it++) {
        const auto &_threeDPts_in_frame = it->second._threeDPts;
        if (it->second._is_initilize_frame) tmp_pcl_cloud->points.clear();
        for (auto sit = _threeDPts_in_frame.begin(); sit != _threeDPts_in_frame.end(); sit++) {
            double x = sit->second.x;
            double y = sit->second.y;
            double z = sit->second.z;
            _point_count += 1;
            tmp_pcl_cloud->points.push_back(pcl::PointXYZ(x, y ,z));
        }
    }
    // *pcl_cloud += *tmp_pcl_cloud;
    std::cout << _point_count << "\n";
    std::cout << ros_cloud.data.size() << "  cloud size\n";
    pcl::toROSMsg(*tmp_pcl_cloud, ros_cloud);
    // std::cout << cloud->size() << "\n";
    // ros_cloud = ros_cloud + tmp_ros_cloud;
    ros_cloud.header.frame_id = "map";
    ros_cloud.header.stamp = ros::Time::now();
    // isinitilized = true;
}

void Matcher::solvePnP() {
    cv::Mat r, R, T;
    int left_id = matchPairs[_match_pairId]._left_image_id;
    int right_id = matchPairs[_match_pairId]._right_image_id;
    const auto &pts1 = _keyframes[left_id]._featurePts;
    const auto &pts2 = _keyframes[right_id]._featurePts;
    //在匹配对里面搜索包含左片已经求得公共点的
    std::vector<cv::Point3f> threeDPts = {};
    std::vector<cv::Point2f> pt1 = {}, pt2 = {};
    std::vector<cv::Point2f> tpt1 = {}, tpt2 = {};
    const auto &pair = matchPairs[_match_pairId]._pairs;
    auto &query = matchPairs[_match_pairId]._label_for_template_filter;
    auto &_threeDPts  = _keyframes[_left_image_id]._threeDPts;
    for (int i = 0; i < pair.size(); ++i) {
        if (query[i] == 0) continue;
        const auto &pt = _keyframes[_left_image_id]._featurePts[pair[i].second]._pt;
        pt1.push_back(_keyframes[_left_image_id]._featurePts[pair[i].first]._pt);
        pt2.push_back(_keyframes[_right_image_id]._featurePts[pair[i].second]._pt);
        cv::Rect2d rect1(cv::Point2d(pt.x - 20, pt.y - 20), size40);
        cv::rectangle(_show_image, rect1, green);
        if (_threeDPts.find(pair[i].first) == _threeDPts.end()) {//没有三维坐标的，需要后续重建

        } else {//有三维点，用于重建
            threeDPts.push_back(_threeDPts[pair[i].first]);
            tpt1.push_back(_keyframes[_left_image_id]._featurePts[pair[i].first]._pt);
            tpt2.push_back(_keyframes[_right_image_id]._featurePts[pair[i].second]._pt);
            cv::Rect2d rect2(cv::Point2d(tpt1.back().x - 20, tpt1.back().y - 20), size40);
            cv::rectangle(_show_image, rect2, red);

        }
    }
    cv::imshow("matched", _show_image);
    cv::waitKey();
    //求解变换矩阵
    std::cout << threeDPts.size() << " " << tpt2.size() << "\n";
    if (threeDPts.size() != tpt2.size() || threeDPts.size() < 5) {
        std::cout << threeDPts.size() << "\n";
        std::cout << "too few 3D-2D point pairs, back to initilize\n";
        isinitilized = false;
        return;
    }
    // cv::solvePnPRansac
    bool solved = solvePnPRansac(threeDPts, tpt2, _K, cv::noArray(), r, T);
    //将旋转向量转换为旋转矩阵
    Rodrigues(r, R);
    //保存变换矩阵
    _keyframes[_right_image_id]._R = R;
    _keyframes[_right_image_id]._T = T;
    //根据之前求得的R，T进行三维重建
    std::vector<cv::Point3f> next_structure;
    // auto it = matchPairs
    std::cout << _keyframes[_left_image_id]._R << "\n\n" << _keyframes[_left_image_id]._T << "\n\n";
    std::cout << _keyframes[_right_image_id]._R << "\n\n" << _keyframes[_right_image_id]._T << "\n";
    std::cout << R.inv() << "\n";
    std::vector<double> angle_vec = {};
    checkInterAngle(_keyframes[_left_image_id]._R,
        _keyframes[_left_image_id]._T,
        _keyframes[_right_image_id]._R,
        _keyframes[_right_image_id]._T,
        pt1, pt2, _K, angle_vec);
    std::cout << _K << "\n";
    reconstruct(_K, 
        _keyframes[_left_image_id]._R,
        _keyframes[_left_image_id]._T,
        _keyframes[_right_image_id]._R,
        _keyframes[_right_image_id]._T,
        pt1, pt2, next_structure);
    // forwardIntersection(
    //     _keyframes[_left_image_id]._R, _keyframes[_left_image_id]._T, 
    //     _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T,
    //     _K, pt1, pt2, next_structure);
    std::vector<double> error1 = {}, error2 = {};
    Eigen::MatrixXd Re0 = cv2Eigen<double>(_keyframes[_left_image_id]._R);
    Eigen::MatrixXd Te0 = cv2Eigen<double>(_keyframes[_left_image_id]._T);
    Eigen::MatrixXd Re1 = cv2Eigen<double>(_keyframes[_right_image_id]._R);
    Eigen::MatrixXd Te1 = cv2Eigen<double>(_keyframes[_right_image_id]._T);
    Eigen::MatrixXd Ke = cv2Eigen<double>(_K);
    reprojectionError(next_structure, pt1, pt2, Re0, Te0, Re1, Te1, Ke, _f, error1, error2);
    int count = 0;
    // auto &structures  = matchPairs[_match_pairId]._structures;
    for (int i = 0; i < pair.size(); ++i) {
        if (query[i] == 0) continue;
        // if (_threeDPts.find(it->first) == _threeDPts.end()) continue;
        // if (structures.find(it->first) != structures.end()) continue;
        _keyframes[_right_image_id]._threeDPts[pair[i].second] = next_structure[count];
        // const auto &pt1 = _keyframes[_left_image_id]._featurePts[it->first]._pt;
        const auto &pt2 = _keyframes[_right_image_id]._featurePts[pair[i].second]._pt; 
        // cv::Rect2d rectl(cv::Point2d(pt1.x - 15, pt1.y - 15), size30);
        cv::Rect2d rectr(cv::Point2d(pt2.x + _last_image.cols - 15, pt2.y - 15), size30);
        // cv::rectangle(_show_image, rectl, red);
        cv::rectangle(_show_image, rectr, red);
        count += 1;
    }
    //求出剩余的三维点，直接根据PnP相机位姿？
    
    // sensor_msgs::PointCloud2 tmp_ros_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
    _point_count = 0;
    for (auto it = _keyframes.begin(); it != _keyframes.end(); it++) {
        const auto &_threeDPts_in_frame = it->second._threeDPts;
        if (it->second._is_initilize_frame) tmp_pcl_cloud->points.clear();
        for (auto sit = _threeDPts_in_frame.begin(); sit != _threeDPts_in_frame.end(); sit++) {
            double x = sit->second.x;
            double y = sit->second.y;
            double z = sit->second.z;
            _point_count += 1;
            tmp_pcl_cloud->points.push_back(pcl::PointXYZ(x, y ,z));
        }
    }
    std::cout << _point_count << "\n";
    std::cout << ros_cloud.data.size() << "  cloud size\n";
    // std::cout << tmp_pcl_cloud->points.size() << "\n";
    pcl::toROSMsg(*tmp_pcl_cloud, ros_cloud);
    ros_cloud.header.frame_id = "map";
    ros_cloud.header.stamp = ros::Time::now();
    std::cout << "------------\n";
    std::cout << "---add one--\n";
    std::cout << "------------\n";
}


std::vector<cv::Rect2d> Matcher::extractPatchs(cv::Mat image, double step, double width, double height) {
    std::vector<cv::Rect2d> patches = {};
    int horizon_count = image.cols / step + 1;
    int vertical_count = image.rows / step + 1;
    // std::cout << horizon_count * vertical_count << "\n";
    patches.resize(horizon_count * vertical_count);
    cv::Mat tem_image = image.clone();
    int patch_count = 0;
    for (int h = 0; h < horizon_count; ++h) {
        for (int v = 0; v < vertical_count; ++v) {
            int x1 = step * h;
            int y1 = step * v;
            if (x1 > image.cols || y1 > image.rows) continue;
            int x2 =  step * h + width;
            int y2 =  step * v + height;
            int t_width = x2 >= image.cols ? image.cols - x1 : width;
            int t_height = y2 >= image.rows ? image.rows - y1 : height;
            cv::Rect rect(x1 ,y1, t_width, t_height);
            // cv::rectangle(tem_image, rect, cv::Vec3b(0, 255, 0));
            // std::cout << rect << "\n";
            // cv::imshow("extract", tem_image);
            // cv::waitKey();
            patches[patch_count] = rect;
            patch_count += 1;
        }
    }
    // std::cout << "first\n\n";
    // for (size_t i = 0; i < patches.size(); ++i) {
    //     std::cout << patches[i] << "\n";
    // }
    // std::cout << "first\n\n";
    // std::cout << patches.size() << "\n";
    patches.shrink_to_fit();
    // std::cout << patches.size() << "\n";
    // for (size_t i = 0; i < patches.size(); ++i) {
    //     std::cout << patches[i] << "\n";
    // }
    // std::cout << "second\n\n";
    return patches;
    
}

cv::Mat Matcher::extractPatch(cv::Mat image) {
    cv::Mat patch, tem_patch;
    cv::namedWindow("test", cv::WINDOW_NORMAL);
    int step = _current_image.cols / 16;
    int width = step * 1.25;
    int height = step * 1.25;
    std::vector<cv::Rect2d> patches = extractPatchs(image, step, width, height);
    

    // std::cout << tem_patch.rows << " " << tem_patch.cols << "\n";
    // cv::circle(tem_patch, cv::Point2f(300, 400), 5, cv::Vec3b(0,255,0));
    // cv::circle(tem_patch, cv::Point2f(300, 300), 5, cv::Vec3b(0,255,0));
    // cv::circle(tem_patch, cv::Point2f(400, 300), 5, cv::Vec3b(0,255,0));
    // cv::circle(tem_patch, cv::Point2f(400, 400), 5, cv::Vec3b(0,255,0));
    // cv::imshow("test", tem_patch);
    // cv::waitKey();
    return patch;
}
std::ostream &operator<<(std::ostream &out, Result &A) {
    out << std::setprecision(17);
    out << "left image id : " << A.left_image_id << "\n"; 
    out << "right image id : " << A.right_image_id << "\n"; 
    out << "match : " << A._match << "\n"; 
    return out;
}

std::ostream &operator<<(std::ofstream &out, Result &A) {
    out << std::setprecision(6);
    out << "id: " << A._id << "\n";
    out << "distance: " << A._distance << "\n";
    out << "left image id : " << std::setprecision(17) << A.left_image_id << "\n"; 
    out << "right image id : " << std::setprecision(17) << A.right_image_id << "\n"; 
    out << "match : \n" << A._match << "\n"; 
    return out;
}

void Matcher::filtByReprojectionError(int matchId, std::vector<int> &discard_match_id) {
    std::vector<cv::Point2f> pts1 = {}, pts2 = {};
    std::vector<int> query = std::vector<int>(matchPairs[_match_pairId]._pairs.size(), 1);
    getMatchedCVPts(matchPairs[_match_pairId], _keyframes[_left_image_id], _keyframes[_right_image_id], pts1, pts2, query);
    //本征矩阵是模板匹配后的点，可以直接用全部特征点，用RANSAC
    if (pts1.size() != pts2.size() || pts1.size() < 5) {
        std::cout << pts1.size() << "\n";
        std::cout << "too few match points, don't initilize\n";
        return;
    } 
    // _distance = std::sqrt(match._T(0) * match._T(0) + match._T(1) * match._T(1));
    double pixel_distance = std::sqrt(match._T(0) * match._T(0) + match._T(1) * match._T(1));  
    if (pixel_distance < 10) {
        std::cout << _distance << "\n";
        std::cout << "too short distance\n";
        return;
    }
    cv::Mat mask, RT;
    bool isgetRT = getRT(pts1, pts2, _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, RT, mask);
    if (!isgetRT) {
        std::cout << "falied to get E martrix\n";
        return;
    }
    double translation = std::sqrt(match._T(0) * match._T(0) + match._T(1) * match._T(1));
    std::cout << "T : " << _keyframes[_right_image_id]._T << "\n";
    cv::Mat R0 = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat T0 = cv::Mat::zeros(3, 1, CV_64FC1);
    T0.at<double>(0, 0) = _global_odometry.x;
    T0.at<double>(1, 0) = _global_odometry.y;
    T0.at<double>(2, 0) = _global_odometry.z;
    _keyframes[_left_image_id]._R = R0;
    _keyframes[_left_image_id]._T = T0;
    cv::Mat &T = _keyframes[_right_image_id]._T; 
    double dis_in_T = std::sqrt(
        T.at<double>(0, 0) * T.at<double>(0, 0) +
        T.at<double>(1, 0) * T.at<double>(1, 0) +
        T.at<double>(2, 0) * T.at<double>(2, 0));
    double dis_in_plane = std::sqrt(
        T.at<double>(0, 0) * T.at<double>(0, 0) +
        T.at<double>(1, 0) * T.at<double>(1, 0));
    double dis_in_meter = pixel_distance * 0.00468;
    T = T / dis_in_T * dis_in_meter;
    std::cout <<_keyframes[_right_image_id]._T << "\n";
    _keyframes[_right_image_id]._T = T0 + _keyframes[_right_image_id]._T;
    // _keyframes[_right_image_id]._T = _keyframes[_right_image_id]._T;
    std::vector<cv::Point3f> structure = {};
    std::cout << pts1.size() << " " << pts2.size() << "\n";
	reconstruct(_K, R0, T0, _keyframes[_right_image_id]._R, _keyframes[_right_image_id]._T, pts1, pts2, structure);
    std::vector<double> error1 = {}, error2 = {};
    Eigen::MatrixXd Re0 = cv2Eigen<double>(R0);
    Eigen::MatrixXd Te0 = cv2Eigen<double>(T0);
    Eigen::MatrixXd Re1 = cv2Eigen<double>(_keyframes[_right_image_id]._R);
    Eigen::MatrixXd Te1 = cv2Eigen<double>(_keyframes[_right_image_id]._T);
    Eigen::MatrixXd Ke = cv2Eigen<double>(_K);
    reprojectionError(structure, pts1, pts2, Re0, Te0, Re1, Te1, Ke, _f, error1, error2);
    int match_size = matchPairs[_match_pairId]._pairs.size();
    for (int i = 0; i < match_size; ++i) {
        if (std::abs(error1[i]) > 1 || std::abs(error2[i]) > 1) {
            discard_match_id.push_back(i);
        } 
    }
}

void Matcher::filtByTemplate(int matchId, std::vector<int> &discard_match_id) {
    cv::Mat TMatched_image;
    auto matchpair = matchPairs[matchId];
    std::vector<cv::Point2f> tpts1 = {}, tpts2 = {}, pts1 = {}, pts2 = {};
    std::vector<int> query = std::vector<int>(matchPairs[_match_pairId]._pairs.size(), 1);
    getMatchedCVPts(matchpair, _keyframes[_left_image_id], _keyframes[_right_image_id], pts1, pts2, query);
    // std::vector<int> index = {1, 2, 4, 7, 8, 16, 28};
    // for (size_t i = 0; i < tpts1.size(); i++) {
    //     if (std::find(index.begin(), index.end(), i) != index.end()) continue;
    //     pts1.push_back(tpts1[i]);
    //     pts2.push_back(tpts2[i]);
    // }
    cv::Rect2d rect1, rect2;
    //左片当模板,左片是真值,右片是模板对应值
    // cv::namedWindow("rotate2", cv::WINDOW_NORMAL);
    std::vector<cv::Point2f> pts3 = {};
    cv::Mat result_image;
    std::vector<double> error0 = {};
    
    matchPairs[matchId]._deprecated_pts = std::vector<int>(matchpair._pairs.size(), 1); 
    std::vector<PairInfo> &pair_info_vec = matchPairs[matchId]._pair_info_vec;
    pair_info_vec = std::vector<PairInfo>(matchpair._pairs.size(), PairInfo());
    std::vector<PairInfo> pairInfoVec = {};
    auto cmpPairInfo1 = [](PairInfoPair l, PairInfoPair r){return l.second.stddev_dif > r.second.stddev_dif;};
    auto cmpPairInfo2 = [](PairInfoPair l, PairInfoPair r){return l.second.stddev_ori < r.second.stddev_ori;};
    std::priority_queue<PairInfoPair, std::vector<PairInfoPair>, decltype(cmpPairInfo1)> simi_mat1(cmpPairInfo1);
    std::priority_queue<PairInfoPair, std::vector<PairInfoPair>, decltype(cmpPairInfo2)> simi_mat2(cmpPairInfo2);
    for (size_t i = 0; i < pts1.size(); ++i) {
        rect1 = cv::Rect2d(cv::Point2d(pts1[i].x - 20, pts1[i].y - 20) , size40);
        rect2 = cv::Rect2d(cv::Point2d(pts2[i].x + _last_image.cols - 30, pts2[i].y - 30) , size60);
        // cv::rectangle(_show_image, rect1, red);
        // cv::rectangle(_show_image, rect2, red);
        // cv::line(_show_image, pts1[i], 
        //     cv::Point2d(pts2[i].x + image_left.cols, pts2[i].y), green);
        // submatch
        rect2.x = rect2.x - _last_image.cols;
        if (exceedImage(rect1, _last_image, false) || 
            exceedImage(rect2, _current_image, false)) {
            error0.push_back(100);
            discard_match_id[i] = 0;
            matchPairs[matchId]._deprecated_pts[i] = 0;
            continue;
        }
        cv::Mat match_image = _current_image(rect2);//right
        cv::Mat templ_image = _last_image(rect1);//left
        // std::cout << match_image.size() << " " << templ_image.size() << "\n";
        submatch = baseTemplate(match_image, templ_image, TMatched_image, 0);
        std::cout << submatch._T << "\n";
        // std::cout << pts1[i] << "\n" << pts2[i] << "\n";
        cv::Size dst_size;
        //translate the left to the right
        double out_x, out_y;
        int h_cols = 20;
        int h_rows = 20;
        Eigen::Vector2d in_XY(0, 0);//原点转换到右边
        Eigen::Vector2d out_xy = submatch._R * in_XY; 
        out_x = out_xy(0) + submatch._T(0) + pts2[i].x;
        out_y = out_xy(1) + submatch._T(1) + pts2[i].y;
        pts3.push_back(cv::Point2f(out_x, out_y));
        double dx = out_x - pts2[i].x;
        double dy = out_y - pts2[i].y;
        double dis_in_HT = std::sqrt(dx * dx + dy * dy);
        error0.push_back(dis_in_HT);
        pair_info_vec[i] = PairInfo(i, dis_in_HT, 0, 0);
        if (dis_in_HT > 2 ) {//deprecated
            discard_match_id[i] = 0;
            if(dis_in_HT > 5) {
                matchPairs[matchId]._deprecated_pts[i] = 0;
            }
            matchPairs[matchId]._pair_info_map[i] = PairInfo(i, dis_in_HT, 0, 0);
            continue;
        }
        
        // continue;
        // std::cout << out_x - pts2[i].x << " " << out_y - pts2[i].y << "\n";
        
        //for debug
        cv::Mat color, color1, color2;
        int inner_x = 3 + rect1.width / 2 - rect2.width / 2 + submatch._coff_mat.cols / 2;
        int inner_y = 3 + rect1.height / 2 - rect2.height / 2 + submatch._coff_mat.rows / 2;
        cv::Rect inner_rect(inner_x, inner_y, rect2.width - rect1.width - 6, rect2.height - rect1.height - 6); 
        // cv::cvtColor(submatch._coff_mat, color1, cv::COLOR_GRAY2BGR, 3); 
        submatch._coff_mat = submatch._coff_mat(inner_rect);
        const cv::Mat &coff_mat0 = submatch._coff_mat;
        double sizeGS = 5;
        cv::Mat coff_mat;
        cv::GaussianBlur(coff_mat0, coff_mat, cv::Size(sizeGS, sizeGS), sizeGS);
        cv::Mat rotate90;
        cv::Mat rotate_matrix = cv::getRotationMatrix2D(cv::Point(coff_mat.cols / 2, coff_mat.rows / 2), 90, 1);
        cv::warpAffine(coff_mat, rotate90, rotate_matrix, coff_mat.size());
        cv::Mat mul_ori = coff_mat.mul(coff_mat);
        cv::Mat mul_90 = coff_mat.mul(rotate90);
        cv::Mat diff_image = mul_90 - mul_ori;
        std::cout << " ttt: " << coff_mat.channels() << "\n";
        // cv::imshow("mul_ori", mul_ori);
        // cv::imshow("mul_90", mul_90);
        // cv::imshow("diff_image", diff_image);
        cv::Scalar mean, stddev;
        double stddev_ori, stddev_dif;
        cv::meanStdDev(coff_mat, mean, stddev);
        stddev_ori = stddev[0];
        cv::meanStdDev(diff_image, mean, stddev);
        stddev_dif = stddev[0];
        // if (/* condition */)
        // {
        //     /* code */
        // }
        
        // cv::meanStdDev(coff_mat, mean, stddev);
        // cv::waitKey();
        cv::Mat tmp_dst2 = cv::Mat::zeros(_show_image.size(), CV_8UC3);
        cv::Point ori1, ori2;
        ori2.x = pts1[i].x - submatch._coff_mat.cols / 2;
        ori2.y = pts1[i].y - submatch._coff_mat.rows / 2;
        cv::Rect rect3(ori2, submatch._coff_mat.size());
        // templ_image.copyTo(tmp_dst2(rect1));
        submatch._coff_mat.convertTo(color1, CV_32FC3);
        // color1 = (color1 - 0.5) * 2;
        color1.convertTo(color, CV_8UC3, 255.0);
        cv::cvtColor(color, color2, cv::COLOR_GRAY2BGR);
        std::cout << submatch._coff_mat.channels() << " " << templ_image.type() << " "<< color.type() << " " << tmp_dst2.type() <<"\n";
        if (exceedImage(rect3, tmp_dst2, false)) continue;
        color2.copyTo(tmp_dst2(rect3));
        cv::Mat mask = tmp_dst2.clone();
        mask.setTo(cv::Vec3i(0, 0, 0));
        mask(rect3) = cv::Vec3i(255, 255, 255);
        // tmp_dst2(rect3) = color;
        cv::imshow("math", tmp_dst2);
        // std::cout << submatch._coff_mat << "\n\n";
        // std::cout << color << "\n\n";
        // cv::waitKey();

        // cv::Point2d center;
        // center.x = tmp_dst1.cols / 2;
        // center.y = tmp_dst1.rows / 2;
        rotate_matrix = cv::getRotationMatrix2D(pts1[i], match._angle, 1.0);
        cv::Mat rotate1, rotate2, rotate3;
        cv::Mat translate_martrix = cv::Mat::zeros(2, 3, CV_32FC1);
        translate_martrix.at<float>(0, 0) = 1;
        translate_martrix.at<float>(1, 1) = 1;
        translate_martrix.at<float>(0, 2) = submatch._T.x() + pts2[i].x - pts1[i].x + _last_image.cols;
        translate_martrix.at<float>(1, 2) = submatch._T.y() + pts2[i].y - pts1[i].y;
        // std::cout << rotate_matrix << "\n";
        // std::cout << submatch._T.x() << " " << match._T.y() << "---\n";
        // cv::warpAffine(tmp_dst1, rotate1, rotate_matrix, dst_size);
            // cv::warpAffine(tmp_dst2, rotate1, rotate_matrix, dst_size);
            // cv::warpAffine(rotate1, rotate2, translate_martrix, dst_size);
            // cv::Mat t_mask1, t_mask2, t_mask3;
            // cv::warpAffine(mask, t_mask1, rotate_matrix, dst_size);
            // cv::warpAffine(t_mask1, t_mask2, translate_martrix, dst_size);
            // result_image = _show_image.clone();
            // cv::bitwise_not(t_mask2, t_mask3);
            // cv::bitwise_and(result_image, t_mask3, rotate1, cv::noArray());
            // cv::bitwise_or(rotate1, rotate2, _show_image, cv::noArray());
        // cv::imshow("rotate2", rotate2);
        // image.copyTo(tmp_dst1(rect1));
        // cv::imshow("gray", rotate1);
        // cv::imshow("math", rotate2);
        cv::imshow("matched", _show_image);
        // cv::imshow("rotate2", result_image);
        std::cout << "stddev: " << stddev_ori << " " << stddev_dif << " " << stddev_ori / stddev_dif << "\n";
        std::cout << dis_in_HT << "\n";
        // cv::waitKey();
        int ttt = 0;
        if (stddev_ori < 0.1) {
            discard_match_id[i] = 0;
            continue;
        }
        if (stddev_ori / stddev_dif < 1.5) {
            discard_match_id[i] = 0;
            continue;
        }
        pair_info_vec[i] = PairInfo(i, dis_in_HT, stddev_ori, stddev_dif);
        simi_mat1.push(std::make_pair(i, PairInfo(i, dis_in_HT, stddev_ori, stddev_dif)));
        simi_mat2.push(std::make_pair(i, PairInfo(i, dis_in_HT, stddev_ori, stddev_dif)));
        pair_info_vec[i] = PairInfo(i, dis_in_HT, stddev_ori, stddev_dif);
        ttt += 1;
    }
    pairInfoVec.clear();
    bool ispush_back = true;
    double acent_value = 0, last_acent_value = 0;
    std::vector<int> id_vec = {};
    int id2_3 = simi_mat2.size() * 1 / 4;
    std::vector<PairInfo> pairInfoVec1 = {};
    //方差指标，获取simi_mat1的前n个,不满足的置为0,更严格
    while (!simi_mat1.empty()) {
        // if(discard_match_id[simi_mat1.top().first] == 0) {//先前删除的点
        //     simi_mat1.pop();
        //     continue;
        // }
        if (ispush_back) {
            pairInfoVec.push_back(simi_mat1.top().second);
        } else {
            discard_match_id[simi_mat1.top().first] = 0;
        }
        simi_mat1.pop();
        if (!simi_mat1.empty() || ispush_back) {
            last_acent_value = simi_mat1.top().second.stddev_dif - pairInfoVec.front().stddev_dif;
            if(acent_value != 0 && pairInfoVec.size() > 3){
                if (last_acent_value / acent_value > 3) {
                    ispush_back = false;
                }
            }
            if (pairInfoVec.size() > 3) {
                acent_value = pairInfoVec[3].stddev_dif - pairInfoVec[0].stddev_dif; 
            } else {
                acent_value = last_acent_value;
            }
        }
    }
    if (pairInfoVec.size() < 5) return;
  
    //方差指标，删除simi_mat2的前n个,置为0,更宽松
    pairInfoVec1 = pairInfoVec; 
    pairInfoVec.clear();
    ispush_back = true;
    acent_value = 0;
    last_acent_value = 0;
    while (!simi_mat2.empty()) {
        if(discard_match_id[simi_mat2.top().first] == 0) {//先前删除的点
            simi_mat2.pop();
            continue;
        }
        if (ispush_back) {
            pairInfoVec.push_back(simi_mat2.top().second);
            id_vec.push_back(simi_mat2.top().first);
        } else {
            discard_match_id[simi_mat2.top().first] = 0;
        }
        simi_mat2.pop();
        while (!simi_mat2.empty()) {//每次取值之前都要先pop掉已经删除的点
            if(discard_match_id[simi_mat2.top().first] == 1) {//先前删除的点
                break;
            }
            simi_mat2.pop();
        }
        if (ispush_back && !simi_mat2.empty()) {
            last_acent_value = pairInfoVec.front().stddev_ori - simi_mat2.top().second.stddev_ori;
            if(acent_value != 0){
                if (last_acent_value > 0.05 && last_acent_value / acent_value > 3) {
                    ispush_back = false;
                }
            }
            id2_3 = pairInfoVec.size() * 1 / 3;
            if (pairInfoVec.size() > 3) {
                acent_value = pairInfoVec[0].stddev_ori - pairInfoVec[3].stddev_ori; 
            } else {
                acent_value = last_acent_value;
            }
        }
        // if (ispush_back && !simi_mat2.empty()) {
        //     last_acent_value = simi_mat2.top().second.stddev_ori - pairInfoVec.front().stddev_ori;
        //     if(acent_value != 0 && pairInfoVec.size() > 2){
        //         int half_id = pairInfoVec.size() * 1 / 3;
        //         if (last_acent_value / acent_value > 3 && last_acent_value / pairInfoVec[half_id].stddev_ori > 5) {
        //             ispush_back = false;
        //         }
        //     }
        //     if (pairInfoVec.size() > id2_3) {
        //         acent_value = pairInfoVec[id2_3].stddev_ori - pairInfoVec[0].stddev_ori; 
        //     } else {
        //         acent_value = last_acent_value;
        //     }
        // }
    }   
    cv::imshow("matched", _show_image);
    // cv::waitKey();
    // discard_match_id[14] = 0;
    // discard_match_id[11] = 0;
    auto &pair_info_map = matchPairs[matchId]._pair_info_map;
    for (int i = 0; i < pts1.size(); ++i) {
        if(discard_match_id[i] == 0) continue;
        pair_info_map[i] = pair_info_vec[i];

    }
    
    pairInfoVec1.clear();
    // acent_value = pairInfoVec[1].stddev - pairInfoVec[0].stddev; 
    // for (int i = 1; i < pairInfoVec.size(); ++i) {
    //     last_acent_value = pairInfoVec[1].stddev - pairInfoVec[0].stddev; 
    //     if (last_acent_value / acent_value > 3)
    // }
        

}
            
void Matcher::drawMatchPoints(cv::Mat &show_image, 
    const std::vector<int> &ids,
    const std::vector<std::pair<int, int>> &pair) {
    // matchPairs[i].
    for (int i = 0; i < ids.size(); ++i) {
        if (ids[i] == 0) continue;
        const auto &pt = _keyframes[_right_image_id]._featurePts[pair[i].second]._pt;
        cv::Rect2d rect(cv::Point2d(pt.x - 25 + _last_image.cols, pt.y - 25), size50);
        cv::rectangle(_show_image, rect, blue);
        // cv::imshow("matched", _show_image);
        // cv::waitKey();
    }
}


}