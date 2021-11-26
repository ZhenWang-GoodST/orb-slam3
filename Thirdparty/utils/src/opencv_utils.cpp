#include "opencv_utils.h"

namespace tergeo {
namespace visualodometry {

cv::Mat stretch(const cv::Mat& image,int minvalue, int maxvalue) {
    // Histogram1D h;

    // cv::Mat hist = h.getHistogram(image);
    int channels[1];//使用的通道数量
    int histSize[1];//直方图箱子（bin）的数量
    const float* ranges[1];//像素值范围
    float hranges[2];
    histSize[0] = 256;//箱子个数设为256
    channels[0] = 0;//使用一个通道，默认为0
    hranges[0] = 0.0;
    hranges[1] = 256.0;
    ranges[0] = hranges;//值范围
    cv::Mat hist;
    cv::calcHist(&image, 1, channels, cv::Mat(), hist, 1,//一维直方图
        histSize, ranges);
    //创建查找表
    cv::Mat lut(1, 256, CV_8U);
    //构建查找表
    for (int i = 0; i < 256;i++) {
        if (i < minvalue)
            lut.at<uchar>(i) = 0;
        else if (i>maxvalue)
            lut.at<uchar>(i) = 255;
        else {
            lut.at<uchar>(i) = cvRound(255.0*(i - minvalue) / (maxvalue - minvalue));
        }
    }
    cv::Mat result;
    cv::LUT(image,lut,result);
    return result;
}
void adaptiveThreshold(const cv::Mat &hist, double thresh, int &minId, int &maxId) {
    cv::Point minLoc, maxLoc;
    double minV, maxV;
    cv::minMaxLoc(hist, &minV, &maxV, &minLoc, &maxLoc);
    double inlier = maxV;
    double total;
    cv::Scalar output = cv::sum(hist);
    total = output[0];
    minId = maxId = maxLoc.y;
    int minCount = 1000, maxCount = 1000;
    while ((inlier / total) < thresh || minCount > 400 || maxCount > 400) {
    // while ((inlier / total) < thresh) {
        if (minId - 1 < 0 || maxId + 1 > 255) break;
        minCount = hist.at<float>(0, minId - 1);
        maxCount = hist.at<float>(0, maxId + 1);
        if (minCount > maxCount) {
            minId -= 1;
            inlier += minCount; 
        } else {
            maxId += 1;
            inlier += maxCount; 
        }
    }
    std::cout << "final inlier: " << inlier / total << "\n";
}
//直方图拉伸
cv::Mat stretch(const cv::Mat& image, int minvalue, int maxvalue, bool makeFloat) {
    int channels[1];//使用的通道数量
    int histSize[1];//直方图箱子（bin）的数量
    const float* ranges[1];//像素值范围
    float hranges[2];
    histSize[0] = 256;//箱子个数设为256
    channels[0] = 0;//使用一个通道，默认为0
    hranges[0] = 0.0;
    hranges[1] = 256.0;
    ranges[0] = hranges;//值范围
    cv::Mat hist;
    cv::calcHist(&image, 1, channels, cv::Mat(), hist, 1,//一维直方图
        histSize, ranges);
    if (minvalue == 0 && maxvalue == 0) {
        adaptiveThreshold(hist, 0.98, minvalue, maxvalue);
    }
    std::cout << "minvalue: " << minvalue << " maxvalue: " << maxvalue << "\n"; 
    cv::Mat fMat, result = cv::Mat::zeros(image.size(), image.type());
    if (makeFloat) {
        image.convertTo(fMat, CV_32F);
        double sigX = 2;
        cv::Size ksize(9, 9);
        int range = maxvalue - minvalue;
        cv::GaussianBlur(fMat, fMat, ksize, sigX);
        for (int i = 0; i < image.rows; ++i) {
            uchar *ip = result.ptr<uchar>(i);
            float *fp = fMat.ptr<float>(i);
            for (int j = 0; j < image.cols; ++j) {
                float fValue = *fp++;
                if ((int)(image.at<uchar>(i, j)) < minvalue) {
                    *ip++ = 0;
                } else if ((int)(image.at<uchar>(i, j)) > maxvalue) {
                    *ip++ = 255;
                } else {
                    *ip++ = 255 * (fValue - minvalue) / range;
                }
            }
            ip = nullptr;
            fp = nullptr;
        }
        return result;
    } else {
        //创建查找表
        cv::Mat lut(1, 256, CV_8U);
        //构建查找表
        for (int i = 0; i < 256;i++) {
            if (i < minvalue)
                lut.at<uchar>(i) = 0;
            else if (i>maxvalue)
                lut.at<uchar>(i) = 255;
            else {
                lut.at<uchar>(i) = cvRound(255.0*(i - minvalue) / (maxvalue - minvalue));
            }
        }
        cv::Mat result;
        cv::LUT(image,lut,result);
        return result;
    }
    return result;
}

void singleMatch(const cv::Mat &image, 
    const cv::Mat &templ, cv::Mat &TMatched_image, Match match) {
    
    cv::Size dst_size;
    dst_size.width = 
        std::sqrt(image.cols * image.cols + image.rows * image.rows);
    dst_size.height = dst_size.width;
    cv::Mat tmp_dst1 = cv::Mat::zeros(dst_size, image.type());
    cv::Mat tmp_dst2 = cv::Mat::zeros(dst_size, image.type());
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
    // std::cout << 3 << "\n";
    // cv::waitKey();
}


Match baseTemplate(const cv::Mat &image, 
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
    cv::Mat translate_templ = cv::Mat::zeros(dst_size, image.type());
    cv::Point ori1, ori2;
    ori1.x = dst_size.width / 2 - image.cols / 2;
    ori1.y = dst_size.height / 2 - image.rows / 2;
    ori2.x = dst_size.width / 2 - templ.cols / 2;
    ori2.y = dst_size.height / 2 - templ.rows / 2;
    cv::Rect rect1(ori1, image.size()), rect2(ori2, templ.size());
    templ.copyTo(translate_templ(rect2));
    tmp_tran_templ = translate_templ.clone();
    // cv::cvtColor(rotate_templ, rotate_templ, CV_BGR2GRAY);
    cv::Mat match_templ;
    match_templ = templ.clone();
    // cv::cvtColor(templ, match_templ, CV_BGR2GRAY);
    Match best_match;
    best_match._coff = -2;
    for (double i = -5; i < 5; i=i + 1) {
        cv::Mat match_image, rotate_template, image_plus, mask;
        double template_angle = - initial_angle + i;
        cv::Mat tmp_match_image = cv::Mat::zeros(dst_size, image.type());
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
    Match match_;
    if (best_match._coff < match._coff) {
        best_match = match;
    }
    
        // match_queue.push(match_);
        
        //for check
        cv::Mat translate_martrix = cv::Mat::zeros(2, 3, CV_32FC1);
        translate_martrix.at<float>(0, 0) = 1;
        translate_martrix.at<float>(1, 1) = 1;
        translate_martrix.at<float>(0, 2) = match._T.x();
        translate_martrix.at<float>(1, 2) = match._T.y();
        cv::warpAffine(tmp_tran_templ, translate_templ, translate_martrix, dst_size);
        // std::cout << left_center << " " << match._T.x() << " " << match._T.y() << "\n";
        
        cv::rectangle(match_image, cv::Rect(maxLoc, match_templ.size()), cv::Vec3b(0, 255, 0));
        // cv::bitwise_or(match_image, translate_templ, image_plus);
        image_plus = match_image.clone();
        templ.copyTo(image_plus(cv::Rect(maxLoc, match_templ.size())));
        cv::imshow("result", result);
        cv::imshow("match", match_image);
        cv::imshow("image_plus", image_plus);
        // std::cout<< template_angle << " " << maxVal << " " << maxLoc << "\n";
        // cv::waitKey();
    }
    // Match t_match = match_queue.top(); 
    // match = match_queue.top();
    // std::cout << "match coff: " << match_queue.top()._coff << " angle " << match_queue.top()._angle << "\n";
    //origin of the following coordinate is in the center of the image
    singleMatch(image, templ, TMatched_image, best_match);
    return best_match;
}

cv::Mat getInnerPatch(const cv::Mat &image, const cv::Mat &templ) {
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
        cv::cvtColor(show_image, show_image, cv::COLOR_GRAY2BGR);
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
    bool psize = size < 0;
    for (int i = 0; i < pts.size(); ++i) {
        if (psize) {
            size = pts[i].size;
        }
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