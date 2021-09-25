#ifndef TERGEO_VISUAL_ODOMETRY_TYPE_CONVERTOR
#define TERGEO_VISUAL_ODOMETRY_TYPE_CONVERTOR

#include "types.hpp"

namespace tergeo {
namespace visualodometry {

template <typename T>
Eigen::MatrixXd cv2Eigen(const cv::Mat &cvmat) {
    Eigen::MatrixXd eigenmat(cvmat.rows, cvmat.cols);
    for (int i = 0; i < cvmat.rows; ++i) {
        for (int j = 0; j < cvmat.cols; ++j) {
            eigenmat(i, j) = cvmat.at<T>(i, j);
        }
    }
    return eigenmat;
}

void eigen2CV(const Eigen::MatrixXd &eigenmat, cv::Mat &cvmat);


}
}

#endif