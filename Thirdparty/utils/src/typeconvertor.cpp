#ifndef TERGEO_VISUAL_ODOMETRY_TYPE_CONVERTOR
#define TERGEO_VISUAL_ODOMETRY_TYPE_CONVERTOR

#include "types.hpp"

namespace tergeo {
namespace visualodometry {

void eigen2CV(const Eigen::MatrixXd &eigenmat, cv::Mat &cvmat) {
    int row = eigenmat.rows();
    int col = eigenmat.cols();
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            cvmat.at<double>(i, j) = eigenmat(i, j);
        }
    }
}


}
}

#endif