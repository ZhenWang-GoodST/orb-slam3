#ifndef TERGEO_VISUAL_ODOMETRY_VO_TYPE_UTILS
#define TERGEO_VISUAL_ODOMETRY_VO_TYPE_UTILS

#include "types.hpp"

namespace tergeo{
namespace visualodometry {

void getMatchedCVPts(const MatchPair &matchpair, KeyFrame &lkeyframe, KeyFrame &rkeyframe,
    std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2,
    const std::vector<int> &query);



}
}

#endif