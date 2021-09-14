
#include "types.hpp"

namespace tergeo{
namespace visualodometry {

void getMatchedCVPts(const MatchPair &matchpair, KeyFrame &lkeyframe, KeyFrame &rkeyframe,
    std::vector<cv::Point2f> &pts1, std::vector<cv::Point2f> &pts2,
    const std::vector<int> &query){
    pts1.clear(); pts2.clear();
    for (int i = 0; i < matchpair._pairs.size(); ++i) {
        if(query[i] == 0) continue;
        int left_id = matchpair._pairs[i].first;
        int right_id = matchpair._pairs[i].second;
        pts1.push_back(lkeyframe._featurePts[left_id]._pt);
        pts2.push_back(rkeyframe._featurePts[right_id]._pt);
    }
}



}
}
