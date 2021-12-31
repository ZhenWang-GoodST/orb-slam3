#ifndef TERGEO_VISUAL_ODOMETRY_PL_MATCHER
#define TERGEO_VISUAL_ODOMETRY_PL_MATCHER

#include "pl_block.h"


class PLMather
{
private:
    /* data */
public:
    std::vector<linematch_score> linematch_scores = {};
    PLMather(/* args */) {};
    ~PLMather() {};
    
    void calculateAngleInInitilization(PLStructure* plframe1, PLStructure* plframe2);
};


#endif
