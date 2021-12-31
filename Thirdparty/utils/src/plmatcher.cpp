#include "plmatcher.h"

//对于初始化计算方向：
//每块找到一个匹配最好的线段，线段周围一定距离的点的方向就设置为这个线段的方向
//最好：长度相近，中点跟大尺度点变换关系一致，匹配分数高
//靠近这些线段一定距离的点才分配方向，靠近两个差不多的不分配
//to do 现在第一帧计算方向为upright，第二帧为角度偏移量
void PLMather::calculateAngleInInitilization(PLStructure* plframe1, PLStructure* plframe2) {
    const auto &linematch = plframe1->line_match;
    const auto &scale_lines = plframe1->scale_lines;
    int line_size = scale_lines.size();
    // std::priority_queue<linematch_score, std::vector<linematch_score>, LineMatchCmp> line_pri;
    // for (const auto &match_score : linematch_scores) {
    //     line_pri.push(match_score);
    // }
    // while (true) {
    //     auto linematch = line_pri.top();

    // }
    
    // for (size_t i = 0; i < line_size; ++i) {
        
    // }

    //left
    int match_size = plframe2->point_match.size();
    for (size_t i = 0; i < match_size; i++) {
        if (plframe1->isolate_pts[i] < 0) continue;
        plframe2->scale_pts.keypoints[i].angle = 0;
    }
    //right
    // const auto &pt_match = plframe2->point_match;
    const auto &isolated_pt = plframe2->isolate_pts;
    match_size = plframe2->point_match.size();
    int b_row;
    int b_col;
    const auto &block_h = plframe1->block_h;
    const auto &block_w = plframe1->block_w;
    for (size_t i = 0; i < match_size; i++) {
        if (isolated_pt[i] < 0) continue;
        auto &pt = plframe2->scale_pts.keypoints[i];
        //检索所在块，点的方向设置为块的旋转方向
        //但是块好像不一定对应，块内的线段也不一定对应
        //不，块是隐含对应了，对应的点所在的块也一定对应，不对应的反而更乱了
        b_row = pt.pt.y / block_h;
        b_col = pt.pt.x / block_w;
        pt.angle = plframe2->block[b_row][b_col].angle;
    }
    
    
}