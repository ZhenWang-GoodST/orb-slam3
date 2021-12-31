#ifndef POINT_LINE_BLOCK
#define POINT_LINE_BLOCK

#include <opencv2/opencv.hpp>
#include "LineStructure.h"

//记录本块内看到的所有点线，局部变换关系
//点只记录中心落在块内的
//线段记录本快内的，本块内不足则区域生长取最近的前K条线段，并且标记
class PLBlock
{
private:
    /* data */
public:
    int col;
    int row;
    bool calculated = false;
    // std::vector<int> hierarchy = {};
    // std::vector<ScalePoints> scale_pts = {};
    // std::vector<ScaleLines> scale_lines = {};
    std::vector<int> high_level_pts = {};
    std::vector<int> low_level_pts = {};
    std::vector<std::vector<int>> scale_pts = {};
    std::set<int> scale_lines = {};
    //3 × 4矩阵
    double angle = 0;
    double off_x;
    double off_y;
    //此坐标系按照像素坐标系
    //模板匹配是按照匹配中心点
    cv::Mat transform = cv::Mat::zeros(2, 3, CV_64F);
    PLBlock(/* args */) {};
    ~PLBlock() {};

};

//描述一帧图像所有的点线信息，视为图像描述符
//           col         rol
// typedef std::vector<std::vector<PLBlock>> PLStructure;

#define GRID_COLS 64
#define GRID_ROWS 64
class PLStructure
{
private:
    cv::Mat image;
    // int im_width;
    // int im_height;
    int max_octave = 8;
    int low_level = 4;

public:
    int rows = 5;
    int cols = 10;
    int block_w = 128;
    int block_h = 144;
    int grid_w = 0;
    int grid_h = 0;

    ScalePoints scale_pts = {};
    ScaleLines scale_lines = {};

    std::vector<int> line_match = {};
    std::vector<int> point_match = {};
    std::vector<int> point_match21 = {};
    
    //对于左片点，找到最近的线段，建立孤立点-线段对，关联孤立点与线段，计算方向
    //对于当前一个点，首先找到匹配的左边点，找到左边点对应的线段，根据左边线段匹配索引到右边对应的线段
    //上面不对，应该是直接找到最近的几条线段，把点分配给线段
    //每块找到一个匹配最好的线段，线段周围一定距离的点的方向就设置为这个线段的方向
    //最好：长度相近，中点跟大尺度点变换关系一致，匹配分数高
    //靠近这些线段一定距离的点才分配方向，靠近两个差不多的不分配
    std::vector<int> line_match21 = {};

    //存储点用于计算角度的线段
    std::vector<int> point_line_pair = {};

    //标记是否计算完角度和孤立点-线对
    bool is_angle_calculated = false;
    
    //远离大尺度特征的点-离群点，需要先计算局部变换关系，计算离群点的描述符，再对其进行匹配
    //                    孤立点索引
    //长度等于keypoints，为-1的是已经计算过的点，大于0的记录了当前索引是第几个需要计算descriptor的点，记录了descriptor的位置
    std::vector<int> isolate_pts;

    //   rows        cols
    std::vector<std::vector<PLBlock>> block = {};
    std::vector<std::size_t> plGrid[GRID_COLS][GRID_ROWS];
    PLStructure(){}
    PLStructure(int _rows, int _cols, int _max_octave, cv::Mat _image):
        rows(_rows), cols(_cols), max_octave(_max_octave), image(_image){}

    ~PLStructure() {};

    //复制一份数据，像findContours一样存储其所属父级节点索引，其size内包含子节点索引
    void loadData(const std::vector<cv::KeyPoint> &key_pts, const ScaleLines &lines);
    
    void nonMaxminuSuppression();

    //找到没有线段的块，把最近的线段分配给他
    void spreadLines();

    //内插局部变换关系
    void spreadTransform();

    //更新孤立点并计算描述符,注意按照孤立点的索引
    void computeDescriptor(cv::Ptr<cv::Feature2D> feature_detector);

    double computeDistance(const cv::Mat &left, const cv::Mat &right);

    void computeLocalTransform(
        const PLStructure &left_structure, std::vector<int> line_match, std::vector<cv::DMatch> pt_match);
    
    void computeAngleInIsolatedPoint(bool right = false);

    std::vector<int> GetFeaturesInArea(
        const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel, 
        const std::vector<int> &match_indices);
    
    cv::Point2f transformPoint(const cv::Point2f &pt);
    //debug
    void test();
};

#endif