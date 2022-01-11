#include "Frame.h"

#include "G2oTypes.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "GeometricCamera.h"

#include <thread>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>
#include <bitset>
#include "orb_utils.h"
extern int bit_pattern_31_;
const float factorPI = (float)(CV_PI/180.f);
#define threshold 4
#define UCHAR_DIF(a, b) (a > b ? (a - b) > threshold : (b - a) > threshold)
static void computeOrbDescriptor(const cv::KeyPoint& kpt,
                                     const cv::Mat& img, const cv::Point* pattern,
                                     uchar* desc, uchar* mask)
    {
        float angle = (float)kpt.angle*factorPI;
        float a = (float)cos(angle), b = (float)sin(angle);

        const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
        const int step = (int)img.step;

#define GET_VALUE(idx) \
        center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
               cvRound(pattern[idx].x*a - pattern[idx].y*b)]


        for (int i = 0; i < 32; ++i, pattern += 16)
        {
            int t0, t1, val, dif;
            t0 = GET_VALUE(0); t1 = GET_VALUE(1);
            val = t0 < t1;
            dif = UCHAR_DIF(t0, t1);
            t0 = GET_VALUE(2); t1 = GET_VALUE(3);
            val |= (t0 < t1) << 1;
            dif |= ((bool)UCHAR_DIF(t0, t1)) << 1;
            t0 = GET_VALUE(4); t1 = GET_VALUE(5);
            val |= (t0 < t1) << 2;
            dif |= ((bool)UCHAR_DIF(t0, t1)) << 2;
            t0 = GET_VALUE(6); t1 = GET_VALUE(7);
            val |= (t0 < t1) << 3;
            dif |= ((bool)UCHAR_DIF(t0, t1)) << 3;
            t0 = GET_VALUE(8); t1 = GET_VALUE(9);
            val |= (t0 < t1) << 4;
            dif |= ((bool)UCHAR_DIF(t0, t1)) << 4;
            t0 = GET_VALUE(10); t1 = GET_VALUE(11);
            val |= (t0 < t1) << 5;
            dif |= ((bool)UCHAR_DIF(t0, t1)) << 5;
            t0 = GET_VALUE(12); t1 = GET_VALUE(13);
            val |= (t0 < t1) << 6;
            dif |= ((bool)UCHAR_DIF(t0, t1)) << 6;
            t0 = GET_VALUE(14); t1 = GET_VALUE(15);
            val |= (t0 < t1) << 7;
            dif |= ((bool)UCHAR_DIF(t0, t1)) << 7;

            desc[i] = (uchar)val;
            mask[i] = dif;
        }

#undef GET_VALUE
    }
#define bitcount32(v, result_) v = v - ((v >> 1) & 0x55555555); \
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333); \
        result_ += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24; 


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

int DescriptorDistance(const cv::Mat &a, const cv::Mat &b, unsigned int* mask)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();
    // const int *pma = mask_a.ptr<int32_t>();
    // const int *pmb = mask_b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++, mask++)
    {
        unsigned  int v = *pa ^ *pb;
        // unsigned  int mask = *pma & *pmb;
        std::cout << (bitset<32>)(v) << "\n";
        v = v & *mask;
        std::cout << (bitset<32>)(v) << "\n";
        // std::cout << (bitset<32>)(*pma) << "\n";
        // std::cout << (bitset<32>)(*pmb) << "\n";
        std::cout << (bitset<32>)(*mask) << "\n\n";
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}
int bitCount(unsigned int v) {
    int count = 0;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    return (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
}

int bitCount(unsigned int *b, int length) {
    int count = 0;
    for (int i = 0; i < length; ++i) {
        unsigned int v = *b++;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        count += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }
    return count;
}

int bitAndMask(unsigned int *a, unsigned int *b, unsigned int *mask, int length) {
    int count = 0;
    for (int i = 0; i < length; ++i, ++a, ++b) {
        *mask = *a & *b;
        unsigned int v = *mask++;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        count += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }
    return count;
}
#undef bitcount32
cv::Mat image1;
cv::Mat image2;
cv::Mat show_image1, show_image2;
ORB_SLAM3::ORBextractor* mpIniORBextractor;// = new ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
cv::KeyPoint kpt1, kpt2;
cv::Mat descriptor1(1, 32, CV_8U), descriptor2(1, 32, CV_8U);
cv::Mat descriptor_mask1(1, 32, CV_8U), descriptor_mask2(1, 32, CV_8U);
int orb_distance, orb_distance_mask;
cv::Mat img,imgPoint;  //全局的图像
cv::Point prePoint;  //前一时刻鼠标的坐标，用于绘制直线
void mouse(int event, int x, int y, int flags, void*);
void mouse_left(int event, int x, int y, int flags, void*);
void mouse_right(int event, int x, int y, int flags, void*);
int main() {
    std::cout << "test orb extrator\n";
    mpIniORBextractor = new ORB_SLAM3::ORBextractor(5*1000,2,8,7,7);
    image1 = cv::imread("/home/tonglu/VO-LOAM/github/output/left_level3.png", 0);
    image2 = cv::imread("/home/tonglu/VO-LOAM/github/output/right_level3.png", 0);
    // image1.setTo(0);
    // image2.setTo(0);
    // cv::rectangle(image1, cv::Rect(100, 100, 200, 200), cv::Scalar(255, 0, 0), 30);
    // cv::rectangle(image2, cv::Rect(100, 100, 200, 200), cv::Scalar(255, 0, 0), 30);
    show_image1 = image1.clone();
    show_image2 = image2.clone();
    // img = image1;
    // img.copyTo(imgPoint);
    // cv::imshow("图像窗口1", img);
    // cv::imshow("图像窗口2", imgPoint);
    // cv::setMouseCallback("图像窗口1", mouse,0 );  //鼠标影响
    cv::imshow("left ORB", show_image1);
    cv::imshow("right ORB", show_image2);
    cv::setMouseCallback("left ORB", mouse_left,0 );  //鼠标影响
    cv::setMouseCallback("right ORB", mouse_right,0 );  //鼠标影响
    cv::waitKey(0);
    return 0;
}
void mouse_left(int event, int x, int y, int flags, void*)
{
    show_image1 = image1.clone();
    if (event == cv::EVENT_LBUTTONDOWN)  //单击右键
    {
        cv::Point2f ori(x, y);
        double length = 10;
        double angle = mpIniORBextractor->calAngle(show_image2, ori);
        cout << "calculate left ORB direction:" << angle << endl;
        angle = angle / 180 * M_PI;
        kpt1.pt = ori;
        kpt1.angle = 0;
        computeOrbDescriptor(kpt1, show_image1, &mpIniORBextractor->pattern[0], descriptor1.ptr(), descriptor_mask1.ptr());
        // mpIniORBextractor->computeSingleOrbDescriptor(kpt1, show_image1, descriptor1.ptr());
        cv::line(show_image1, ori, cv::Point2f(x + length * cos(angle), y + length * sin(angle)), cv::Scalar(0, 255, 0));
        cv::imshow("left ORB", show_image1);
    }
}
void mouse_right(int event, int x, int y, int flags, void*)
{
    show_image2 = image2.clone();
    if (event == cv::EVENT_LBUTTONDOWN)  //单击右键
    {
        cv::Point2f ori(x, y);
        double length = 10;
        double angle = mpIniORBextractor->calAngle(show_image2, ori);
        cout << "calculate right ORB direction:" << angle << endl;
        angle = angle / 180 * M_PI;
        kpt2.pt = ori;
        kpt2.angle = 0;
        computeOrbDescriptor(kpt2, show_image2, &mpIniORBextractor->pattern[0], descriptor2.ptr(), descriptor_mask2.ptr());
        // mpIniORBextractor->computeSingleOrbDescriptor(kpt2, show_image2, descriptor2.ptr());
        cv::line(show_image2, ori, cv::Point2f(x + length * cos(angle), y + length * sin(angle)), cv::Scalar(0, 255, 0));
        cv::imshow("right ORB", show_image2);
        if (!descriptor1.empty()) {
            unsigned int mask[8];
            bitAndMask((unsigned int*)descriptor_mask1.ptr(), (unsigned int*)descriptor_mask2.ptr(), mask, 8);
            int good = bitCount(mask, 8);
            orb_distance = DescriptorDistance(descriptor1, descriptor2);
            orb_distance_mask = DescriptorDistance(descriptor1, descriptor2, mask);
            int32_t * mask_ptr = (int32_t *)descriptor_mask2.ptr();
            for (size_t i = 0; i < 8; i++) {
                std::cout << (bitset<32>)(*mask_ptr++) << "\n";
            }
            std::cout << "good: " << good << "\n";
            std::cout << "distance : " << orb_distance << " ratio : " << orb_distance / 256.0 << "\n";
            std::cout << "distance : " << orb_distance_mask << " ratio : " << orb_distance_mask * 1.0 / good << "\n";
        }
        
    }
}
void mouse(int event, int x, int y, int flags, void*)
{
    if (event == cv::EVENT_RBUTTONDOWN)  //单击右键
    {
        cout << "点击鼠标左键才可以绘制轨迹" << endl;
    }
    if (event == cv::EVENT_LBUTTONDOWN)  //单击左键，输出坐标
    {
        prePoint = cv::Point(x, y);
        cout << "轨迹起使坐标" << prePoint << endl;

    }
    if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON))  //鼠标按住左键移动
    {
        //通过改变图像像素显示鼠标移动轨迹
        imgPoint.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
        imgPoint.at<cv::Vec3b>(y, x-1) = cv::Vec3b(0, 0, 255);
        imgPoint.at<cv::Vec3b>(y, x+1) = cv::Vec3b(0, 0, 255);
        imgPoint.at<cv::Vec3b>(y+1, x) = cv::Vec3b(0, 0, 255);
        imgPoint.at<cv::Vec3b>(y+1, x) = cv::Vec3b(0, 0, 255);
        cv::imshow("图像窗口2", imgPoint);

        //通过绘制直线显示鼠标移动轨迹
        cv::Point pt(x, y);
        cv::line(img, prePoint, pt, cv::Scalar(0, 0, 255), 2, 5, 0);
        prePoint = pt;
        cv::imshow("图像窗口1", img);
    }
}