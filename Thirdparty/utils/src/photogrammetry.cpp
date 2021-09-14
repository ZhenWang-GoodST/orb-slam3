#include "photogrammetry.h"
#include "coverage_utils.h"

namespace tergeo {
namespace visualodometry {

template <typename T>
void getRotationMatrix3d(T &R, 
    double fai, double omg, double kap) {
    double cosf = cos(fai), coso = cos(omg), cosk = cos(kap);
    double sinf = sin(fai), sino = sin(omg), sink = sin(kap);
    R(0, 0) = cosf * cosk - sinf * sino * sink;
    R(0, 1) = - cosf * sink - sinf * sino * cosk;
    R(0, 2) = - sinf * coso;
    R(1, 0) = coso * sink;
    R(1, 1) = coso * cosk;
    R(1, 2) = - sino;
    R(2, 0) = sinf * cosk + cosf * sino * sink;
    R(2, 1) = - sinf * sink + cosf * sino * cosk;
    R(2, 2) = cosf * coso;
}


// bool stop

//按照摄影测量书上严密公式，书上ｘ为运动方向，采集数据运动方向为ｙ轴
//所以设BY = -1,bx,bz为０，算出结果应该是bx = a > 0, bz = 0
bool relativeOrientation(
    std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2,
    Eigen::MatrixXd &R, Eigen::MatrixXd &T, Eigen::MatrixXd K,
    double &fai1, double &omg1, double &kap1,
    double &fai2, double &omg2, double &kap2) {
    
    int pt_size = pts1.size();
    double BX = 0.1, BY = -1, BZ = 0.1;
    std::vector<Eigen::Vector3d> X1 = {}, Y1 = {}, Z1 = {};
    std::vector<Eigen::Vector3d> X2 = {}, Y2 = {}, Z2 = {};
    Eigen::VectorXd L(pt_size);
    Eigen::Matrix3d R1, R2;
    Eigen::MatrixXd A, B;
    R1 = Eigen::MatrixXd::Identity(3, 3);
    A = Eigen::MatrixXd::Zero(pt_size, 4 * pt_size);
    B = Eigen::MatrixXd::Zero(pt_size, 5);
    Eigen::Matrix3d MTMP;// 求行列式
    Eigen::Vector3d BVec;// 求行列式
    int loop_count = 10;
    Eigen::MatrixXd P1 = Eigen::MatrixXd::Identity(pt_size * 4, pt_size * 4);
    Eigen::MatrixXd P2 = Eigen::MatrixXd::Identity(pt_size * 4, pt_size * 4);
    std::vector<double> co_planner_error(pt_size, 0);
    fai1 = 0; omg1 = 0; kap1 = 0;
    fai2 = 0; omg2 = 0; kap2 = 0;
    std::cout << K.inverse() << "\n";
    double max_coplan_error = std::numeric_limits<double>::max();
    bool isinthresh = false;
    for (int i = 0; i < loop_count; i++) {
        // getRotationMatrix3d(R1, fai1, omg1, kap1);
        std::cout << R1 << "\n";
        getRotationMatrix3d<Eigen::Matrix3d>(R2, fai2, omg2, kap2);
        Eigen::Matrix3Xd x1_(3, pt_size), x2_(3, pt_size);
        for (int i = 0; i < pt_size; ++i) {
            x1_(0, i) = pts1[i].x;
            x1_(1, i) = pts1[i].y;
            x1_(2, i) = 1;
            x2_(0, i) = pts2[i].x;
            x2_(1, i) = pts2[i].y;
            x2_(2, i) = 1;
        }
        Eigen::Matrix3Xd X1_ = R1 * K.inverse() * x1_;
        Eigen::Matrix3Xd X2_ = R2 * K.inverse() * x2_;
        BVec(0, 0) = BX;
        BVec(1, 0) = BY;
        BVec(2, 0) = BZ;
        std::cout << X1_ << "\n";
        std::cout << X2_ << "\n";
        
        for (int i = 0; i < pt_size; ++i) {
            double x1 = X1_(0, i), y1 = X1_(1, i), z1 = X1_(2, i);
            double x2 = X2_(0, i), y2 = X2_(1, i), z2 = X2_(2, i);
            MTMP.row(0) = BVec.transpose();
            MTMP.row(1) = R1.col(0).transpose();    
            MTMP.row(2) = X2_.col(i).transpose();
            double detXXZZ = y1 * z2 - y2 * z1;
            A(i, i * 4 + 0) = MTMP.determinant() / detXXZZ; 
            std::cout << y1 << " " << z2 << " " << y2 << " " << z1 << "\n";
            std::cout << MTMP.determinant() << " " << detXXZZ << "\n";
            std::cout << A(i, i * 4 + 0) << "\n";
            MTMP.row(1) = R1.col(1).transpose();  
            A(i, i * 4 + 1) = MTMP.determinant(); 
            // std::cout << MTMP << "\n";
            MTMP.row(1) = X1_.col(i).transpose();
            MTMP.row(2) = R2.col(0).transpose();  
            A(i, i * 4 + 2) = MTMP.determinant(); 
            MTMP.row(2) = R2.col(1).transpose();  
            A(i, i * 4 + 3) = MTMP.determinant(); 
            B(i, 0) = -(y1 * z2 - y2 * z1);
            B(i, 1) = - (x1 * y2 - x2 * y1);
            //一阶近似
            MTMP(2, 0) = - z2; MTMP(2, 1) = 0; MTMP(2, 2) = x2;
            B(i, 2) = - MTMP.determinant();
            //一阶近似
            MTMP(2, 0) = 0; 
            MTMP(2, 1) = - z2; 
            MTMP(2, 2) = y2;
            B(i, 3) = - MTMP.determinant() / detXXZZ;
            //一阶近似
            MTMP(2, 0) = - y2; 
            MTMP(2, 1) = x2; 
            MTMP(2, 2) = 0;
            B(i, 4) = - MTMP.determinant();
            std::cout << MTMP << " \n" << MTMP.determinant() << "\n";
            double nq1 = (BX * z2 - BZ * x1) / detXXZZ;
            double nq2 = (BX * z2 - BZ * x2) / detXXZZ;
            // L(i) = nq1 * y1 - nq2 * y2 - BY;
            MTMP.row(0) = BVec.transpose();
            MTMP.row(1) = X1_.col(i).transpose();
            MTMP.row(2) = X2_.col(i).transpose();
            L(i) = MTMP.determinant();
            co_planner_error[i] = std::abs(MTMP.determinant());
        }
        double tmperror = std::accumulate(co_planner_error.begin(), co_planner_error.end(), 0.0);
        std::cout << tmperror << "\n";
        Eigen::MatrixXd N1 = A * P1.inverse() * A.transpose(); 
        Eigen::MatrixXd N = B.transpose() * N1.inverse() * B;
        Eigen::MatrixXd X = N.inverse() * B.transpose() * N1.inverse() * L;
        N = B.transpose() * B;
        X = N.inverse() * B.transpose() * L;
        std::cout << N << "\n";
        std::cout << B << "\n";
        std::cout << L << "\n\n";
        std::cout << N1.determinant() << "\n";
        std::cout << N.determinant() << "\n";
        std::cout << N1.determinant() << "\n";
        bool isinthres = std::abs(X(0)) < 3e-5 && std::abs(X(1)) < 3e-5 &&
            std::abs(X(2)) < 3e-5 && std::abs(X(3)) < 3e-5 && 
            std::abs(X(4)) < 3e-5;
        if (tmperror >  max_coplan_error && i > 3) {
            break;
        } else {
            max_coplan_error = tmperror;
        }
        BX += X(0);
        BZ += X(1);
        fai2 += X(2);
        omg2 += X(3);
        kap2 += X(4);
        std::cout << X << "\n";
        if (isinthres) {
            break;
        }
    }
    getRotationMatrix3d<Eigen::MatrixXd>(R, fai2, omg2, kap2);
    T = BVec / BVec.dot(BVec);
    std::cout << fai2 << " " << omg2 << " " << kap2 << "\n";
    std::cout << "R:\n" << R << "\n\nT:\n" << T << "\n";
    return isinthresh;
}

bool forwardIntersection(
    const cv::Mat &cR1, const cv::Mat &cT1, const cv::Mat &cR2, const cv::Mat &cT2, const cv::Mat &K,
    const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2,
    std::vector<cv::Point3f> &ThDPts, int mode) {
    
    //转化为摄影测量的ＲＴ，
    cv::Mat R1 = cR1.inv();
    cv::Mat R2 = cR2.inv();
    cv::Mat T1 = -R1 * cT1;
    cv::Mat T2 = -R2 * cT2;
    std::cout << T2 << "\n";
    FINProjection(R1, T1, R2, T2, K, p1, p2, ThDPts);
    return true;
}

bool FINProjection(
    const cv::Mat &R1, const cv::Mat &T1, const cv::Mat &R2, const cv::Mat &T2, const cv::Mat &K,
    const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2,
    std::vector<cv::Point3f> &ThDPts){
    
    std::cout << K << "\n";
    int pt_size = p1.size();
    std::cout << R1.type() << " " << T1.type() << " " << R2.type() << " " << T2.type() << " " << K.type() << "\n";
    cv::Mat pixMat1 = cv::Mat::zeros(cv::Size(pt_size, 3), CV_64F);
    cv::Mat pixMat2 = cv::Mat::zeros(cv::Size(pt_size, 3), CV_64F);
    for (int i = 0; i < pt_size; ++i) {
        pixMat1.at<double>(0, i) = p1[i].x;
        pixMat2.at<double>(0, i) = p2[i].x;
        pixMat1.at<double>(1, i) = p1[i].y;
        pixMat2.at<double>(1, i) = p2[i].y;
        pixMat1.at<double>(2, i) = 1;
        pixMat2.at<double>(2, i) = 1;
    }
    //转换到像空间辅助坐标系
    std::cout << K.inv() << "\n\n";
    std::cout << K.inv() * pixMat1 << "\n\n";
    std::cout << K.inv() * pixMat2 << "\n\n";
    std::cout << R1 << "\n\n" << R2 << "\n\n";
    pixMat1 = R1 * K.inv() * pixMat1;
    pixMat2 = R2 * K.inv() * pixMat2;
    const cv::Mat &CAuCoor1 = pixMat1;
    const cv::Mat &CAuCoor2 = pixMat2;
    //计算点投影系数
    cv::Mat T1Mat = cv::Mat::zeros(cv::Size(pt_size, 3), CV_64F);
    cv::Mat T2Mat = cv::Mat::zeros(cv::Size(pt_size, 3), CV_64F);
    for (int i = 0; i < pt_size; ++i) {
        T1.copyTo(T1Mat.col(i));
        T2.copyTo(T2Mat.col(i));
    }
    cv::Mat BMat = T2Mat - T1Mat;
    std::cout << BMat << "\n\n";
    cv::Mat det = pixMat1.row(0).mul(pixMat2.row(2)) - pixMat1.row(2).mul(pixMat2.row(0)); 
    cv::Mat N1 = (BMat.row(0).mul(pixMat2.row(2)) - BMat.row(2).mul(pixMat2.row(0))) / det;
    cv::Mat N2 = (BMat.row(0).mul(pixMat1.row(2)) - BMat.row(2).mul(pixMat1.row(0))) / det;
    cv::Mat N1Mat = cv::Mat::zeros(cv::Size(pt_size, 3), CV_64F);
    cv::Mat N2Mat = cv::Mat::zeros(cv::Size(pt_size, 3), CV_64F);
    for (int i = 0; i < 3; ++i) {
        N1.copyTo(N1Mat.row(i));
        N2.copyTo(N2Mat.row(i));
    }
    cv::Mat ThreeDPtMat = cv::Mat::zeros(cv::Size(pt_size, 4), CV_64F);
    std::cout << N1Mat << "\n\n" << N2Mat << "\n\n";
    ThreeDPtMat = T1Mat + N1Mat.mul(pixMat1);
    cv::Mat N2X2 = T2Mat.row(0) + N2.mul(pixMat2.row(0));
    cv::Mat ErrorX = ThreeDPtMat.row(1) - N2X2;
    ThreeDPtMat.row(0) = 0.5 * (ThreeDPtMat.row(0) + N2X2);
    std::cout << ThreeDPtMat << "\n\n";
    std::cout << ErrorX << "\n\n";
    // ThDPts.clear();
    for (int i = 0; i < pt_size; ++i) {
        double x = ThreeDPtMat.at<double>(0, i);
        double y = ThreeDPtMat.at<double>(1, i);
        double z = ThreeDPtMat.at<double>(2, i);
        ThDPts.push_back(cv::Point3f(x, y, z));
    }
    
    // ThreeDPtMat.row(0) = T1Mat.row(0) + N1.mul(pixMat1.row(0));
    // ThreeDPtMat.row(1) = T1Mat.row(1) + N1.mul(pixMat1.row(1));
    // ThreeDPtMat.row(2) = T2Mat.row(0) + N2.mul(pixMat2.row(0));
    // ThreeDPtMat.row(3) = ThreeDPtMat.row(1) - ThreeDPtMat.row(2);
    // ThreeDPtMat.row(1) = 0.5 * (ThreeDPtMat.row(1) + ThreeDPtMat.row(2));
    // ThreeDPtMat.row(3) = T1Mat.row(0) + N1.mul(pixMat1.row(0));
    return true;
}

}
}