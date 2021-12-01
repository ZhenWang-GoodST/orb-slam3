#include "computervision.h"
#include "opencv_utils.h"
#include "coverage_utils.h"

namespace tergeo {
namespace visualodometry {


cv::Point2d predicteHPoint(const cv::Point2d &pt, const cv::Mat &H, int mode) {
    cv::Mat pt3 = cv::Mat::zeros(3, 1, CV_64F);
    pt3.at<double>(0, 0) = pt.x;
    pt3.at<double>(1, 0) = pt.y;
    pt3.at<double>(2, 0) = 1;
    cv::Mat ptMat;
    if (mode == 0) {
        ptMat = H * pt3;
    } else if(mode == 1){
        ptMat = H.inv() * pt3;
    } else {
        return cv::Point2d(0, 0);
    }
    return cv::Point2d(ptMat.at<double>(0) / ptMat.at<double>(2), 
                       ptMat.at<double>(1) / ptMat.at<double>(2));
}

bool reprojectionError(
    const std::vector<cv::Point3f> &pt3d,
    const std::vector<cv::Point2f> &pts1, const std::vector<cv::Point2f> &pts2,
    Eigen::MatrixXd &R1, Eigen::MatrixXd &T1, 
    Eigen::MatrixXd &R2, Eigen::MatrixXd &T2,
    Eigen::MatrixXd &K, double f, 
    std::vector<double> &perror1, std::vector<double> &perror2) {
    
    int pt_size = pt3d.size();
    Eigen::Matrix4Xd HTDPTS(4, pt_size);
    Eigen::Matrix3Xd PIXEL1(3, pt_size), PIXEL2(3, pt_size);
    for (int i = 0; i < pt_size; ++i) {
        HTDPTS(0, i) = pt3d[i].x;
        HTDPTS(1, i) = pt3d[i].y;
        HTDPTS(2, i) = pt3d[i].z;
        HTDPTS(3, i) = 1;
    }
    Eigen::Matrix3Xd RT1(3, 4), RT2(3, 4);
    RT1.block(0, 0, 3, 3) = R1;
    RT2.block(0, 0, 3, 3) = R2;
    RT1.col(3) = T1; RT2.col(3) = T2;
    PIXEL1 = K * RT1 * HTDPTS;
    PIXEL2 = K * RT2 * HTDPTS;
    perror1.clear();
    std::cout << RT1 << "\ntest\n" << RT2 << "\n\n";
    // std::cout << PIXEL1 << "\n\n" << PIXEL2 << "\n\n";
    for (int i = 0; i < pt_size; ++i) {
        std::cout << PIXEL1(0, i) << " " << PIXEL1(1, i) << " " << PIXEL1(2, i) << "\n";
    }
    for (int i = 0; i < pt_size; ++i) {
        std::cout << PIXEL2(0, i) << " " << PIXEL2(1, i) << " " << PIXEL2(2, i) << "\n";
    }
    
    std::vector<double> errorvx1 = {}, errorvx2 = {};
    std::vector<double> errorvy1 = {}, errorvy2 = {};
    for (int i = 0; i < pt_size; ++i) {
        double errorx1 = pts1[i].x - PIXEL1(0, i) / PIXEL1(2, i);
        double errory1 = pts1[i].y - PIXEL1(1, i) / PIXEL1(2, i);
        double errorx2 = pts2[i].x - PIXEL2(0, i) / PIXEL2(2, i);
        double errory2 = pts2[i].y - PIXEL2(1, i) / PIXEL2(2, i);
        double error1 = std::sqrt(errorx1 * errorx1 + errory1 * errory1);
        double error2 = std::sqrt(errorx2 * errorx2 + errory2 * errory2);
        
        std::cout << i << " " 
            << error1 << " " << error2 << " " 
            << errorx1 << " " << errory1 << " " 
            << errorx2 << " " << errory2<< "\n";
        perror1.push_back(error1);
        perror2.push_back(error2);
        // errorvx1.push_back(errorx1);
        // errorvx2.push_back(errorx2);
        // errorvy1.push_back(errory1);
        // errorvy2.push_back(errory2);
    }
    printVec<double>(perror1);
    return true;
    
}

bool getRTByEssential(cv::Mat& K, const std::vector<cv::Point2f>& p1, const std::vector<cv::Point2f>& p2, cv::Mat& R, cv::Mat& T, cv::Mat& mask)
{
	//根据内参矩阵获取相机的焦距和光心坐标（主点坐标）
	double focal_length = 0.5*(K.at<double>(0) + K.at<double>(4));
	cv::Point2d principle_point(K.at<double>(2), K.at<double>(5));
    // std::cout << focal_length << principle_point << "\n";
	//根据匹配点求取本征矩阵，使用RANSAC，进一步排除失配点
	cv::Mat E = findEssentialMat(p1, p2, focal_length, principle_point, cv::RANSAC, 0.999, 3.0, mask);
	// E = cv::findHomography ( p1, p2, cv::RANSAC, 3, noArray(), 2000, 0.99 );
	if (E.empty()) {
		std::cout << "return E.empty()\n";
	    return false;	
	} 
	double feasible_count = countNonZero(mask);
	std::cout << (int)feasible_count << " -in- " << p1.size() << std::endl;
	//对于RANSAC而言，outlier数量大于50%时，结果是不可靠的
	if (feasible_count < 6 || (feasible_count / p1.size()) < 0.5) {
        std::cout << "too few or small success percentage points\n";
		return false;
	}
	// if (feasible_count <= 15 || (feasible_count / p1.size()) < 0.6)

	//分解本征矩阵，获取相对变换
	// std::cout << E.size() << "E SIZE\n"; 
	int pass_count = recoverPose(E, p1, p2, R, T, focal_length, principle_point, mask);
    std::cout << R << "\n\n" << T << "\n\n";
	//同时位于两个相机前方的点的数量要足够大
	if (((double)pass_count) / feasible_count < 0.8)
		return false;
    
	return true;
}

bool getRTByHomography(cv::Mat& K, const std::vector<cv::Point2f>& p1, 
    const std::vector<cv::Point2f>& p2, std::vector<cv::Point3f> D3pts, 
    cv::Mat& R, cv::Mat& T, cv::Mat &H, cv::Mat& mask, int mode)
{
    int pt_size = p1.size();
	//根据内参矩阵获取相机的焦距和光心坐标（主点坐标）
	double focal_length = 0.5*(K.at<double>(0) + K.at<double>(4));
	cv::Point2d principle_point(K.at<double>(2), K.at<double>(5));
	//根据匹配点求取单应矩阵，使用RANSAC，进一步排除失配点
	H = cv::findHomography (p1, p2, cv::RANSAC, 3, mask, 2000, 0.99 );
	if (H.empty()) {
		std::cout << "return H.empty()\n";
	    return false;	
	}
    
    std::cout << "H: " << H <<"\n";
    printCVPoint<cv::Point2f>(p1, p2);
	double feasible_count = countNonZero(mask);
	std::cout << (int)feasible_count << " -in- " << p1.size() << std::endl;
	//对于RANSAC而言，outlier数量大于50%时，结果是不可靠的
	// if (feasible_count < 6 || (feasible_count / p1.size()) < 0.5) {
    //     std::cout << "too few or small success percentage points\n";
	// 	return false;
	// }
	//分解单应矩阵，获取相对变换
    std::vector<cv::Mat> decompose_R = {}, decompose_T = {}, decompose_N = {};
    int solutions = cv_wz::decomposeHomographyMat(H, K, decompose_R, decompose_T, decompose_N, mode);
    
    //检查分解RT
    //1.首先检查法线，相机与平面平行，法线应该为(0, 0, 1)T,与先验法线做内积，取最大的前两个
    cv::Mat prior_norm = cv::Mat(cv::Matx31d(0, 0, 1));
    typedef std::pair<int, double> dot_pair;
    auto cmpdot=[](dot_pair l, dot_pair r) {return l.second < r.second;};
    std::priority_queue<dot_pair, std::vector<dot_pair>, decltype(cmpdot)> priori_dot(cmpdot);
    for (int i = 0; i < solutions; ++i) {
        priori_dot.push(std::make_pair(i, std::abs(decompose_N[i].dot(prior_norm))));
        std::cout << decompose_T[i] << "\n";
    }
    //２.检查点是否都在相机前方，景深/深度都大于０
    std::vector<cv::Point2f> _p1 = {}, _p2 = {};
    std::cout << mask.type() << "\n";
    for (size_t i = 0; i < pt_size; ++i) {
        if (mask.at<uchar>(i) == 1) {
            _p1.push_back(p1[i]);
            _p2.push_back(p2[i]);
        }
    }
    
    // for (int i = 0; i < 4; ++i) {
    //     std::cout << decompose_R[i].t() * decompose_R[i] << "\n";
    //     decompose_R[i] = decompose_R[i].inv();
    // }
    
    std::priority_queue<dot_pair, std::vector<dot_pair>, decltype(cmpdot)> depth_ratio(cmpdot);
    std::map<int, std::vector<cv::Point3f>> _D3Pts = {};
    for (int i = 0; i < 2; ++i) {
        int id = priori_dot.top().first;
        const cv::Mat &tR = decompose_R[id];
        const cv::Mat &tT = decompose_T[id];
        //三维重建
        //求出左右片到像素坐标系的投影矩阵
        cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
        cv::Mat P2(3, 4, CV_32F, cv::Scalar(0));
        K.copyTo(P1.rowRange(0, 3).colRange(0, 3));
        tR.copyTo(P2.rowRange(0, 3).colRange(0, 3));
        tT.copyTo(P2.rowRange(0, 3).col(3));
        cv::Mat _K;
        K.convertTo(_K, CV_32F);
        P2 = _K * P2;
        //三角化
        cv::Mat D3PtsInMat1, D3PtsInMat2;
        cv::triangulatePoints(P1, P2, _p1, _p2, D3PtsInMat1);
        _D3Pts[id] = std::vector<cv::Point3f>();
        std::cout << D3PtsInMat1 << "\n";
        for (int j = 0; j < D3PtsInMat1.cols; ++j) {
            D3PtsInMat1.at<float>(0, j) = D3PtsInMat1.at<float>(0, j) / D3PtsInMat1.at<float>(3, j);
            D3PtsInMat1.at<float>(1, j) = D3PtsInMat1.at<float>(1, j) / D3PtsInMat1.at<float>(3, j);
            D3PtsInMat1.at<float>(2, j) = D3PtsInMat1.at<float>(2, j) / D3PtsInMat1.at<float>(3, j);
            _D3Pts[id].push_back(cv::Point3f(D3PtsInMat1.at<float>(0, j),
                D3PtsInMat1.at<float>(1, j), D3PtsInMat1.at<float>(2, j)));
        }
        std::cout << D3PtsInMat1 << "\n";
        //检查点是否都在相机前方
        int errorCount = 0;
        std::vector<int> tMask(p1.size(), 1);
        cv::Mat _tR;
        tR.convertTo(_tR, CV_32F);
        cv::Mat _ttT;
        tT.convertTo(_ttT, CV_32F);
        cv::Mat _tT = cv::Mat::zeros(cv::Size(D3PtsInMat1.cols, 3), CV_32F);
        for (int j = 0; j < D3PtsInMat1.cols; ++j) {
            _ttT.copyTo(_tT.col(j));
        }
        std::cout << D3PtsInMat1.type() << "\n" << tR.type() << "\n";
        D3PtsInMat2 = _tR * D3PtsInMat1.rowRange(0, 3).colRange(0, D3PtsInMat1.cols) + _tT; 
        std::cout << "\n" << D3PtsInMat2 << "\n";
        for (int j = 0; j < D3PtsInMat1.cols; ++j) {
            if(D3PtsInMat1.at<float>(2, j) <= 0 || D3PtsInMat2.at<float>(2, j) <= 0) {
                tMask[j] = 0;
            }
        }
        double ratio = std::accumulate(tMask.begin(), tMask.end(), 0) * 1.0 / tMask.size();
        depth_ratio.push(std::make_pair(id, ratio));
        priori_dot.pop();
    }
    dot_pair pair1, pair2;
    pair1 = depth_ratio.top();
    depth_ratio.pop(); 
    pair2 = depth_ratio.top();
    if (pair1.second < 0.7 && pair1.second * 0.75 < pair2.second) {
        return false;
    } 
    R = decompose_R[pair1.first];
    T = decompose_T[pair1.first];
    D3pts = _D3Pts[pair1.first];
    // cv::Mat = T * decompose_N[pair1.first].t();
    // cv::Mat KH = K.inv() * H * K;
    // double dH = 1 - R.at<double>(2, 2);
    cv::Mat mean, stddev;
    cv::Mat D3PTSM = cv::Mat::zeros(cv::Size(feasible_count, 1), CV_32F);
    std::cout <<  D3PTSM << "\n";
    for (int i = 0; i < feasible_count; ++i) {
        D3PTSM.at<float>(0, i) = D3pts[i].z;
    }
    // mean = cv::mean(D3PTSM);
    cv::meanStdDev(D3PTSM, mean, stddev);
    std::cout << mean << " \n\n" << stddev << "\n";
    std::cout << R << "\n\n " << T << "\n";
    std::cout << decompose_R[pair1.first] << "\n\n " << decompose_T[pair1.first] << "\n";
    std::cout << R.t() * T << "\n";
	return true;
}


bool HReprojectionError(
        std::vector<cv::Point2f> p1, std::vector<cv::Point2f> p2,
        const cv::Mat &H, std::vector<cv::Point2f> &HP2, std::vector<int> &query, std::vector<double> &herror, double thres) {
    int pt_size = p1.size();
    herror.clear();
    herror = std::vector<double>(pt_size, -1);
    cv::Mat p1Mat = cv::Mat::zeros(cv::Size(p1.size(), 3), CV_64F);
    for (int i = 0; i < pt_size; ++i) {
        p1Mat.at<double>(0, i) = p1[i].x;
        p1Mat.at<double>(1, i) = p1[i].y;
        p1Mat.at<double>(2, i) = 1;
    }
    cv::Mat p2ProjMat = H * p1Mat;
    std::cout << H << "\n";
    for (int i = 0; i < pt_size; ++i) {
        p2ProjMat.at<double>(0, i) /= p2ProjMat.at<double>(2, i);
        p2ProjMat.at<double>(1, i) /= p2ProjMat.at<double>(2, i);
        double dx = p2ProjMat.at<double>(0, i) - p2[i].x;
        double dy = p2ProjMat.at<double>(1, i) - p2[i].y;
        std::cout << p2ProjMat.col(i) << "\n";
        HP2[i] = cv::Point2f(p2ProjMat.at<double>(0, i), p2ProjMat.at<double>(1, i));
        double dis = std::sqrt(dx * dx + dy * dy);
        herror[i] = dis;
        // if(query[i] == 0) continue;
        if (dis > thres) query[i] = 0;
    }
    return true;
}

bool ReconstructH(
    std::vector<cv::Point2f> pt1, std::vector<cv::Point2f> pt2, 
    Match matchpair,
    std::vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
    cv::Mat &R21, cv::Mat &t21, std::vector<cv::Point3f> &vP3D, 
    std::vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
    if(vbMatchesInliers[i])
        N++;//匹配点对 内点

    // 8种运动假设  We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988
    
        // 因为特征点是图像坐标系，所以将H矩阵由相机坐标系换算到图像坐标系
    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);
    
        // SVD分解的正常情况是特征值降序排列
    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
    return false;// 初始化失败
    }

   std::vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
        // 法向量n'= [x1 0 x3] 对应ppt的公式17
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    // 计算ppt中公式19
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};
    // 计算旋转矩阵 R‘，计算ppt中公式18
    //          | ctheta         0   -aux_stheta|         | aux1|
    // Rp = |    0               1       0             |  tp = |  0     |
    //          | aux_stheta  0    ctheta       |         |-aux3|

    //          | ctheta          0    aux_stheta|          | aux1|
    // Rp = |    0                1       0              |  tp = |  0  |
    //          |-aux_stheta  0    ctheta         |          | aux3|

    //          | ctheta         0    aux_stheta|         |-aux1|
    // Rp = |    0               1       0             |  tp = |  0     |
    //          |-aux_stheta  0    ctheta       |         |-aux3|

    //          | ctheta         0   -aux_stheta|         |-aux1|
    // Rp = |    0               1       0             |  tp = |  0  |
    //          | aux_stheta  0    ctheta       |          | aux3|

    for(int i=0; i<4; i++)
    {
    cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
    Rp.at<float>(0,0)=ctheta;
    Rp.at<float>(0,2)=-stheta[i];
    Rp.at<float>(2,0)=stheta[i];
    Rp.at<float>(2,2)=ctheta;

    cv::Mat R = s*U*Rp*Vt;
    vR.push_back(R);

    cv::Mat tp(3,1,CV_32F);
    tp.at<float>(0)=x1[i];
    tp.at<float>(1)=0;
    tp.at<float>(2)=-x3[i];
    tp*=d1-d3;
    
    // 这里虽然对t有归一化，并没有决定单目整个SLAM过程的尺度
    // 因为CreateInitialMapMonocular函数对3D点深度会缩放，然后反过来对 t 有改变
    cv::Mat t = U*tp;
    vt.push_back(t/cv::norm(t));

    cv::Mat np(3,1,CV_32F);
    np.at<float>(0)=x1[i];
    np.at<float>(1)=0;
    np.at<float>(2)=x3[i];

    cv::Mat n = V*np;
    if(n.at<float>(2)<0)
        n=-n;
    vn.push_back(n);
    }

    //case d'=-d2
    // 计算ppt中公式22
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};
    
            // 计算旋转矩阵 R‘，计算ppt中公式21
    for(int i=0; i<4; i++)
    {
    cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
    Rp.at<float>(0,0)=cphi;
    Rp.at<float>(0,2)=sphi[i];
    Rp.at<float>(1,1)=-1;
    Rp.at<float>(2,0)=sphi[i];
    Rp.at<float>(2,2)=-cphi;

    cv::Mat R = s*U*Rp*Vt;
    vR.push_back(R);

    cv::Mat tp(3,1,CV_32F);
    tp.at<float>(0)=x1[i];
    tp.at<float>(1)=0;
    tp.at<float>(2)=x3[i];
    tp*=d1+d3;

    cv::Mat t = U*tp;
    vt.push_back(t/cv::norm(t));

    cv::Mat np(3,1,CV_32F);
    np.at<float>(0)=x1[i];
    np.at<float>(1)=0;
    np.at<float>(2)=x3[i];

    cv::Mat n = V*np;
    if(n.at<float>(2)<0)
        n=-n;
    vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    std::vector<cv::Point3f> bestP3D;
    std::vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    // d'=d2和d'=-d2分别对应8组(R t)
    float mSigma2 = 4;
    for(size_t i=0; i<8; i++)
    {
    float parallaxi;
    std::vector<cv::Point3f> vP3Di;
    std::vector<bool> vbTriangulatedi;
    int nGood;
    // nGood = CheckRT(vR[i],vt[i],pt1,pt2,matchpair,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);
            // 保留最优的和次优的
    if(nGood>bestGood)
    {
        secondBestGood = bestGood;
        bestGood = nGood;
        bestSolutionIdx = i;
        bestParallax = parallaxi;
        bestP3D = vP3Di;
        bestTriangulated = vbTriangulatedi;
    }
    else if(nGood>secondBestGood)
    {
        secondBestGood = nGood;
    }
    }


    if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
    {
    vR[bestSolutionIdx].copyTo(R21);
    vt[bestSolutionIdx].copyTo(t21);
    vP3D = bestP3D;
    vbTriangulated = bestTriangulated;

    return true;// 初始化成功
    }

    return false;// 初始化失败
}

int CheckRT(const cv::Mat &R, const cv::Mat &t, const std::vector<cv::KeyPoint> &vKeys1, const std::vector<cv::KeyPoint> &vKeys2,
			      const std::vector<std::pair<int, int>> &vMatches12, std::vector<bool> &vbMatchesInliers,
			      const cv::Mat &K, std::vector<cv::Point3f> &vP3D, float th2, std::vector<bool> &vbGood, float parallax)
	{
	    // Calibration parameters
	   // 校正参数
	    const float fx = K.at<float>(0,0);
	    const float fy = K.at<float>(1,1);
	    const float cx = K.at<float>(0,2);
	    const float cy = K.at<float>(1,2);
 
	    vbGood = std::vector<bool>(vKeys1.size(),false);
	    vP3D.resize(vKeys1.size());// 对应的三维点
 
	    std::vector<float> vCosParallax;
	    vCosParallax.reserve(vKeys1.size());
 
	    // Camera 1 Projection Matrix K[I|0]
// 步骤1：得到一个相机的投影矩阵
            // 以第一个相机的光心作为世界坐标系	    
	    // 相机1  变换矩阵 在第一幅图像下 的变换矩阵  Pc1  =   Pw  =  T1 * Pw      T1 = [I|0]
	    // Pp1  = K *  Pc1 = K * T1 * Pw  =   [K|0] *Pw  = P1 × Pw
	    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
	    K.copyTo(P1.rowRange(0,3).colRange(0,3));
            // 第一个相机的光心在世界坐标系下的坐标
	    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);// 相机1原点 000
	    
// 步骤2：得到第二个相机的投影矩阵
	    // Camera 2 Projection Matrix K[R|t]
	    // 相机2  变换矩阵  Pc2  =   Pw  =  T2 * Pw      T2 = [R|t]
	    // Pp2  = K *  Pc2 = K * T2 * Pw  =  K* [R|t] *Pw  = P2 × Pw 
	    cv::Mat P2(3,4,CV_32F);
	    R.copyTo(P2.rowRange(0,3).colRange(0,3));
	    t.copyTo(P2.rowRange(0,3).col(3));
	    P2 = K*P2;
            // 第二个相机的光心在世界坐标系下的坐标
	    cv::Mat O2 = -R.t()*t;//相机2原点  R逆 * - t  R 为正交矩阵  逆 = 转置
 
	    int nGood=0;
 
	    for(size_t i=0, iend=vMatches12.size();i<iend;i++)// 每一个匹配点对
	    {
		if(!vbMatchesInliers[i])// 离线点  非内点
		    continue;
               // kp1和kp2是匹配特征点
		const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
		const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
		cv::Mat p3dC1;
		
// 步骤3：利用三角法恢复三维点p3dC1
		// kp1 = P1 * p3dC1     kp2 = P2 * p3dC1   
		Triangulate(kp1,kp2,P1,P2,p3dC1);
 
		if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
		{// 求出的3d点坐标 值有效
		    vbGood[vMatches12[i].first]=false;
		    continue;
		}
		
// 步骤4：计算视差角余弦值
		// Check parallax
		cv::Mat normal1 = p3dC1 - O1;
		float dist1 = cv::norm(normal1);
 
		cv::Mat normal2 = p3dC1 - O2;
		float dist2 = cv::norm(normal2);
 
		float cosParallax = normal1.dot(normal2)/(dist1*dist2);
		
 // 步骤5：判断3D点是否在两个摄像头前方
		// Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
	   // 步骤5.1：3D点深度为负，在第一个摄像头后方，淘汰
		if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
		    continue;
		
           // 步骤5.2：3D点深度为负，在第二个摄像头后方，淘汰
		// Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
		cv::Mat p3dC2 = R*p3dC1+t;
 
		if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
		    continue;
		
// 步骤6：计算重投影误差
		// Check reprojection error in first image
		// 计算3D点在第一个图像上的投影误差
		float im1x, im1y;
		float invZ1 = 1.0/p3dC1.at<float>(2);
		im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
		im1y = fy*p3dC1.at<float>(1)*invZ1+cy;
		float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);
		
         // 步骤6.1：重投影误差太大，跳过淘汰
                 // 一般视差角比较小时重投影误差比较大
		if(squareError1>th2)
		    continue;
		
               // 计算3D点在第二个图像上的投影误差
		// Check reprojection error in second image
		float im2x, im2y;
		float invZ2 = 1.0/p3dC2.at<float>(2);
		im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
		im2y = fy*p3dC2.at<float>(1)*invZ2+cy;
		float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);
 
         // 步骤6.2：重投影误差太大，跳过淘汰
                 // 一般视差角比较小时重投影误差比较大
		if(squareError2>th2)
		    continue;
		
         // 步骤7：统计经过检验的3D点个数，记录3D点视差角
		vCosParallax.push_back(cosParallax);
		vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
		//nGood++;
 
		if(cosParallax<0.99998){
		    vbGood[vMatches12[i].first]=true;
		  // WYW  20180130 修改
		  nGood++;
		 }
	    }
	    
// 步骤8：得到3D点中较大的视差角
	    if(nGood>0)
	    {
		std::sort(vCosParallax.begin(),vCosParallax.end());// 从小到大排序
		
	      // trick! 排序后并没有取最大的视差角
	      // 取一个较大的视差角
		size_t idx = std::min(50,int(vCosParallax.size()-1));
		parallax = acos(vCosParallax[idx])*180/CV_PI;
	    }
	    else
		parallax=0;
 
	    return nGood;
	}

//交会角越大，误差传播越小，因此舍弃相片中心的点？边缘的点更稳定
void checkInterAngle(
    const cv::Mat &_R1, cv::Mat &_T1, const cv::Mat &_R2, cv::Mat &_T2,
    const std::vector<cv::Point2f> &p1, const std::vector<cv::Point2f> &p2,
    const cv::Mat &_K, std::vector<double> &angle_vec) {
    
    int type = _R1.type();
    // cv::Mat P1(cv::Size(3, 3), type), P2(cv::Size(3, 4), type);
    // P1.setTo(0); P2.setTo(0);
    // cv::Mat R1 = _R1.inv();
    // cv::Mat R2 = _R2.inv();
    // cv::Mat T1 = -R1 * _T1;
    // cv::Mat T2 = -R2 * _T2;
    // R1.copyTo(P1.rowRange(0, 2).colRange(0, 2));
    // R2.copyTo(P2.rowRange(0, 2).colRange(0, 2));
    // P1 = P1 * _K.inv();
    // P2 = P2 * _K.inv();
    // T1.copyTo(P1.rowRange(0, 2).colRange(3, 3));
    // T1.copyTo(P1.rowRange(0, 2).colRange(3, 3));
    int pt_size = p1.size();
    cv::Mat p1Mat = cv::Mat::zeros(cv::Size(pt_size, 3), type);
    cv::Mat p2Mat = cv::Mat::zeros(cv::Size(pt_size, 3), type);
    for (int i = 0; i < pt_size; ++i) {
        p1Mat.at<double>(0, i) = p1[i].x;
        p1Mat.at<double>(1, i) = p1[i].y;
        p1Mat.at<double>(2, i) = 1;
        p2Mat.at<double>(0, i) = p2[i].x;
        p2Mat.at<double>(1, i) = p2[i].y;
        p2Mat.at<double>(2, i) = 1;
    }
    cv::Mat pWorldMat1 = _R1.inv() * _K.inv() * p1Mat;
    cv::Mat pWorldMat2 = _R2.inv() * _K.inv() * p2Mat;
    angle_vec.clear();
    angle_vec.resize(pt_size);
    for (int i = 0; i < pt_size; ++i) {
        double dot1 = pWorldMat1.col(i).dot(pWorldMat1.col(i));
        double dot2 = pWorldMat2.col(i).dot(pWorldMat2.col(i));
        double dot12 = pWorldMat1.col(i).dot(pWorldMat2.col(i));
        double angle = dot12 / dot1 / dot2;
        angle = std::acos(angle);
        angle_vec[i] = angle / M_PI * 180;
    }
}
void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
	{
	  // 在DecomposeE函数和ReconstructH函数中对t有归一化
	  // 这里三角化过程中恢复的3D点深度取决于 t 的尺度，
	  // 但是这里恢复的3D点并没有决定单目整个SLAM过程的尺度
	  // 因为CreateInitialMapMonocular函数对3D点深度会缩放，然后反过来对 t 有改变
	    cv::Mat A(4,4,CV_32F);
 
	    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
	    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
	    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
	    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);
 
	    cv::Mat u,w,vt;
	    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
	    x3D = vt.row(3).t();
	    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);//  转换成非齐次坐标  归一化
	}

}
}