/*
 * PairwiseLineMatching.cpp
 *
 *  Created on: 2011-8-19
 *      Author: lz
 */
//opencv得放到arl前面，有定义冲突
#include "opencv_utils.h"
#include "PairwiseLineMatching.h"
#include "gsl_optimize.h"
#include <arlsmat.h>
#include <arlssym.h>
#include <vector>
#include <math.h>
#include <iostream>


using namespace std;

#define Inf 1e10 //Infinity
#define ParallelCross2 sin(6.0 / 180 * M_PI) * sin(6.0 / 180 * M_PI) //sin(6)*sin(6)
//the resolution scale of theta histogram, used when compute angle histogram of lines in the image
#define range 360
#define search_range 60
#define _ResolutionScale 5 //10 degree
#define difInF 1000
const unsigned int _dim_ = range / _ResolutionScale;
/*The following two thresholds are used to decide whether the estimated global rotation is acceptable.
 *Some image pairs don't have a stable global rotation angle, e.g. the image pair of the wide baseline
 *non planar scene. */
#define AcceptableAngleHistogramDifference 0.49
#define AcceptableLengthVectorDifference 0.4

/*The following four thresholds are used to decide whether a line in the left and a line in the right
 *are the possible matched line pair. If they are, then their differences should be smaller than these
 *thresholds.*/
#define LengthDifThreshold 4
#define AngleDifferenceThreshold 0.7854 //45degree
#define DescriptorDifThreshold 0.35		//0.35, or o.5 are good values for LBD
#define DescriptorDifThresholdPt 0.5		//0.35, or o.5 are good values for LBD
/*The following four threshold are used to decide whether the two possible matched line pair agree with each other.
 *They reflect the similarity of pairwise geometric information.
 */
#define RelativeAngleDifferenceThreshold 0.7854 //45degree
#define IntersectionRationDifThreshold 1
#define ProjectionRationDifThreshold 1
#define PointDistRatioThreshold 0.2

//this is used when get matching result from principal eigen vector
#define WeightOfMeanEigenVec 0.1

double getNormL2(double *arr, int size)
{
	double result = 0;
	for (int i = 0; i < size; i++)
	{
        result = result + arr[i] * arr[i];
	}
	return sqrt(result);
}
void arrayMultiRatio(double *arr, int size, double ratio)
{
	for (int i = 0; i < size; i++)
	{
		arr[i] = arr[i] * ratio;
	}
}
bool matSave(std::vector<std::vector<double>> mat, const std::string &filename)
{
	std::ofstream fs(filename.c_str());
	if (!fs)
	{
		return false;
	}
	else
	{
		for (int i = 0; i < mat.size(); ++i) {
			fs <<"id: " << i << " : ";
			for (int j = 0; j < mat[i].size(); ++j) {
				fs << mat[i][j] << " ";
			}
			fs << "\n";
		}
		
	};
	return true;
}

void PairwiseLineMatching::LineMatching(
        ScaleLines &linesInLeft, ScalePoints leftPt, ScaleLines &linesInRight, ScalePoints rightPt, 
        std::vector<int> &matchResult, std::vector<cv::DMatch> &pt_matchs)
{
	//compute the global rotation angle of image pair;
	std::array<double, _dim_> _angleHistLeft, _lengthLeft, _angleHistRight, _lengthRight;
    calAngleAndLenHist(linesInLeft, _angleHistLeft, _lengthLeft);
    calAngleAndLenHist(linesInRight, _angleHistRight, _lengthRight);
	globalRotationAngle_ = GlobalRotationOfImagePair_(_angleHistLeft, _lengthLeft, _angleHistRight, _lengthRight);
    BuildAdjacencyMatrix_(linesInLeft, leftPt, linesInRight, rightPt);
	MatchingResultFromPrincipalEigenvector_(linesInLeft, leftPt, linesInRight, rightPt, matchResult, pt_matchs);
	// consistencyCheck(linesInLeft, leftPt, linesInRight, rightPt, matchResult, pt_matchs);
}

void PairwiseLineMatching::drawMatch(const cv::Mat &left, const ScaleLines &left_lines, 
        const cv::Mat &right, const ScaleLines &right_lines, 
        const std::vector<int> &matchResult, cv::Mat &show_image) {
	cv::Point p1, p2;
    int lineIDLeft, lineIDRight;
    int lowest1 = 0, highest1 = 255;
    int range1 = (highest1 - lowest1) + 1;
	show_image = cv::Mat::zeros(cv::Size(left.cols + right.cols, left.rows), left.type());
	cv::Rect rect1(cv::Point2f(0, 0), left.size());
	cv::Rect rect2(cv::Point2f(left.cols, 0), left.size());
	left.copyTo(show_image(rect1));
	right.copyTo(show_image(rect2));
	// cv::cvtColor(show_image, show_image, cv::COLOR_GRAY2BGR);
    std::vector<unsigned int> r1(matchResult.size() / 2), g1(matchResult.size() / 2), b1(matchResult.size() / 2); //the color of lines
	for (unsigned int pair = 0; pair < matchResult.size() / 2; pair++)
    {
        r1[pair] = lowest1 + int(rand() % range1);
        g1[pair] = lowest1 + int(rand() % range1);
        b1[pair] = 255 - r1[pair];
        lineIDLeft = matchResult[2 * pair];
        lineIDRight = matchResult[2 * pair + 1];
        p1 = cv::Point(int(left_lines[lineIDLeft][0].startPointX), int(left_lines[lineIDLeft][0].startPointY));
        p2 = cv::Point(int(left_lines[lineIDLeft][0].endPointX), int(left_lines[lineIDLeft][0].endPointY));
        cv::line(show_image, p1, p2, CV_RGB(r1[pair], g1[pair], b1[pair]), 4, cv::LINE_AA, 0);
        p1 = cv::Point(int(right_lines[lineIDRight][0].startPointX + left.cols), int(right_lines[lineIDRight][0].startPointY));
        p2 = cv::Point(int(right_lines[lineIDRight][0].endPointX + left.cols), int(right_lines[lineIDRight][0].endPointY));
        cv::line(show_image, p1, p2, CV_RGB(r1[pair], g1[pair], b1[pair]), 4, cv::LINE_AA, 0);
    }
	for (unsigned int pair = 0; pair < matchResult.size() / 2; pair++)
    {
        lineIDLeft = matchResult[2 * pair];
        lineIDRight = matchResult[2 * pair + 1];
        p1 = cv::Point(int(left_lines[lineIDLeft][0].startPointX), int(left_lines[lineIDLeft][0].startPointY));
        p2 = cv::Point(int(right_lines[lineIDRight][0].startPointX + left.cols), int(right_lines[lineIDRight][0].startPointY));
        cv::line(show_image, p1, p2, CV_RGB(r1[pair], g1[pair], b1[pair]), 1, cv::LINE_AA, 0);
    }
}

double PairwiseLineMatching::calAngleAndLenHist(
	ScaleLines &lines, std::array<double, _dim_> &angleHist, std::array<double, _dim_> &lengthHist)
{
	double TwoPI = 2 * M_PI;
	double rotationAngle = TwoPI;

	//step 1: compute the angle histogram of lines in the left and right images
	// const unsigned int dim = 360 / ResolutionScale; //number of the bins of histogram
	unsigned int index;								//index in the histogram
	double direction;
	double scalar = 180 / (ResolutionScale * M_PI); //used when compute the index
	double angleShift = (ResolutionScale * M_PI) / 360;  //make sure zero is the middle of the interval

	// std::array<double, dim> angleHistLeft;
	// std::array<double, dim> angleHistRight;
	// std::array<double, dim> lengthLeft; //lengthLeft[i] store the total line length of all the lines in the ith angle bin.
	// std::array<double, dim> lengthRight;
	angleHist.fill(0);
	lengthHist.fill(0);

	for (unsigned int linenum = 0; linenum < lines.size(); linenum++)
	{
		direction = lines[linenum][0].direction + M_PI + angleShift;
		direction = direction < TwoPI ? direction : (direction - TwoPI);
		index = floor(direction * scalar);
		angleHist[index]++;
		lengthHist[index] += lines[linenum][0].lineLength;
	}
	arrayMultiRatio(angleHist.data(), angleHist.size(), (1 / getNormL2(angleHist.data(), angleHist.size())));
	arrayMultiRatio(lengthHist.data(), lengthHist.size(), (1 / getNormL2(lengthHist.data(), lengthHist.size())));
}

double PairwiseLineMatching::GlobalRotationOfImagePair_(
	std::array<double, _dim_> angleHistLeft, std::array<double, _dim_> lengthLeft,
	std::array<double, _dim_> angleHistRight, std::array<double, _dim_> lengthRight)
{
	//  angleHistLeft.Save("histLeft.txt");
	//  angleHistRight.Save("histRight.txt");
	double TwoPI = 2 * M_PI;
	double rotationAngle = TwoPI;
    const unsigned int dim = 360 / _ResolutionScale;
	unsigned int index;								//index in the histogram
	//  angleHistLeft.Save("histLeft.txt");
	//  angleHistRight.Save("histRight.txt");

	//step 2: find shift to decide the approximate global rotation
	std::array<double, dim> difVec; //the difference vector between left histogram and shifted right histogram
	double minDif = difInF;				//the minimal angle histogram difference
	double secondMinDif = difInF;		//the second minimal histogram difference
	unsigned int minShift;			//the shift of right angle histogram when minimal difference achieved
	unsigned int secondMinShift;	//the shift of right angle histogram when second minimal difference achieved

	std::array<double, dim> lengthDifVec; //the length difference vector between left and right
	double minLenDif = difInF;				  //the minimal length difference
	double secondMinLenDif = difInF;		  //the second minimal length difference
	unsigned int minLenShift;			  //the shift of right length vector when minimal length difference achieved
	unsigned int secondMinLenShift;		  //the shift of right length vector when the second minimal length difference achieved

	double normOfVec;
	GslOptimizer go;
	const size_t data_dim = search_range / _ResolutionScale * 2 + 1;
    const size_t para_dim = 3;
    double t[data_dim], y[data_dim], weights[data_dim], paras[para_dim];
    struct GslData data = { data_dim, para_dim, t, y, weights, paras };
	// -search_range to search_range
	for (int shift = - search_range / _ResolutionScale; shift <=  search_range / _ResolutionScale; shift++)
	{
		int ori_shift = shift;
		shift = shift > 0 ? shift : shift + dim;
		for (unsigned int j = 0; j < dim; j++)
		{
			index = j + shift;
			index = index < dim ? index : (index - dim);
			difVec[j] = angleHistLeft[j] - angleHistRight[index];
			lengthDifVec[j] = lengthLeft[j] - lengthRight[index];
		}
		//find the minShift and secondMinShift for angle histogram
		normOfVec = getNormL2(difVec.data(), difVec.size());
		t[ori_shift + search_range / _ResolutionScale] = ori_shift;
		y[ori_shift + search_range / _ResolutionScale] = normOfVec;
		if (normOfVec < secondMinDif)
		{
			if (normOfVec < minDif)
			{
				secondMinDif = minDif;
				secondMinShift = minShift;
				minDif = normOfVec;
				minShift = shift;
			}
			else
			{
				secondMinDif = normOfVec;
				secondMinShift = shift;
			}
		}
		//find the minLenShift and secondMinLenShift of length vector
		normOfVec = getNormL2(lengthDifVec.data(), lengthDifVec.size());
		if (normOfVec < secondMinLenDif)
		{
			if (normOfVec < minLenDif)
			{
				secondMinLenDif = minLenDif;
				secondMinLenShift = minLenShift;
				minLenDif = normOfVec;
				minLenShift = shift;
			}
			else
			{
				secondMinLenDif = normOfVec;
				secondMinLenShift = shift;
			}
		}
		shift = ori_shift;
	}
	go.solve(data, expb_f, expb_df, callback);
	double shiftPeak = - data.paras[1] / data.paras[0] / 2;

	//first check whether there exist an approximate global rotation angle between image pair
	if (minDif < AcceptableAngleHistogramDifference && minLenDif < AcceptableLengthVectorDifference)
	{
		rotationAngle = shiftPeak * ResolutionScale;
        if (rotationAngle > 90 && 360 - rotationAngle > 90)
		{
			//In most case we believe the rotation angle between two image pairs should belong to [-Pi/2, Pi/2]
			rotationAngle = rotationAngle - 180;
		}
		rotationAngle = rotationAngle * M_PI / 180;
	}

	cout << "minimal histgram distance = " << minDif << ", Approximate global rotation angle = " << rotationAngle << endl;
	return rotationAngle;
}

void PairwiseLineMatching::BuildAdjacencyMatrix_(ScaleLines &linesInLeft, ScalePoints &ptInLeft, ScaleLines &linesInRight, ScalePoints &ptInRight)
{
	double TwoPI = 2 * M_PI;
	auto &left_key_pt = ptInLeft.points;
	auto &right_key_pt = ptInRight.points;
	const unsigned int numLineLeft = linesInLeft.size();
	const unsigned int numLineRight = linesInRight.size();
	const unsigned int numPointLeft = ptInLeft.points.size();
	const unsigned int numPointRight = ptInRight.points.size();
	/*first step, find nodes which are possible correspondent lines in the left and right images according to
	 *their direction, gray value  and gradient magnitude.
     */
	nodesList_.clear();
	volatile double angleDif;
	volatile double lengthDif;

	unsigned int dimOfDes = linesInLeft[0][0].descriptor.size();
	double desDisMat[numLineLeft][numLineRight]; //store the descriptor distance of lines in left and right images.
	double desPtDisMat[numPointLeft][numPointRight + 2]; //store the descriptor distance of lines in left and right images.

	std::vector<float> desLeft;
	std::vector<float> desRight;

	//first compute descriptor distances

	float *desL, *desR, *desMax, *desOld;

	float minDis, dis, temp;
	for (int idL = 0; idL < numLineLeft; idL++)
	{
		short sameLineSize = linesInLeft[idL].size();
		for (int idR = 0; idR < numLineRight; idR++)
		{
			minDis = 100;
			short sameLineSizeR = linesInRight[idR].size();
			for (short lineIDInSameLines = 0; lineIDInSameLines < sameLineSize; lineIDInSameLines++)
			{
				desOld = linesInLeft[idL][lineIDInSameLines].descriptor.data();
				for (short lineIDInSameLinesR = 0; lineIDInSameLinesR < sameLineSizeR; lineIDInSameLinesR++)
				{
					desL = desOld;
					desR = linesInRight[idR][lineIDInSameLinesR].descriptor.data();
					desMax = desR + dimOfDes;
					dis = 0;
					while (desR < desMax)
					{
                        temp = *desL++ - *desR++; //discriptor minus save to temp
						dis += temp * temp;
					}
					dis = sqrt(dis);
					if (dis < minDis)
					{
						minDis = dis;
					}
				}
			} //end for(short lineIDInSameLines = 0; lineIDInSameLines<sameLineSize; lineIDInSameLines++)
			desDisMat[idL][idR] = minDis;
		} //end for(int idR=0; idR<rightSize; idR++)
	}	 // end for(int idL=0; idL<leftSize; idL++)


	for (unsigned int i = 0; i < numLineLeft; i++)
	{
		for (unsigned int j = 0; j < numLineRight; j++)
		{
			if (desDisMat[i][j] > DescriptorDifThreshold) {
				continue; //the descriptor difference is too large;
			}

			lengthDif = fabs(linesInLeft[i][0].lineLength - linesInRight[j][0].lineLength) / MIN(linesInLeft[i][0].lineLength, linesInRight[j][0].lineLength);
			if (lengthDif > LengthDifThreshold) {
				continue; //the length difference is too large;
			}
			if (globalRotationAngle_ < TwoPI) { //there exist a global rotation angle between two image
				angleDif = fabs(linesInLeft[i][0].direction + globalRotationAngle_ - linesInRight[j][0].direction);
				if (fabs(TwoPI - angleDif) > AngleDifferenceThreshold && angleDif > AngleDifferenceThreshold)
				{
					continue; //the angle difference is too large;
				}

				Node node; //line i in left image and line j in right image pass the test, (i,j) is a possible matched line pair.
				node.leftLineID = i;
				node.rightLineID = j;
				nodesList_.push_back(node);
			}
		} //end inner loop
	}
	std::cout << "the number of possible matched line pair = " << nodesList_.size() << endl;
	//	desDisMat.Save("DescriptorDis.txt");
    //计算大尺度特征点之间的距离矩阵
	double min_dis_in_scale, min_dis_in_right, min_dis = 100, second_min_in_right = 100;
	for (int idL = 0; idL < numPointLeft; ++idL)
	{
		min_dis_in_right = 100;
		second_min_in_right = 100;
		short samePtSizeL = ptInLeft.points[idL].size();
		for (int idR = 0; idR < numPointRight; ++idR)
		{
			min_dis_in_scale = 100;
			short samePtSizeR = ptInRight.points[idR].size();
			desOld = (float *)ptInLeft.descriptor.ptr(idL);
			for (short ptIDInSamePtsL = 0; ptIDInSamePtsL < samePtSizeL; ++ptIDInSamePtsL)
			{
				for (short ptIDInSamePtsR = 0; ptIDInSamePtsR < samePtSizeR; ++ptIDInSamePtsR)
				{
					dis = 0;
					desL = desOld;
					desR = (float *)ptInRight.descriptor.ptr(idR);
					for (int i = 0; i < ptInLeft.dim_of_descriptor; ++i) {
						temp = *desL++ - *desR++; //discriptor minus save to temp
						dis += temp * temp;
					}
					dis = sqrt(dis);
					if (dis < min_dis_in_scale)
					{
						min_dis_in_scale = dis;
					}
				}
			} //end for(short lineIDInSameLines = 0; lineIDInSameLines<sameLineSize; lineIDInSameLines++)
			desPtDisMat[idL][idR] = min_dis_in_scale;
			if (min_dis_in_scale < second_min_in_right) {
				second_min_in_right = min_dis_in_scale;
				if (min_dis_in_scale < min_dis_in_right) {
					second_min_in_right = min_dis_in_right;
					min_dis_in_right = min_dis_in_scale;
				}
			}
			min_dis_in_right = min_dis_in_right > min_dis_in_scale ? min_dis_in_scale : min_dis_in_right; 
		} //end for(int idR=0; idR<rightSize; idR++)
		desPtDisMat[idL][numPointRight] = min_dis_in_right;
		desPtDisMat[idL][numPointRight + 1] = second_min_in_right;
		min_dis = min_dis > min_dis_in_right ? min_dis_in_right : min_dis;
	}	 // end for(int idL=0; idL<leftSize; idL++)

    //计算候选匹配， 采用KnnMatch
    for (unsigned int i = 0; i < numPointLeft; ++i)
	{
		for (unsigned int j = 0; j < numPointRight; ++j)
		{
			// if (desPtDisMat[i][j] > DescriptorDifThresholdPt) {
			if (desPtDisMat[i][j] > desPtDisMat[i][numPointRight] * 2) {
				continue; //the descriptor difference is too large;
			}
			if (std::abs(ptInLeft.points[i][0].size - ptInRight.points[j][0].size) > 10) {
				continue; //尺度差异太大;
			}
			if (globalRotationAngle_ < TwoPI) { //there exist a global rotation angle between two image
				angleDif = fabs(ptInLeft.points[i][0].angle + globalRotationAngle_ - ptInRight.points[j][0].angle);
				// if (fabs(TwoPI - angleDif) > AngleDifferenceThreshold && angleDif > AngleDifferenceThreshold)
				// {
				// 	continue; //the angle difference is too large;
				// }

				Node node; //line i in left image and line j in right image pass the test, (i,j) is a possible matched line pair.
				node.type = FeatureType::Point_;
				node.leftPtID = i + numLineLeft;
				node.rightPtID = j + numLineRight;
				node.leftLineID = i + numLineLeft;
				node.rightLineID = j + numLineRight;
				nodesList_.push_back(node);
			}
		} //end inner loop
	}
	/*Second step, build the adjacency matrix which reflect the geometric constraints between nodes.
	 *The matrix is stored in the Compressed Sparse Column(CSC) format.
     */
	unsigned int dim = nodesList_.size(); // Dimension of the problem.

	//std::array<double, dim_temp> adjacenceVec;
	std::vector<double> adjacenceVec(dim * (dim + 1) / 2, 0);

	int nnz = 0; // Number of nonzero elements in adjacenceMat.
	/*adjacenceVec only store the lower part of the adjacency matrix which is a symmetric matrix.
	 *                    | 0  1  0  2  0 |
	 *                    | 1  0  3  0  1 |
	 *eg:  adjMatrix =    | 0  3  0  2  0 |
	 *                    | 2  0  2  0  3 |
	 *                    | 0  1  0  3  0 |
	 *     adjacenceVec = [0,1,0,2,0,0,3,0,1,0,2,0,0,3,0]
	 */
	//	Matrix<double> testMat(dim,dim);
	//	testMat.SetZero();

	/*In order to save computational time, the following variables are used to store
	 *the pairwise geometric information which has been computed and will be reused many times
	 *latter. The reduction of computational time is at the expenses of memory consumption.
	 */
    unsigned int bComputedLeft[numLineLeft][numLineLeft]; //flag to show whether the ith pair of left image has already been computed.
    memset(bComputedLeft, 0, numLineLeft * numLineLeft * sizeof(unsigned int));
    double intersecRatioLeft[numLineLeft][numLineLeft]; //the ratio of intersection point and the line in the left pair
    double projRatioLeft[numLineLeft][numLineLeft];		//the point to line distance divided by the projected length of line in the left pair.

    unsigned int bComputedRight[numLineRight][numLineRight]; //flag to show whether the ith pair of right image has already been computed.
    memset(bComputedRight, 0, numLineRight * numLineRight * sizeof(unsigned int));
    double intersecRatioRight[numLineRight][numLineRight]; //the ratio of intersection point and the line in the right pair
    double projRatioRight[numLineRight][numLineRight];	 //the point to line distance divided by the projected length of line in the right pair.

    unsigned int idLeft1, idLeft2;						//the id of lines in the left pair
	unsigned int idRight1, idRight2;					//the id of lines in the right pair
	double relativeAngleLeft, relativeAngleRight;		//the relative angle of each line pair
	double gradientMagRatioLeft, gradientMagRatioRight; //the ratio of gradient magnitude of lines in each pair

	double iRatio1L, iRatio1R, iRatio2L, iRatio2R;
	double pRatio1L, pRatio1R, pRatio2L, pRatio2R;

	double relativeAngleDif, gradientMagRatioDif, iRatioDif, pRatioDif;

	double interSectionPointX, interSectionPointY;
	double a1, a2, b1, b2, c1, c2, l1, l2; //line1: a1 x + b1 y + c1 =0; line2: a2 x + b2 y + c2=0
	double a1b2_a2b1;			   //a1b2-a2b1
	double length1, length2, len;
	double disX, disY;
	double disS, disE;
	double similarity;

    int line_type_count;
	double left_pt_dist, right_pt_dist;
    unsigned int b_pt_computed_left[numPointLeft + numLineLeft][numPointLeft + numLineLeft]; //flag to show whether the ith pair of left image has already been computed.
    unsigned int b_pt_computed_right[numPointRight + numLineRight][numPointRight + numLineRight]; //flag to show whether the ith pair of left image has already been computed.
    memset(b_pt_computed_left, 0, (numPointLeft + numLineLeft) * (numPointLeft + numLineLeft) * sizeof(unsigned int));
    memset(b_pt_computed_right, 0, (numPointRight + numLineRight) * (numPointRight + numLineRight) * sizeof(unsigned int));
	double pt_x1, pt_y1, pt_x2, pt_y2;
	double pt_dist_left[numPointLeft][numPointLeft];
	double pt_dist_right[numPointRight][numPointRight];

    //定权，距离和描述符ratio test
	double max_dist = 1280 * 1280;
	double dist_thresh = 600 * 600;//图像宽度一半，超出这部分的太远，不能加入约束
	matrix = cv::Mat::zeros(dim, dim, CV_32F);
	for (unsigned int j = 0; j < dim; j++)
	{ //column
	    FeatureType left_type = nodesList_[j].type;
		idLeft1 = left_type == FeatureType::Point_ ? nodesList_[j].leftPtID : nodesList_[j].leftLineID;
		idRight1 = left_type == FeatureType::Point_ ? nodesList_[j].rightPtID : nodesList_[j].rightLineID;
		for (unsigned int i = j + 1; i < dim; i++)
        { //row
			FeatureType right_type = nodesList_[i].type;
			idLeft2 = right_type == FeatureType::Point_ ? nodesList_[i].leftPtID : nodesList_[i].leftLineID;
			idRight2 = right_type == FeatureType::Point_ ? nodesList_[i].rightPtID : nodesList_[i].rightLineID;
			if ((idLeft1 == idLeft2) || (idRight1 == idRight2))
			{
				continue; //not satisfy the one to one match condition
			}
			line_type_count = left_type + right_type;
			if (line_type_count == 0) {// 点点组合
				int left_pt1 = idLeft1 - numLineLeft;
				int left_pt2 = idLeft2 - numLineLeft;
				int right_pt1 = idRight1 - numLineRight;
				int right_pt2 = idRight2 - numLineRight;
			    if (b_pt_computed_left[idLeft1][idLeft2]) {
					left_pt_dist = pt_dist_left[idLeft1][idLeft2];
				} else {
					pt_x1 = ptInLeft.points[left_pt1][0].pt.x;
					pt_y1 = ptInLeft.points[left_pt1][0].pt.y;
					pt_x2 = ptInLeft.points[left_pt2][0].pt.x;
					pt_y2 = ptInLeft.points[left_pt2][0].pt.y;
					b_pt_computed_left[idLeft1][idLeft2] = 1;
					b_pt_computed_left[idLeft2][idLeft1] = 1;
					left_pt_dist = (pt_x1 - pt_x2) * (pt_x1 - pt_x2) + (pt_y1 - pt_y2) * (pt_y1 - pt_y2);
					pt_dist_left[idLeft1][idLeft2] = left_pt_dist;
					pt_dist_left[idLeft2][idLeft1] = left_pt_dist;
				}
				if (b_pt_computed_right[idRight1][idRight2]) {
					right_pt_dist = pt_dist_right[idRight1][idRight2];
				} else {
					pt_x1 = ptInRight.points[right_pt1][0].pt.x;
					pt_y1 = ptInRight.points[right_pt1][0].pt.y;
					pt_x2 = ptInRight.points[right_pt2][0].pt.x;
					pt_y2 = ptInRight.points[right_pt2][0].pt.y;
					b_pt_computed_right[idRight1][idRight2] = 1;
					b_pt_computed_right[idRight2][idRight1] = 1;
					right_pt_dist = (pt_x1 - pt_x2) * (pt_x1 - pt_x2) + (pt_y1 - pt_y2) * (pt_y1 - pt_y2);
					pt_dist_right[idRight1][idRight2] = right_pt_dist;
					pt_dist_right[idRight2][idRight1] = right_pt_dist;
				}
				//1.描述符约束、2.距离约束、3.角度约束，后两者为空间约束，选其中差异大的一个
				double dist = std::min(left_pt_dist, right_pt_dist);
				double ratio_test1 = desPtDisMat[idLeft1][numPointLeft + 1] / desPtDisMat[idLeft1][numPointLeft];
				double ratio_test2 = desPtDisMat[idRight1][numPointRight + 1] / desPtDisMat[idRight2][numPointRight];
				//如果两个ratio_test都比较小，直接给最值，不考虑距离了
				double weight = 0;
				weight += ratio_test1 / 2;
				weight -= dist / max_dist;
				//设置权重 2 + weight， 1 - weight / 2，另外，还需要算出阈值
				// if (std::min(left_pt_dist, right_pt_dist) > dist_thresh) continue;//必须任意两对都加上约束，否则约束不够
				//第一次运行状况为无描述符约束，仅仅包含空间约束就可以匹配正确，点线组合参数为2
				// double dist_ratio = 1 - std::min(left_pt_dist / right_pt_dist, right_pt_dist / left_pt_dist);
				double dist_ratio = std::abs(left_pt_dist - right_pt_dist) / std::min(left_pt_dist, right_pt_dist);
				if (dist_ratio > PointDistRatioThreshold ) continue;
				nnz++;
				similarity = 4 - dist_ratio * 2 - desPtDisMat[left_pt1][right_pt1] / DescriptorDifThresholdPt - desPtDisMat[left_pt2][right_pt2] / DescriptorDifThresholdPt ;
                adjacenceVec[(2 * dim - j - 1)* j / 2 + i] = similarity;
				matrix.at<float>(i, j) = similarity; 
				continue;
			} else if (line_type_count == 1) { //点线组合
				int left_pt1 = idLeft1 - numLineLeft;
				int left_pt2 = idLeft2 - numLineLeft;
				int right_pt1 = idRight1 - numLineRight;
				int right_pt2 = idRight2 - numLineRight;
			    if (b_pt_computed_left[idLeft1][idLeft2]) {
                    left_pt_dist = pt_dist_left[idLeft1][idLeft2];
				} else {
					if (left_type == FeatureType::Line_) {
						pt_x1 = linesInLeft[idLeft1][0].midX;
						pt_y1 = linesInLeft[idLeft1][0].midY;
						pt_x2 = ptInLeft.points[idLeft2 - numLineLeft][0].pt.x;
						pt_y2 = ptInLeft.points[idLeft2 - numLineLeft][0].pt.y;
					} else {
						pt_x1 = ptInLeft.points[idLeft1 - numLineLeft][0].pt.x;
						pt_y1 = ptInLeft.points[idLeft1 - numLineLeft][0].pt.y;
						pt_x2 = linesInLeft[idLeft2][0].midX;
						pt_y2 = linesInLeft[idLeft2][0].midY;
					}
                    b_pt_computed_left[idLeft1][idLeft2] = 1;
					left_pt_dist = (pt_x1 - pt_x2) * (pt_x1 - pt_x2) + (pt_y1 - pt_y2) * (pt_y1 - pt_y2);
					pt_dist_left[idLeft1][idLeft2] = left_pt_dist;
					pt_dist_left[idLeft2][idLeft1] = left_pt_dist;
				}
				if (b_pt_computed_right[idRight1][idRight2]) {
                    right_pt_dist = pt_dist_right[idRight1][idRight2];
				} else {
					if (left_type == FeatureType::Line_) {
						pt_x1 = linesInRight[idRight1][0].midX;
						pt_y1 = linesInRight[idRight1][0].midY;
						pt_x2 = ptInRight.points[idRight2 - numLineRight][0].pt.x;
						pt_y2 = ptInRight.points[idRight2 - numLineRight][0].pt.y;
					} else {
						pt_x1 = ptInRight.points[idRight1 - numLineRight][0].pt.x;
						pt_y1 = ptInRight.points[idRight1 - numLineRight][0].pt.y;
						pt_x2 = linesInRight[idRight2][0].midX;
						pt_y2 = linesInRight[idRight2][0].midY;
					}
                    b_pt_computed_right[idRight1][idRight2] = 1;
                    b_pt_computed_right[idRight2][idRight1] = 1;
					right_pt_dist = (pt_x1 - pt_x2) * (pt_x1 - pt_x2) + (pt_y1 - pt_y2) * (pt_y1 - pt_y2);
					pt_dist_right[idRight1][idRight2] = right_pt_dist;
					pt_dist_right[idRight2][idRight1] = right_pt_dist;
				}
				// if (std::min(left_pt_dist, right_pt_dist) > dist_thresh) continue;
				// double dist_ratio = 1 - std::min(left_pt_dist / right_pt_dist, right_pt_dist / left_pt_dist);
				double dist_ratio = std::abs(left_pt_dist - right_pt_dist) / std::min(left_pt_dist, right_pt_dist);
				if (dist_ratio > PointDistRatioThreshold ) continue;
				nnz++;
				similarity = 4 - dist_ratio * 4;//2可以，好像有一次改成4不行，权重设置是否要和点保持一致
                adjacenceVec[(2 * dim - j - 1) * j / 2 + i] = similarity;
				matrix.at<float>(i, j) = similarity;
				continue;
			}
			//first compute the relative angle between left pair and right pair.
			relativeAngleLeft = linesInLeft[idLeft1][0].direction - linesInLeft[idLeft2][0].direction;
			relativeAngleLeft = (relativeAngleLeft < M_PI) ? relativeAngleLeft : (relativeAngleLeft - TwoPI);
			relativeAngleLeft = (relativeAngleLeft > (-M_PI)) ? relativeAngleLeft : (relativeAngleLeft + TwoPI);
			relativeAngleRight = linesInRight[idRight1][0].direction - linesInRight[idRight2][0].direction;
			relativeAngleRight = (relativeAngleRight < M_PI) ? relativeAngleRight : (relativeAngleRight - TwoPI);
			relativeAngleRight = (relativeAngleRight > (-M_PI)) ? relativeAngleRight : (relativeAngleRight + TwoPI);
			relativeAngleDif = fabs(relativeAngleLeft - relativeAngleRight);
			if ((TwoPI - relativeAngleDif) > RelativeAngleDifferenceThreshold && relativeAngleDif > RelativeAngleDifferenceThreshold)
			{
				continue; //the relative angle difference is too large;
			}
			else if ((TwoPI - relativeAngleDif) < RelativeAngleDifferenceThreshold)
			{
				relativeAngleDif = TwoPI - relativeAngleDif;
			}

			//at last, check the intersect point ratio and point to line distance ratio
			//check whether the geometric information of pairs (idLeft1,idLeft2) and (idRight1,idRight2) have already been computed.
			if (!bComputedLeft[idLeft1][idLeft2])
			{ //have not been computed yet

				a1 = linesInLeft[idLeft1][0].endPointY - linesInLeft[idLeft1][0].startPointY;					//disY
				b1 = linesInLeft[idLeft1][0].startPointX - linesInLeft[idLeft1][0].endPointX;					//-disX
				c1 = (0 - b1 * linesInLeft[idLeft1][0].startPointY) - a1 * linesInLeft[idLeft1][0].startPointX; //disX*sy - disY*sx
				length1 = linesInLeft[idLeft1][0].lineLength;

				a2 = linesInLeft[idLeft2][0].endPointY - linesInLeft[idLeft2][0].startPointY;					//disY
				b2 = linesInLeft[idLeft2][0].startPointX - linesInLeft[idLeft2][0].endPointX;					//-disX
				c2 = (0 - b2 * linesInLeft[idLeft2][0].startPointY) - a2 * linesInLeft[idLeft2][0].startPointX; //disX*sy - disY*sx
				length2 = linesInLeft[idLeft2][0].lineLength;
                l1 = a1 * a1 + b1 * b1;
                l2 = a2 * a2 + b2 * b2;
				a1b2_a2b1 = a1 * b2 - a2 * b1;
				if (a1b2_a2b1 * a1b2_a2b1 < ParallelCross2 * l1 * l2)//l1.cross(l2) < sin(6)
				{ //two lines are almost parallel
					iRatio1L = Inf;
					iRatio2L = Inf;
				}
				else
				{
					interSectionPointX = (c2 * b1 - c1 * b2) / a1b2_a2b1;
					interSectionPointY = (c1 * a2 - c2 * a1) / a1b2_a2b1;
					//r1 = (s1I*s1e1)/(|s1e1|*|s1e1|)
					disX = interSectionPointX - linesInLeft[idLeft1][0].startPointX;
					disY = interSectionPointY - linesInLeft[idLeft1][0].startPointY;
					len = disY * a1 - disX * b1;
					iRatio1L = len / (length1 * length1);
					//r2 = (s2I*s2e2)/(|s2e2|*|s2e2|)
					disX = interSectionPointX - linesInLeft[idLeft2][0].startPointX;
					disY = interSectionPointY - linesInLeft[idLeft2][0].startPointY;
					len = disY * a2 - disX * b2;
					iRatio2L = len / (length2 * length2);
				}
				intersecRatioLeft[idLeft1][idLeft2] = iRatio1L;
				intersecRatioLeft[idLeft2][idLeft1] = iRatio2L; //line order changed

				/*project the end points of line1 onto line2 and compute their distances to line2;
				 */
				disS = fabs(a2 * linesInLeft[idLeft1][0].startPointX + b2 * linesInLeft[idLeft1][0].startPointY + c2) / length2;
				disE = fabs(a2 * linesInLeft[idLeft1][0].endPointX + b2 * linesInLeft[idLeft1][0].endPointY + c2) / length2;
				pRatio1L = (disS + disE) / length1;
				projRatioLeft[idLeft1][idLeft2] = pRatio1L;

				/*project the end points of line2 onto line1 and compute their distances to line1;
				 */
				disS = fabs(a1 * linesInLeft[idLeft2][0].startPointX + b1 * linesInLeft[idLeft2][0].startPointY + c1) / length1;
				disE = fabs(a1 * linesInLeft[idLeft2][0].endPointX + b1 * linesInLeft[idLeft2][0].endPointY + c1) / length1;
				pRatio2L = (disS + disE) / length2;
				projRatioLeft[idLeft2][idLeft1] = pRatio2L;

				//mark them as computed
				bComputedLeft[idLeft1][idLeft2] = true;
				bComputedLeft[idLeft2][idLeft1] = true;
			}
			else
			{ //read these information from matrix;
				iRatio1L = intersecRatioLeft[idLeft1][idLeft2];
				iRatio2L = intersecRatioLeft[idLeft2][idLeft1];
				pRatio1L = projRatioLeft[idLeft1][idLeft2];
				pRatio2L = projRatioLeft[idLeft2][idLeft1];
			}
            if (!bComputedRight[idRight1][idRight2])
            {
                //have not been computed yet
                a1 = linesInRight[idRight1][0].endPointY - linesInRight[idRight1][0].startPointY;					//disY
                b1 = linesInRight[idRight1][0].startPointX - linesInRight[idRight1][0].endPointX;					//-disX
                c1 = (0 - b1 * linesInRight[idRight1][0].startPointY) - a1 * linesInRight[idRight1][0].startPointX; //disX*sy - disY*sx
                length1 = linesInRight[idRight1][0].lineLength;

                a2 = linesInRight[idRight2][0].endPointY - linesInRight[idRight2][0].startPointY;					//disY
                b2 = linesInRight[idRight2][0].startPointX - linesInRight[idRight2][0].endPointX;					//-disX
                c2 = (0 - b2 * linesInRight[idRight2][0].startPointY) - a2 * linesInRight[idRight2][0].startPointX; //disX*sy - disY*sx
                length2 = linesInRight[idRight2][0].lineLength;
				l1 = a1 * a1 + b1 * b1;
                l2 = a2 * a2 + b2 * b2;
                a1b2_a2b1 = a1 * b2 - a2 * b1;
                if (a1b2_a2b1 * a1b2_a2b1 < ParallelCross2 * l1 * l2)
                { //two lines are almost parallel
                    iRatio1R = Inf;
                    iRatio2R = Inf;
                }
                else
                {
                    interSectionPointX = (c2 * b1 - c1 * b2) / a1b2_a2b1;
                    interSectionPointY = (c1 * a2 - c2 * a1) / a1b2_a2b1;
                    //r1 = (s1I*s1e1)/(|s1e1|*|s1e1|)
                    disX = interSectionPointX - linesInRight[idRight1][0].startPointX;
                    disY = interSectionPointY - linesInRight[idRight1][0].startPointY;
                    len = disY * a1 - disX * b1; //because b1=-disX
                    iRatio1R = len / (length1 * length1);
                    //r2 = (s2I*s2e2)/(|s2e2|*|s2e2|)
                    disX = interSectionPointX - linesInRight[idRight2][0].startPointX;
                    disY = interSectionPointY - linesInRight[idRight2][0].startPointY;
                    len = disY * a2 - disX * b2; //because b2=-disX
                    iRatio2R = len / (length2 * length2);
                }
                intersecRatioRight[idRight1][idRight2] = iRatio1R;
                intersecRatioRight[idRight2][idRight1] = iRatio2R; //line order changed
                /*project the end points of line1 onto line2 and compute their distances to line2;
                 */
                disS = fabs(a2 * linesInRight[idRight1][0].startPointX + b2 * linesInRight[idRight1][0].startPointY + c2) / length2;
                disE = fabs(a2 * linesInRight[idRight1][0].endPointX + b2 * linesInRight[idRight1][0].endPointY + c2) / length2;
                pRatio1R = (disS + disE) / length1;
                projRatioRight[idRight1][idRight2] = pRatio1R;

                /*project the end points of line2 onto line1 and compute their distances to line1;
                 */
                disS = fabs(a1 * linesInRight[idRight2][0].startPointX + b1 * linesInRight[idRight2][0].startPointY + c1) / length1;
                disE = fabs(a1 * linesInRight[idRight2][0].endPointX + b1 * linesInRight[idRight2][0].endPointY + c1) / length1;
                pRatio2R = (disS + disE) / length2;
                projRatioRight[idRight2][idRight1] = pRatio2R;

                //mark them as computed
                bComputedRight[idRight1][idRight2] = true;
                bComputedRight[idRight2][idRight1] = true;
            }
            else
            { //read these information from matrix;
                iRatio1R = intersecRatioRight[idRight1][idRight2];
                iRatio2R = intersecRatioRight[idRight2][idRight1];
                pRatio1R = projRatioRight[idRight1][idRight2];
                pRatio2R = projRatioRight[idRight2][idRight1];
            }
            pRatioDif = MIN(fabs(pRatio1L - pRatio1R), fabs(pRatio2L - pRatio2R));

			if (pRatioDif > ProjectionRationDifThreshold)
			{
				continue; //the projection length ratio difference is too large;
			}
			if ((iRatio1L == Inf) || (iRatio2L == Inf) || (iRatio1R == Inf) || (iRatio2R == Inf))
            {
				//don't consider the intersection length ratio
				similarity = 4 - desDisMat[idLeft1][idRight1] / DescriptorDifThreshold - desDisMat[idLeft2][idRight2] / DescriptorDifThreshold - pRatioDif / ProjectionRationDifThreshold - relativeAngleDif / RelativeAngleDifferenceThreshold;
                adjacenceVec[(2 * dim - j - 1) * j / 2 + i] = similarity;
				nnz++;
				matrix.at<float>(i, j) = similarity;
				//				testMat[i][j] = similarity;
				//				testMat[j][i] = similarity;
			}
			else
            {
				iRatioDif = MIN(fabs(iRatio1L - iRatio1R), fabs(iRatio2L - iRatio2R));
				if (iRatioDif > IntersectionRationDifThreshold)
				{
					continue; //the intersection length ratio difference is too large;
				}
				//now compute the similarity score between two line pairs.
				similarity = 5 - desDisMat[idLeft1][idRight1] / DescriptorDifThreshold - desDisMat[idLeft2][idRight2] / DescriptorDifThreshold - iRatioDif / IntersectionRationDifThreshold - pRatioDif / ProjectionRationDifThreshold - relativeAngleDif / RelativeAngleDifferenceThreshold;
				adjacenceVec[(2 * dim - j - 1) * j / 2 + i] = similarity;
				nnz++;
				matrix.at<float>(i, j) = similarity;
				//				testMat[i][j] = similarity;
				//				testMat[j][i] = similarity;
			}
		}
    }
	// pointer to an array that stores the nonzero elements of Adjacency matrix.
	double *adjacenceMat = new double[nnz];
	// pointer to an array that stores the row indices of the non-zeros in adjacenceMat.
	int *irow = new int[nnz];
	// pointer to an array of pointers to the beginning of each column of adjacenceMat.
	int *pcol = new int[dim + 1];
	int idOfNNZ = 0; //the order of none zero element
	pcol[0] = 0;
	unsigned int tempValue;
	for (unsigned int j = 0; j < dim; j++)
	{ //column
		for (unsigned int i = j; i < dim; i++)
		{ //row
			tempValue = (2 * dim - j - 1) * j / 2 + i;
			if (adjacenceVec[tempValue] != 0)
			{
				adjacenceMat[idOfNNZ] = adjacenceVec[tempValue];
				irow[idOfNNZ] = i;
				idOfNNZ++;
			}
		}
		pcol[j + 1] = idOfNNZ;
    }
	/*Third step, solve the principal eigenvector of the adjacency matrix using Arpack lib.
	 */
	ARluSymMatrix<double> arMatrix(dim, nnz, adjacenceMat, irow, pcol);
	ARluSymStdEig<double> dprob(2, arMatrix, "LM"); // Defining what we need: the first eigenvector of arMatrix with largest magnitude.
	// Finding eigenvalues and eigenvectors.
	dprob.FindEigenvectors();
    cout << "Number of 'converged' eigenvalues  : " << dprob.ConvergedEigenvalues() << endl;

	eigenMap_.clear();

	double meanEigenVec = 0;
	if (dprob.EigenvectorsFound())
	{
		double value;
		for (unsigned int j = 0; j < dim; j++)
		{
			value = fabs(dprob.Eigenvector(1, j));
			meanEigenVec += value;
			eigenMap_.insert(std::make_pair(value, j));
		}
	}
	minOfEigenVec_ = WeightOfMeanEigenVec * meanEigenVec / dim;
	delete[] adjacenceMat;
	delete[] irow;
	delete[] pcol;
}

void PairwiseLineMatching::MatchingResultFromPrincipalEigenvector_(
        const ScaleLines &linesInLeft, const ScalePoints &ptInLeft, const ScaleLines &linesInRight, const ScalePoints &ptInRight,
		std::vector<int > &matchResult, std::vector<cv::DMatch> &pt_matchs)
{
	double TwoPI = 2 * M_PI;
	std::vector<int> matchRet1;
	std::vector<unsigned int> matchRet2;
	double matchScore1 = 0;
	double matchScore2 = 0;
	EigenMAP mapCopy = eigenMap_;
	unsigned int dim = nodesList_.size();
	EigenMAP::iterator iter;
	unsigned int id, idLeft2, idRight2;
	double sideValueL, sideValueR;
	double pointX, pointY;
	double relativeAngleLeft, relativeAngleRight; //the relative angle of each line pair
	double relativeAngleDif;

	//store eigenMap for debug
	std::fstream resMap;
	ostringstream fileNameMap;
	fileNameMap << "eigenVec.txt";
	resMap.open(fileNameMap.str().c_str(), std::ios::out);
    
	int left_scale_size = linesInLeft.size() + ptInLeft.points.size();
	int right_scale_size = linesInRight.size() + ptInRight.points.size();
	std::vector<std::vector<double>> mat = {};
	mat.resize(left_scale_size);
	for (int i = 0; i < left_scale_size; i++) {
		mat[i].resize(right_scale_size);
	}
	mat.resize(left_scale_size);
	for (iter = eigenMap_.begin(); iter != eigenMap_.end(); iter++)
	{
		id = iter->second;
		resMap << nodesList_[id].leftLineID << "    " << nodesList_[id].rightLineID << "   " << iter->first << endl;
		mat[nodesList_[id].leftLineID][nodesList_[id].rightLineID] = iter->second;
	}
	// mat.Save("eigenMap.txt");
	matSave(mat, "eigenMap.txt");
	resMap.flush();
	resMap.close();
    cv::Mat line_show = show_image.clone();
    cv::Mat point_show = show_image.clone();
	std::fstream out("/home/tonglu/VO-LOAM/github/orb-slam3/build/emap.txt", std::ios_base::openmode::_S_out);
	out << std::setw(9);
	for (size_t i = 0; i < matrix.rows; ++i) {
		out << i << " : ";
		for (size_t j = 0; j < matrix.cols; ++j) {
			out << std::setw(9) << matrix.at<float>(i, j) << " ";
		}
		out << "\n";
		
	}
	// out << matrix << "\n";
	out.close();
	if (eigenMap_.empty()) return;
	
	/*first try, start from the top element in eigenmap */
	while (1)
	{
		iter = eigenMap_.begin();
		//if the top element in the map has small value, then there is no need to continue find more matching line pairs;
		if (iter->first < minOfEigenVec_)
		{
			break;
		}
		id = iter->second;
		cv::Scalar randcolor = tergeo::visualodometry::randColor();
		if (nodesList_[id].type == FeatureType::Point_) {
			std::cout << "match type: point\n";
			int left_pt = nodesList_[id].leftPtID - linesInLeft.size();
			int right_pt = nodesList_[id].rightPtID - linesInRight.size();
			pt_matchs.push_back(cv::DMatch(left_pt, right_pt, -1));
			cv::Point2f lpt = ptInLeft.points[left_pt][0].pt;
			cv::Point2f rpt = ptInRight.points[right_pt][0].pt + cv::Point2f(1280, 0);
			// cv::circle(point_show, lpt, 5, randcolor);
			// cv::circle(point_show, rpt, 5, randcolor);
			// cv::line(point_show, lpt, rpt, randcolor);
			// cv::imshow("point_show", point_show);
			// cv::waitKey();
			for (; iter->first >= minOfEigenVec_;) {
				id = iter->second;
				if (nodesList_[id].type == FeatureType::Line_){ iter++; continue; };
				idLeft2 = nodesList_[id].leftLineID - linesInLeft.size();
				idRight2 = nodesList_[id].rightLineID - linesInRight.size();
				//check one to one match condition
				if ((left_pt == idLeft2) || (right_pt == idRight2))
				{
					eigenMap_.erase(iter++);//也删除了顶部自身
					continue; //not satisfy the one to one match condition
				}
				iter++;
			}
			continue;
			
		}
		std::cout << "match type: line\n";
		unsigned int idLeft1 = nodesList_[id].leftLineID;
		unsigned int idRight1 = nodesList_[id].rightLineID;
		matchRet1.push_back(idLeft1);
		matchRet1.push_back(idRight1);
		matchScore1 += iter->first;
		eigenMap_.erase(iter++);
		cv::Point2f lspt(linesInLeft[idLeft1][0].startPointX, linesInLeft[idLeft1][0].startPointY);
		cv::Point2f lept(linesInLeft[idLeft1][0].endPointX, linesInLeft[idLeft1][0].endPointY);
		cv::Point2f rspt(linesInRight[idRight1][0].startPointX + 1280, linesInRight[idRight1][0].startPointY);
		cv::Point2f rept(linesInRight[idRight1][0].endPointX + 1280, linesInRight[idRight1][0].endPointY);
		// cv::line(line_show, lspt, lept, randcolor, 2);
		// cv::line(line_show, rspt, rept, randcolor, 2);
		// cv::line(line_show, lspt, rspt, tergeo::visualodometry::randColor(), 2);
		// cv::imshow("line_show", line_show);
		// cv::waitKey();
		//remove all potential assignments in conflict with top matched line pair
		double xe_xsLeft = linesInLeft[idLeft1][0].endPointX - linesInLeft[idLeft1][0].startPointX;
		double ye_ysLeft = linesInLeft[idLeft1][0].endPointY - linesInLeft[idLeft1][0].startPointY;
		double xe_xsRight = linesInRight[idRight1][0].endPointX - linesInRight[idRight1][0].startPointX;
		double ye_ysRight = linesInRight[idRight1][0].endPointY - linesInRight[idRight1][0].startPointY;
		double coefLeft = sqrt(xe_xsLeft * xe_xsLeft + ye_ysLeft * ye_ysLeft);
		double coefRight = sqrt(xe_xsRight * xe_xsRight + ye_ysRight * ye_ysRight);
		for (; iter->first >= minOfEigenVec_;)
		{
			id = iter->second;
			if (nodesList_[id].type == FeatureType::Point_){ iter++; continue; };
			idLeft2 = nodesList_[id].leftLineID;
			idRight2 = nodesList_[id].rightLineID;
			//check one to one match condition
			if ((idLeft1 == idLeft2) || (idRight1 == idRight2))
			{
				eigenMap_.erase(iter++);
				continue; //not satisfy the one to one match condition
			}
			//check sidedness constraint, the middle point of line2 should lie on the same side of line1.
			//sideValue = (y-ys)*(xe-xs)-(x-xs)*(ye-ys);
			pointX = 0.5 * (linesInLeft[idLeft2][0].startPointX + linesInLeft[idLeft2][0].endPointX);
			pointY = 0.5 * (linesInLeft[idLeft2][0].startPointY + linesInLeft[idLeft2][0].endPointY);
			sideValueL = (pointY - linesInLeft[idLeft1][0].startPointY) * xe_xsLeft - (pointX - linesInLeft[idLeft1][0].startPointX) * ye_ysLeft;
			sideValueL = sideValueL / coefLeft;
			pointX = 0.5 * (linesInRight[idRight2][0].startPointX + linesInRight[idRight2][0].endPointX);
			pointY = 0.5 * (linesInRight[idRight2][0].startPointY + linesInRight[idRight2][0].endPointY);
			sideValueR = (pointY - linesInRight[idRight1][0].startPointY) * xe_xsRight - (pointX - linesInRight[idRight1][0].startPointX) * ye_ysRight;
			sideValueR = sideValueR / coefRight;
			if (sideValueL * sideValueR < 0 && fabs(sideValueL) > 5 && fabs(sideValueR) > 5)
			{ //have the different sign, conflict happens.
				eigenMap_.erase(iter++);
				continue;
			}
			//check relative angle difference
			relativeAngleLeft = linesInLeft[idLeft1][0].direction - linesInLeft[idLeft2][0].direction;
			relativeAngleLeft = (relativeAngleLeft < M_PI) ? relativeAngleLeft : (relativeAngleLeft - TwoPI);
			relativeAngleLeft = (relativeAngleLeft > (-M_PI)) ? relativeAngleLeft : (relativeAngleLeft + TwoPI);
			relativeAngleRight = linesInRight[idRight1][0].direction - linesInRight[idRight2][0].direction;
			relativeAngleRight = (relativeAngleRight < M_PI) ? relativeAngleRight : (relativeAngleRight - TwoPI);
			relativeAngleRight = (relativeAngleRight > (-M_PI)) ? relativeAngleRight : (relativeAngleRight + TwoPI);
			relativeAngleDif = fabs(relativeAngleLeft - relativeAngleRight);
			if ((TwoPI - relativeAngleDif) > RelativeAngleDifferenceThreshold && relativeAngleDif > RelativeAngleDifferenceThreshold)
			{
				eigenMap_.erase(iter++);
				continue; //the relative angle difference is too large;
			}
			iter++;
		}
	} //end while(stillLoop)
	matchResult = matchRet1;
	

	cout << "matchRet1.size=" << matchRet1.size() << ", minOfEigenVec_= " << minOfEigenVec_ << endl;
}


//根据光流角度建立二维map，设置阈值聚类，筛除离群值
void PairwiseLineMatching::consistencyCheck(
	const ScaleLines &linesInLeft, const ScalePoints &leftPt, const ScaleLines &linesInRight, const ScalePoints &rightPt, 
	std::vector<int> &matchResult, std::vector<cv::DMatch> &pt_matchs) {
	// int lmatch_size = matchResult.size() / 2;
	// int ptmatch_size = pt_matchs.size();
	// //划分匹配光流空间
	// double an_resolution = 10;
	// double dis_resolution = 900;//对应30个像素
	// //       角度           距离                       索引            真实角度 真实距离
	// std::map<int, std::map<int, std::vector<std::pair<int, std::pair<double, double>>>>> match_map = {};
	// typedef std::pair<int, std::pair<double, double>> match_info;
	// for (size_t i = 0; i < ptmatch_size; ++i) {
	// 	int left_id = pt_matchs[i].queryIdx;
	// 	int right_id = pt_matchs[i].queryIdx;
	// 	const cv::Point2f &lpt =  leftPt.points[left_id][0].pt;
	// 	const cv::Point2f &rpt =  rightPt.points[left_id][0].pt;
	// 	double dx = rpt.x - lpt.x;
	// 	double dy = rpt.y - lpt.y;
	// 	double length2 = (dx * dx + dx * dy) / dis_resolution;
	// 	double angle = (std::atan2(dy, dx) / M_PI * 180) / an_resolution;
	// 	match_map[angle][length2].push_back(match_info(i, std::make_pair(angle, length2)));
	// }
	// int max = 0;
	// auto _ait = match_map.end();
	// auto _dit = _ait->second.end();
	// for (auto ait = match_map.begin(); ait!= match_map.end(); ++ait) {
	// 	for (auto dit = ait->second.begin(); dit!= ait->second.end(); ++dit) {
	// 		if (max < dit->second.size()) {
	// 			max = dit->second.size();
	// 			_ait = ait;
	// 			_dit = dit;
	// 		}
	// 	}
	// }
	// //to do大尺度点删除比较近的，避免这里分类比较近
	// //离群点正好在栅格中心，除以八
	// if (max > ptmatch_size / 4) {
	// 	/* code */
	// }
    
	//to do 还没做线段的检查
	int lmatch_size = matchResult.size() / 2;
	int ptmatch_size = pt_matchs.size();
	//划分匹配光流空间
	double mean_dis = 0;
	double mean_ang = 0;//对应30个像素
	double dis;
	// double ang;
	std::vector<double> dis_vec = {};
	std::vector<double> angle_vec = {};
	for (size_t i = 0; i < ptmatch_size; ++i) {
		int left_id = pt_matchs[i].queryIdx;
		int right_id = pt_matchs[i].trainIdx;
		const cv::Point2f &lpt =  leftPt.points[left_id][0].pt;
		const cv::Point2f &rpt =  rightPt.points[right_id][0].pt;
		double dx = rpt.x - lpt.x;
		double dy = rpt.y - lpt.y;
		dis = std::sqrt(dx * dx + dy * dy);
		// ang = std::atan2(dy, dx) / M_PI * 180;
		dis_vec.push_back(dis);
		// angle_vec.push_back(ang);
		mean_dis += dis;
		// mean_ang += ang;

	}
	// mean_ang /= ptmatch_size;
	mean_dis /= ptmatch_size;
	//随机分布，假设检验去除
	std::vector<double> dis_diff_vec = {};
	// std::vector<double> angle_diff_vec = {};
	double D_RMSE = 0;
	// double AN_RMSE = 0;
	for (size_t i = 0; i < ptmatch_size; ++i) {
		double dis_diff = (dis_vec[i] - mean_dis) * (dis_vec[i] - mean_dis);
		// double ang_diff = (angle_vec[i] - mean_ang) * (angle_vec[i] - mean_ang);
		dis_diff_vec.push_back(dis_diff);
		// angle_diff_vec.push_back(ang_diff);
		D_RMSE += dis_diff;
		// AN_RMSE += ang_diff;
	}
	D_RMSE /= (ptmatch_size - 1);
	// AN_RMSE /= (ptmatch_size - 1);
	//角度用全局偏移量判断 to do
	//三倍中误差
	if (D_RMSE > 900) {
		pt_matchs.clear();
		return;
	}
	// if (D_RMSE > 30 || AN_RMSE > 10) {
	// 	pt_matchs.clear();
	// 	return;
	// }
	//这里要用int，因为size_t下(0 - 1)为正数
	for (int i = ptmatch_size - 1; i >= 0; --i) {
		// if (dis_diff_vec[i] > 9 * D_RMSE || angle_diff_vec[i] > 9 * AN_RMSE) {
		// 	pt_matchs.erase(pt_matchs.begin() + i);
		// }
		if (dis_diff_vec[i] > 9 * D_RMSE) {
			pt_matchs.erase(pt_matchs.begin() + i);
		}
	}
}