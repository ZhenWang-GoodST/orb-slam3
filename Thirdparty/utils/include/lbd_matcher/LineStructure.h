

#ifndef LINESTRUCTURE_HH_
#define LINESTRUCTURE_HH_

#include <vector>
// A 2D line (normal equation parameters).
struct SingleLine
{
	//note: rho and theta are based on coordinate origin, i.e. the top-left corner of image
	double rho;//unit: pixel length
	double theta;//unit: rad
	double linePointX;// = rho * cos(theta);
	double linePointY;// = rho * sin(theta);
	//for EndPoints, the coordinate origin is the top-left corner of image.
	double startPointX;
	double startPointY;
	double endPointX;
	double endPointY;
	//direction of a line, the angle between positive line direction (dark side is in the left) and positive X axis.
	double direction;
	//mean gradient magnitude
	double gradientMagnitude;
	//mean gray value of pixels in dark side of line
	double darkSideGrayValue;
	//mean gray value of pixels in light side of line
	double lightSideGrayValue;
	//the length of line
	double lineLength;
	//the width of line;
	double width;
	//number of pixels
	int numOfPixels;
	//the decriptor of line
	std::vector<double> descriptor;
};

// Specifies a vector of lines.
typedef std::vector<SingleLine> Lines_list;

enum FeatureType {
	Point_, Line_
};
//线段应该进行采样，重新设置可见性，采样点根据变换关系进行对其匹配，恢复线段位姿
//采样应该计算垂足点
struct OctaveSingleLine
{
	FeatureType type = FeatureType::Point_;
	/*endPoints, the coordinate origin is the top-left corner of the original image.
	 *startPointX = sPointInOctaveX * (factor)^octaveCount;	*/
	float startPointX;// 直线起点/大尺度特征点坐标
	float startPointY;
	float endPointX;
	float endPointY;
	//endPoints, the coordinate origin is the top-left corner of the octave image.
	float sPointInOctaveX;
	float sPointInOctaveY;
	float ePointInOctaveX;
	float ePointInOctaveY;
	float midX;
	float midY;
	//direction of a line, the angle between positive line direction (dark side is in the left) and positive X axis.
	float direction;
	//the summation of gradient magnitudes of pixels on lines
	float salience;
	//the length of line
	float lineLength;
	//number of pixels
	unsigned int numOfPixels;
	//the octave which this line is detected
	unsigned int octaveCount;
	//the decriptor of line
    std::vector<float> descriptor;
	cv::Mat pt_descriptor;
	int descriptor_size;
	std::vector<cv::KeyPoint> keypoints = {};
	void computeElements(double segment_length);
	void draw(cv::Mat &image) const;
};

// Specifies a vector of lines.
typedef std::vector<OctaveSingleLine> LinesVec;

typedef std::vector<LinesVec> ScaleLines;//each element in ScaleLines is a vector of lines which corresponds the same line detected in different octave images.

class ScalePoint : public cv::KeyPoint {
public:
	std::vector<int> hierarchy = {};
};

//  0：父级节点索引， 1 - n：size内包含索引
typedef std::vector<int> Hierarchy;

struct ScalePoints
{
	int dim_of_descriptor = 64;
	//对于层级关系，底层只存父级索引，长度为1，高层存包含的所有底层特征点索引
	std::vector<Hierarchy> hierarchy = {};
	std::vector<std::vector<cv::KeyPoint>> points;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptor;
};

struct linematch_score
{
    int id;
    double distance_ratio;
    double descriptor_dis;
    double affine_score;
};

// std::pair<int, linematch_score> linematch_pair;
struct LineMatchCmp {
    bool operator()(linematch_score p1, linematch_score p2) {
        if (p1.affine_score < p2.affine_score) {
            return false;
        } else if (p1.distance_ratio < p2.distance_ratio) {
            return false;
        } else if (p1.descriptor_dis > p2.descriptor_dis) {
            return false;
        }
        return true;
    }
};

#endif /* LINESTRUCTURE_HH_ */
