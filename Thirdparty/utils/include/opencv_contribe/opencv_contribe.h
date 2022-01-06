#ifndef OPENCV_CONTRIBE_CUSTOMM
#define OPENCV_CONTRIBE_CUSTOMM
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
// #include "opencv2/core/private.hpp"

namespace cv {
namespace cv_contribe {
//来自4.2.0的关键字
#ifndef CV_OVERRIDE
#  define CV_OVERRIDE override
#endif
#ifndef CV_FINAL
#  define CV_FINAL final
#endif
template <typename T>
bool findXIntersection(const double x, const T& seg_a, const T& seg_b, T* pt) {
    if ((seg_a.x - x) * (seg_b.x - x) > 0) return false;
    if (std::abs(seg_a.x - seg_b.x) < 1e-4) return false;//horizontal
    double scale = (x - seg_a.x) / (seg_b.x - seg_a.x);
    pt->x = x;
    if(std::abs(seg_a.y - seg_b.y)<1e-4){
        pt->y = seg_a.y;
    }else{
        pt->y = seg_a.y + scale * (seg_b.y - seg_a.y);
    }
    return true;
}

template <typename T>
bool findYIntersection(const double y, const T& seg_a, const T& seg_b, T* pt) {
    if ((seg_a.y - y) * (seg_b.y - y) > 0) return false;
    if (std::abs(seg_a.y - seg_b.y) < 1e-4) return false;//horizontal
    double scale = (y - seg_a.y) / (seg_b.y - seg_a.y);
    pt->y = y;
    if(std::abs(seg_a.x - seg_b.x)<1e-4){
        pt->x = seg_a.x;
    }else{
        pt->x = seg_a.x + scale * (seg_b.x - seg_a.x);
    }
    return true;
}

// //! Variants of Line Segment %Detector
// //! @ingroup imgproc_feature
// enum LineSegmentDetectorModes {
//     LSD_REFINE_NONE = 0, //!< No refinement applied
//     LSD_REFINE_STD  = 1, //!< Standard refinement is applied. E.g. breaking arches into smaller straighter line approximations.
//     LSD_REFINE_ADV  = 2  //!< Advanced refinement. Number of false alarms is calculated, lines are
//                          //!< refined through increase of precision, decrement in size, etc.
// };

/** @example lsd_lines.cpp
An example using the LineSegmentDetector
*/

/** @brief Line segment detector class

following the algorithm described at @cite Rafael12 .
*/
class CV_EXPORTS_W LineSegmentDetector : public Algorithm
{
public:
    cv::Mat mask;
    /** @brief Finds lines in the input image.

    This is the output of the default parameters of the algorithm on the above shown image.

    ![image](pics/building_lsd.png)

    @param _image A grayscale (CV_8UC1) input image. If only a roi needs to be selected, use:
    `lsd_ptr-\>detect(image(roi), lines, ...); lines += Scalar(roi.x, roi.y, roi.x, roi.y);`
    @param _lines A vector of Vec4i or Vec4f elements specifying the beginning and ending point of a line. Where
    Vec4i/Vec4f is (x1, y1, x2, y2), point 1 is the start, point 2 - end. Returned lines are strictly
    oriented depending on the gradient.
    @param width Vector of widths of the regions, where the lines are found. E.g. Width of line.
    @param prec Vector of precisions with which the lines are found.
    @param nfa Vector containing number of false alarms in the line region, with precision of 10%. The
    bigger the value, logarithmically better the detection.
    - -1 corresponds to 10 mean false alarms
    - 0 corresponds to 1 mean false alarm
    - 1 corresponds to 0.1 mean false alarms
    This vector will be calculated only when the objects type is LSD_REFINE_ADV.
    */
    CV_WRAP virtual void detect(InputArray _image, OutputArray _lines,
                        OutputArray width = noArray(), OutputArray prec = noArray(),
                        OutputArray nfa = noArray()) = 0;

    /** @brief Draws the line segments on a given image.
    @param _image The image, where the liens will be drawn. Should be bigger or equal to the image,
    where the lines were found.
    @param lines A vector of the lines that needed to be drawn.
     */
    CV_WRAP virtual void drawSegments(InputOutputArray _image, InputArray lines) = 0;

    /** @brief Draws two groups of lines in blue and red, counting the non overlapping (mismatching) pixels.

    @param size The size of the image, where lines1 and lines2 were found.
    @param lines1 The first group of lines that needs to be drawn. It is visualized in blue color.
    @param lines2 The second group of lines. They visualized in red color.
    @param _image Optional image, where the lines will be drawn. The image should be color(3-channel)
    in order for lines1 and lines2 to be drawn in the above mentioned colors.
     */
    CV_WRAP virtual int compareSegments(const Size& size, InputArray lines1, InputArray lines2, InputOutputArray _image = noArray()) = 0;

    virtual ~LineSegmentDetector() { }
};

/** @brief Creates a smart pointer to a LineSegmentDetector object and initializes it.

The LineSegmentDetector algorithm is defined using the standard values. Only advanced users may want
to edit those, as to tailor it for their own application.

@param _refine The way found lines will be refined, see cv::LineSegmentDetectorModes
@param _scale The scale of the image that will be used to find the lines. Range (0..1].
@param _sigma_scale Sigma for Gaussian filter. It is computed as sigma = _sigma_scale/_scale.
@param _quant Bound to the quantization error on the gradient norm.
@param _ang_th Gradient angle tolerance in degrees.
@param _log_eps Detection threshold: -log10(NFA) \> log_eps. Used only when advancent refinement
is chosen.
@param _density_th Minimal density of aligned region points in the enclosing rectangle.
@param _n_bins Number of bins in pseudo-ordering of gradient modulus.
 */
CV_EXPORTS_W Ptr<LineSegmentDetector> createLineSegmentDetector(
    int _refine = LSD_REFINE_STD, double _scale = 0.8,
    double _sigma_scale = 0.6, double _quant = 2.0, double _ang_th = 22.5,
    double _log_eps = 0, double _density_th = 0.7, int _n_bins = 1024, double _length_ratio = 0.125);

//! @} imgproc_feature



//! @addtogroup features2d_main
//! @{

/** @brief Wrapping class for feature detection using the AGAST method. :
 */
class CV_EXPORTS_W AgastFeatureDetector : public Feature2D
{
public:
    enum DetectorType
    {
        AGAST_5_8 = 0, AGAST_7_12d = 1, AGAST_7_12s = 2, OAST_9_16 = 3,
    };

    enum
    {
        THRESHOLD = 10000, NONMAX_SUPPRESSION = 10001,
    };

    CV_WRAP static Ptr<AgastFeatureDetector> create( int threshold=10,
                                                     bool nonmaxSuppression=true,
                                                     AgastFeatureDetector::DetectorType type = AgastFeatureDetector::OAST_9_16);

    CV_WRAP virtual void setThreshold(int threshold) = 0;
    CV_WRAP virtual int getThreshold() const = 0;

    CV_WRAP virtual void setNonmaxSuppression(bool f) = 0;
    CV_WRAP virtual bool getNonmaxSuppression() const = 0;

    CV_WRAP virtual void setType(AgastFeatureDetector::DetectorType type) = 0;
    CV_WRAP virtual AgastFeatureDetector::DetectorType getType() const = 0;
    CV_WRAP virtual String getDefaultName() const CV_OVERRIDE;
};

/** @overload */
CV_EXPORTS void AGAST( InputArray image, CV_OUT std::vector<KeyPoint>& keypoints,
                      int threshold, bool nonmaxSuppression=true );

/** @brief Detects corners using the AGAST algorithm

@param image grayscale image where keypoints (corners) are detected.
@param keypoints keypoints detected on the image.
@param threshold threshold on difference between intensity of the central pixel and pixels of a
circle around this pixel.
@param nonmaxSuppression if true, non-maximum suppression is applied to detected corners
(keypoints).
@param type one of the four neighborhoods as defined in the paper:
AgastFeatureDetector::AGAST_5_8, AgastFeatureDetector::AGAST_7_12d,
AgastFeatureDetector::AGAST_7_12s, AgastFeatureDetector::OAST_9_16

For non-Intel platforms, there is a tree optimised variant of AGAST with same numerical results.
The 32-bit binary tree tables were generated automatically from original code using perl script.
The perl script and examples of tree generation are placed in features2d/doc folder.
Detects corners using the AGAST algorithm by @cite mair2010_agast .

 */
CV_EXPORTS void AGAST( InputArray image, CV_OUT std::vector<KeyPoint>& keypoints,
                      int threshold, bool nonmaxSuppression, AgastFeatureDetector::DetectorType type );


//! @addtogroup features2d_main
//! @{

/** @brief Class implementing the BRISK keypoint detector and descriptor extractor, described in @cite LCS11 .
 */
class CV_EXPORTS_W BRISK : public Feature2D
{
public:
    /** @brief The BRISK constructor

    @param thresh AGAST detection threshold score.
    @param octaves detection octaves. Use 0 to do single scale.
    @param patternScale apply this scale to the pattern used for sampling the neighbourhood of a
    keypoint.
     */
    CV_WRAP static Ptr<BRISK> create(int thresh=30, int octaves=3, float patternScale=1.0f);

    /** @brief The BRISK constructor for a custom pattern

    @param radiusList defines the radii (in pixels) where the samples around a keypoint are taken (for
    keypoint scale 1).
    @param numberList defines the number of sampling points on the sampling circle. Must be the same
    size as radiusList..
    @param dMax threshold for the short pairings used for descriptor formation (in pixels for keypoint
    scale 1).
    @param dMin threshold for the long pairings used for orientation determination (in pixels for
    keypoint scale 1).
    @param indexChange index remapping of the bits. */
    CV_WRAP static Ptr<BRISK> create(const std::vector<float> &radiusList, const std::vector<int> &numberList,
        float dMax=5.85f, float dMin=8.2f, const std::vector<int>& indexChange=std::vector<int>());

    /** @brief The BRISK constructor for a custom pattern, detection threshold and octaves

    @param thresh AGAST detection threshold score.
    @param octaves detection octaves. Use 0 to do single scale.
    @param radiusList defines the radii (in pixels) where the samples around a keypoint are taken (for
    keypoint scale 1).
    @param numberList defines the number of sampling points on the sampling circle. Must be the same
    size as radiusList..
    @param dMax threshold for the short pairings used for descriptor formation (in pixels for keypoint
    scale 1).
    @param dMin threshold for the long pairings used for orientation determination (in pixels for
    keypoint scale 1).
    @param indexChange index remapping of the bits. */
    CV_WRAP static Ptr<BRISK> create(int thresh, int octaves, const std::vector<float> &radiusList,
        const std::vector<int> &numberList, float dMax=5.85f, float dMin=8.2f,
        const std::vector<int>& indexChange=std::vector<int>());
    CV_WRAP virtual String getDefaultName() const CV_OVERRIDE;

    /** @brief Set detection threshold.
    @param threshold AGAST detection threshold score.
    */
    CV_WRAP virtual void setThreshold(int threshold) { CV_UNUSED(threshold); return; }
    CV_WRAP virtual int getThreshold() const { return -1; }

    /** @brief Set detection octaves.
    @param octaves detection octaves. Use 0 to do single scale.
    */
    CV_WRAP virtual void setOctaves(int octaves) { CV_UNUSED(octaves); return; }
    CV_WRAP virtual int getOctaves() const { return -1; }
};

}

}
namespace cv {
namespace xfeatures2d {
namespace cv_contribe {
//from opencv4.2.0 nonfree.hpp
/** @brief Class for extracting Speeded Up Robust Features from an image @cite Bay06 .

The algorithm parameters:
-   member int extended
    -   0 means that the basic descriptors (64 elements each) shall be computed
    -   1 means that the extended descriptors (128 elements each) shall be computed
-   member int upright
    -   0 means that detector computes orientation of each feature.
    -   1 means that the orientation is not computed (which is much, much faster). For example,
if you match images from a stereo pair, or do image stitching, the matched features
likely have very similar angles, and you can speed up feature extraction by setting
upright=1.
-   member double hessianThreshold
Threshold for the keypoint detector. Only features, whose hessian is larger than
hessianThreshold are retained by the detector. Therefore, the larger the value, the less
keypoints you will get. A good default value could be from 300 to 500, depending from the
image contrast.
-   member int nOctaves
The number of a gaussian pyramid octaves that the detector uses. It is set to 4 by default.
If you want to get very large features, use the larger value. If you want just small
features, decrease it.
-   member int nOctaveLayers
The number of images within each octave of a gaussian pyramid. It is set to 2 by default.
@note
   -   An example using the SURF feature detector can be found at
        opencv_source_code/samples/cpp/generic_descriptor_match.cpp
    -   Another example using the SURF feature detector, extractor and matcher can be found at
        opencv_source_code/samples/cpp/matcher_simple.cpp
 */


class CV_EXPORTS_W SURF : public Feature2D
{
public:
   
   
    /** @brief Computes the descriptors for a set of keypoints detected in an image (first variant) or image set
    (second variant).

    @param image Image.
    @param keypoints Input collection of keypoints. Keypoints for which a descriptor cannot be
    computed are removed. Sometimes new keypoints can be added, for example: SIFT duplicates keypoint
    with several dominant orientations (for each orientation).
    @param descriptors Computed descriptors. In the second variant of the method descriptors[i] are
    descriptors computed for a keypoints[i]. Row j is the keypoints (or keypoints[i]) is the
    descriptor for keypoint j-th keypoint.
     */
    // CV_WRAP void compute( InputArray image,
    //                               CV_OUT CV_IN_OUT std::vector<KeyPoint>& keypoints,
    //                               OutputArray descriptors ) CV_OVERRIDE;

   
    /**
    @param hessianThreshold Threshold for hessian keypoint detector used in SURF.
    @param nOctaves Number of pyramid octaves the keypoint detector will use.
    @param nOctaveLayers Number of octave layers within each octave.
    @param extended Extended descriptor flag (true - use extended 128-element descriptors; false - use
    64-element descriptors).
    @param upright Up-right or rotated features flag (true - do not compute orientation of features;
    false - compute orientation).
     */
    CV_WRAP static Ptr<SURF> create(double hessianThreshold=100,
                  int nOctaves = 4, int nOctaveLayers = 3,
                  bool extended = false, bool upright = false);
    
    CV_WRAP virtual void computeFixedAngle(InputArray _img, InputArray _mask,
                      CV_OUT std::vector<KeyPoint>& keypoints,
                      OutputArray _descriptors,
                      bool useProvidedKeypoints) = 0;

    CV_WRAP virtual void setHessianThreshold(double hessianThreshold) = 0;
    CV_WRAP virtual double getHessianThreshold() const = 0;

    CV_WRAP virtual void setNOctaves(int nOctaves) = 0;
    CV_WRAP virtual int getNOctaves() const = 0;

    CV_WRAP virtual void setNOctaveLayers(int nOctaveLayers) = 0;
    CV_WRAP virtual int getNOctaveLayers() const = 0;

    CV_WRAP virtual void setExtended(bool extended) = 0;
    CV_WRAP virtual bool getExtended() const = 0;

    CV_WRAP virtual void setUpright(bool upright) = 0;
    CV_WRAP virtual bool getUpright() const = 0;
};

typedef SURF SurfFeatureDetector;
typedef SURF SurfDescriptorExtractor;

//! @}
}
}
}


#endif