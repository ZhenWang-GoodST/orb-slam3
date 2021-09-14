#ifndef TERGEO_VISUAL_ODOMETRY_OPTIMIZER
#define TERGEO_VISUAL_ODOMETRY_OPTIMIZER

#include "types.hpp"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

namespace tergeo{
namespace visualodometry {

enum OPTIMIZE_TYPE {
    FULL_CAMERA, SO2, SO3, ROLL, PICTH, YAW
};

typedef void (*getTypePtr)();
struct OptimizeObject {
    
    static OptimizeObject getType();
};


struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y)
      : _observed_x(observed_x), _observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = -p[0] / p[2];
    T yp = -p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = camera[7];
    const T& l2 = camera[8];
    T r2 = xp * xp + yp * yp;
    T distortion = 1.0 + r2 * (l1 + l2 * r2);

    // Compute final projected point position.
    const T& focal = camera[6];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - _observed_x;
    residuals[1] = predicted_y - _observed_y;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
        new SnavelyReprojectionError(observed_x, observed_y)));
  }

  double _observed_x;
  double _observed_y;
};

class Optimizer
{
private:
    // std::shared_ptr<std::vector<Match>>  match = nullptr;
    // std::shared_ptr<std::vector<KeyFrame>>  keyframe = nullptr;
    std::vector<MatchPair>  *match = nullptr;
    std::vector<KeyFrame>  *keyframe = nullptr;
    std::vector<int> opt_vec = {};
    //输入有相机位姿，2d点坐标，三维坐标，优化相机位姿，使得2d点坐标
    //重投影误差最小
    int point_count = 0;
    int camera_count = 0;
    OptimizeObject optimize_object;
    double *pixpts = nullptr;
    double *camera = nullptr;
    double *threept = nullptr;
    int *camera_index;
    int *point_index;

    //
    std::map<int, getTypePtr> type_map = {};
    
public:

public:
    Optimizer() {
        type_map[0] = &this->OptimizeFullCamera;
        type_map[1] = &this->OptimizeSO3;
        type_map[2] = &this->OptimizeYaw;
        type_map[3] = &this->OptimizePitch;
        type_map[4] = &this->OptimizeRoll;
    }
    ~Optimizer() {}
    void load(std::vector<MatchPair> &match_, 
        std::vector<KeyFrame> &keyframe_, 
        const std::vector<int> &opt_vec_);
    //
    void Optimize();
    OptimizeObject getOptimizeObject(OPTIMIZE_TYPE type);
    
    static void OptimizeFullCamera();

    static void OptimizeSO3();

    static void OptimizeYaw();
    
    static void OptimizePitch();
    
    static void OptimizeRoll();
};

}
}
#endif