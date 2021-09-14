#include "optimizer.h"

namespace tergeo{
namespace visualodometry {

void Optimizer::load(std::vector<MatchPair> &match_, 
        std::vector<KeyFrame> &keyframe_, 
        const std::vector<int> &opt_vec_) {
    opt_vec = std::move(opt_vec_);
    std::cout << opt_vec_.size() << "\n";
    point_count = 0;
    camera_count = 0;
    std::set<int> point_set = {};
    std::set<int> camera_set = {};
    for (int i = 0; i < opt_vec.size(); ++i) {
        camera_set.insert(match_[opt_vec[i]]._left_image_id);
        camera_set.insert(match_[opt_vec[i]]._right_image_id);
    }
    for (auto it = camera_set.begin(); it != camera_set.end(); ++it) {
        int id = *it;
        point_count += keyframe->at(id)._threeDPts.size();
    }
    
    camera_count = camera_set.size();
    
    camera = new double[camera_count];
    threept = new double[point_count];
}
void Optimizer::Optimize() {
    ceres::Problem problem;
    for (int i = 0; i < point_count; ++i) {
        ceres::CostFunction* cost_function = SnavelyReprojectionError::Create(
            pixpts[2 * i + 0], pixpts[2 * i + 1]);
        problem.AddResidualBlock(cost_function,
                                NULL /* squared loss */,
                                camera + camera_index[i] * 9,
                                threept + point_index[i] * 3);
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    problem.SetParameterUpperBound(camera, -1, 1);
    problem.SetParameterUpperBound(camera, -1, 1);

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    //根据opt_vec更新回去
}

OptimizeObject Optimizer::getOptimizeObject(OPTIMIZE_TYPE type) {
    std::cout << "test Function\n";
    // getTypePtr func = type_map[0];
    // func();
    type_map[0]();
    type_map[1]();
    type_map[2]();
    type_map[3]();
    type_map[4]();
    return OptimizeObject();
}

void Optimizer::OptimizeFullCamera() {
    std::cout << "OptimizeFullCamera\n";
}

void Optimizer::OptimizeSO3() {
    std::cout << "OptimizeSO3\n";
}

void Optimizer::OptimizeYaw() {
    std::cout << "OptimizeYaw\n";
}

void Optimizer::OptimizePitch() {
    std::cout << "OptimizePitch\n";
}

void Optimizer::OptimizeRoll() {
    std::cout << "OptimizeRoll\n";
}


}
}