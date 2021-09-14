#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/squared_distance_2.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Segment_2<Kernel>                           Segment_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef std::vector<Polygon_with_holes_2>                 Pwh_list_2;
namespace tergeo{
namespace visualodometry{

// bool dubinsShot(
//     const DSPoint& start, const DSPoint& goal,
//     std::vector<DSPoint>& rs_path,
//     double radius, double& len, double resolution) {
    
//     namespace ob = ompl::base;
//     namespace og = ompl::geometric;
//     ob::StateSpacePtr space(
//         std::make_shared<ob::DubinsStateSpace>(radius));
//     ob::ScopedState<> from(space), to(space), s(space);
//     from[0] = start.x; from[1] = start.y; from[2] = start.theta;
//     to[0] = goal.x; to[1] = goal.y; to[2] = goal.theta;
//     std::vector<double> reals;
//     len = space->distance(from(), to()); 
//     int num_pts = len / resolution;    
//     for (unsigned int i=0; i<=num_pts; ++i){
//         space->interpolate(from(), to(), (double)i/num_pts, s());
//         reals = s.reals();
//         rs_path.push_back(DSPoint(reals[0], reals[1] , reals[2]));
//     }
// }


}
}