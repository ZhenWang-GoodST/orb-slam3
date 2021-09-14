#ifndef _TERGEO_PLANNING_ALGORITHM_COVERAGE_UTILS_H_
#define _TERGEO_PLANNING_ALGORITHM_COVERAGE_UTILS_H_
#include "base_coverage_planner.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/squared_distance_2.h>
typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point_2;
typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
typedef CGAL::Segment_2<Kernel>                           Segment_2;
typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
typedef std::vector<Polygon_with_holes_2>                 Pwh_list_2;

namespace tergeo{
namespace visualodometry{

/**
 * @brief 
 * 
 * @param poly 
 * @param border_offset 
 * @param out 
 * @return true 
 * @return false 
 */
bool zoomInPolygon(
    const std::vector<Point>& poly, double border_offset, std::vector<Point>& out);


// int zoomInPolygon(
//     const std::vector<std::vector<Point>> &polys, double border_offset, 
//     std::vector<std::vector<Point>>& outs);

template <typename T>
void printVec(const std::vector<T> &vec_data){
    for(int i = 0; i < vec_data.size(); ++i) {
        std::cout << i << " " << vec_data[i] << "\n";
    }
}
// bool dubinsShot(const DSPoint& start,
//                 const DSPoint& goal,
//                 std::vector<DSPoint>& rs_path,
//                 double radius,
//                 double& len,
//                 double resolution);


// double isPointInPoly(const std::vector<Point>& polygon,
//                     double x, double y);

// find line segment's y coordinate at given x
// bool findIntersection(const double x, 
//     const Point& seg_a, const Point& seg_b, Point* pt);

// void getBoundary(const std::vector<Point>& poly, double* minx,
//     double* miny, double* maxx, double* maxy);

// interpolate points between a and b, using the map's resolution
template <typename T>
void interpolate(const T&a, const T&b, std::vector<T>* pts, double length);

// void findLongestEdgeIndex(const std::vector<Point>& polygon, int* idx);


}
}
#endif