#ifndef _TERGEO_PLANNING_ALGORITHM_COVERAGE_MATH_H_
#define _TERGEO_PLANNING_ALGORITHM_COVERAGE_MATH_H_
#include "base_coverage_planner.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
// #include <ompl/base/spaces/DubinsStateSpace.h>
// #include <ompl/geometric/SimpleSetup.h>
// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>
// #include <CGAL/Boolean_set_operations_2.h>
// #include <CGAL/squared_distance_2.h>
// typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
// typedef Kernel::Point_2                                   Point_2;
// typedef CGAL::Polygon_2<Kernel>                           Polygon_2;
// typedef CGAL::Segment_2<Kernel>                           Segment_2;
// typedef CGAL::Polygon_with_holes_2<Kernel>                Polygon_with_holes_2;
// typedef std::vector<Polygon_with_holes_2>                 Pwh_list_2;
namespace tergeo{
namespace visualodometry{

// template <typename T>
// void toClockWise(std::vector<T> &in_p) {
//     Polygon_2 P;
//     for (int i=0; i < in_p.size(); ++i) {
//         P.push_back(Point_2(in_p[i].x, in_p[i].y));
//     }
//     if (!P.is_clockwise_oriented()) {
//         reverse(in_p.begin(), in_p.end());
//     } 
// }


// template <typename T>
// void toCounterClockWise(std::vector<T> &in_p) {
//     Polygon_2 P;
//     for (int i=0; i < in_p.size(); ++i) {
//         P.push_back(Point_2(in_p[i].x, in_p[i].y));
//     }
//     if (P.is_clockwise_oriented()) {
//         reverse(in_p.begin(), in_p.end());
//     } 
// }

template <typename T>//counterclockwise
double angle(T p1, T p2, T p3, T p4) {
    double x1, y1, x2, y2;
    x1 = p2.x - p1.x;
    y1 = p2.y - p1.y;
    x2 = p4.x - p3.x;
    y2 = p4.y - p3.y;
    double a1 = atan2(y1, x1);
    double a2 = atan2(y2, x2);
    double a = a2 - a1;
    if (a < 0) {
        a += 2*M_PI;
    }
    return a;
}

template <typename T>
void rotate(const T& in_pt, const double rad, T* out_pt){
    double sinth = std::sin(rad);
    double costh = std::cos(rad);
    out_pt->y = sinth * in_pt.x + costh * in_pt.y;   
    out_pt->x = costh * in_pt.x - sinth * in_pt.y;
}

// rotate polygon of rad along origin
template <typename T>
void rotate(const std::vector<T>& in_poly,
    const double rad, std::vector<T>* out_poly){
    for(int i = 0; i < in_poly.size(); ++i){
        T pt;
        rotate<T>(in_poly.at(i), rad, &pt);  
        out_poly->emplace_back(pt);
    }
}


// template <typename T>
// bool dubinsShot(
//     const DSPoint& start, const DSPoint& goal,
//     std::vector<DSPoint>& rs_path,
//     double radius, double resolution){                
//     rs_path.clear();
//     namespace ob = ompl::base;
//     namespace og = ompl::geometric;
//     ob::StateSpacePtr space(
//         std::make_shared<ob::DubinsStateSpace>(radius));
//     int num_pts = 20;
//     ob::ScopedState<> from(space), to(space), s(space);
//     from[0] = start.x; from[1] = start.y; from[2] = start.theta;
//     to[0] = goal.x; to[1] = goal.y; to[2] = goal.theta;
//     std::vector<double> reals;
//     double len = space->distance(from(), to()); 
//     num_pts = len / resolution; 
//     for (unsigned int i = 0; i <= num_pts; ++i){
//         space->interpolate(from(), to(), (double)i/num_pts, s());
//         reals = s.reals();
//         rs_path.push_back(DSPoint(reals[0], reals[1] , reals[2]));
//     }
//     if(start.dis(goal) * 4 < len) {
//         return false;
//     } else {
//         return true;
//     }
// }

// template <typename T>//counterclockwise
// void rotate(const T& in_pt, const double rad, T* out_pt) {
//     out_pt->x = std::cos(rad)*in_pt.x - std::sin(rad)*in_pt.y;
//     out_pt->y = std::sin(rad)*in_pt.x + std::cos(rad)*in_pt.y;
// }


// template <typename T>
// void rotate(const std::vector<T>& in_poly,
//             const double rad, std::vector<T>* out_poly) {
//     for(int i = 0; i < in_poly.size(); ++i) {
//         T pt;
//         rotate<T>(in_poly.at(i), rad, &pt);
//         out_poly->push_back(pt);
//     }
// }


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


// template <typename T>
// void getBoundary(const std::vector<T>& poly, double* minx,
//                 double* miny, double* maxx, double* maxy) {
//     *minx = std::numeric_limits<double>::max();
//     *miny = std::numeric_limits<double>::max();
//     *maxx = std::numeric_limits<double>::min();
//     *maxy = std::numeric_limits<double>::min();
//     for(const T&p: poly){
//         *minx = *minx < p.x ? *minx : p.x;
//         *miny = *miny < p.y ? *miny : p.y;
//         *maxx = *maxx > p.x ? *maxx : p.x;
//         *maxy = *maxy > p.y ? *maxy : p.y;
//     }
// }


template <typename T>
void pointUnique(std::vector<T> &pts) {
    for(auto it=pts.begin() + 1; it != pts.end() + 1; ++it) {
        if(it->x == (it-1)->x && it->y == (it-1)->y){
            pts.erase(it);
            it = it-1;
        }
    }
}


// interpolate points between a and b, using the map's resolution
template <typename T>
void interpolate(const T&a, const T&b, std::vector<T>* pts, double length){
    pts->clear();
    
    double s = sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
    int n = s / length;
    pts->push_back(a);
    if(n < 3){
        pts->push_back(b);
        return;
    }
    for(int i = 1; i < n; i++){
        double scale = i * 1.0 / n;
        pts->push_back(Point(scale*(b.x - a.x) + a.x, scale*(b.y - a.y) + a.y));
    }
    pts->push_back(b);
}


template <typename T>
void pointUnique(std::vector<std::vector<T>> &polys) {
    for(auto &poly: polys){
        pointUnique<T>(poly);
    }
}


template <typename T>
void handleSelfIntersection(std::vector<std::vector<T>> &polys) {
    for(auto &poly: polys) {
        pointUnique<T>(poly);
    }
}

template <typename T>
void draw(const cv::Mat &image, const std::vector<T> &pts, cv::Vec3b color) {
    for (int i = 0; i < pts.size(); ++i) {
        int n_idx = (i + 1) % pts.size();
        cv::line(image, 
            cv::Point2d(pts[i].x, pts[i].y), 
            cv::Point2d(pts[n_idx].x, pts[n_idx].y), color);
    }
    
}
}
}
#endif