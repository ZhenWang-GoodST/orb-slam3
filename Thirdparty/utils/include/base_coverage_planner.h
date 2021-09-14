#ifndef _BASE_COVERAGE_PLANNER_H
#define _BASE_COVERAGE_PLANNER_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

namespace tergeo{
namespace visualodometry {

struct Point{
    public:
    Point():x(0.0), y(0.0){}
    Point(double x_, double y_):x(x_),y(y_){}
    double x = 0, y = 0;
    double dis (const Point& p) const {
        return std::hypot(x - p.x, y - p.y);
    }
    double dis2 (const Point& p) const {
        double dx = x - p.x;
        double dy = y - p.y;
        return dx * dx + dy * dy;
    }
    void print() {
        std::cout << "Point: (" << this->x << ", " << this->y << ")\n";
    }
    bool operator==(Point p) {
        if(p.x == x && p.y == y){
            return true;
        }else{
            return false;
        }

    }
};

  
}
}


#endif