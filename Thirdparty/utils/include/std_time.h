#ifndef TDDIAO_ROS_TIME
#define TDDIAO_ROS_TIME
#include "iostream"
// #include "types.hpp"
#ifdef __ROS_TIME__
#include <ros/ros.h>
class StdTime
{
private:
    ros::Time constant_start;
    ros::Time dynamic_start;
    ros::Time output_start;
public:
    StdTime() {
        constant_start = ros::Time::now();
        dynamic_start = ros::Time::now();
    }
    ~StdTime() {}
    void reset() {
        dynamic_start = ros::Time::now();
    }
    // reference
    StdTime operator[](int n) {
        switch (n) {
            case 0:
                output_start = constant_start;
                break;
            case 1:
                output_start = dynamic_start;
                this->reset();
                break;
            default:
                output_start = constant_start;
                break;
        }
        return *this;
    }
    friend std::ostream& operator<<(std::ostream &os, RosTime rostime);
    // friend std::iotream& operator<<(std::oftream &of, RosTime rostime);
};

std::ostream& operator<<(std::ostream &os, RosTime rostime) {
    os << ros::Time::now().toSec() - rostime.output_start.toSec() << "\n";
    return os;
}
// std::ostream& operator<<(std::ostream &of, RosTime rostime) {
//     of << ros::Time::now().toSec() - rostime.output_start.toSec() << "\n";
//     return of;
// }
#else
#include <ctime>
class StdTime
{
private:
    clock_t constant_start;
    clock_t dynamic_start;
    clock_t output_start;
public:
    StdTime() {
        constant_start = clock();
        dynamic_start = clock();
    }
    ~StdTime() {}
    void reset() {
        dynamic_start = clock();
    }
    // reference
    StdTime operator[](int n) {
        switch (n) {
            case 0:
                output_start = constant_start;
                break;
            case 1:
                output_start = dynamic_start;
                this->reset();
                break;
            default:
                output_start = constant_start;
                break;
        }
        return *this;
    }
    friend std::ostream& operator<<(std::ostream &os, StdTime stdtime);
    // friend std::iotream& operator<<(std::oftream &of, RosTime rostime);
};

// std::ostream& operator<<(std::ostream &os, StdTime stdtime) {
//     os << (clock() - stdtime.output_start) * 1.0 / CLOCKS_PER_SEC << "\n";
//     return os;
// }
// std::ostream& operator<<(std::ostream &of, RosTime rostime) {
//     of << ros::Time::now().toSec() - rostime.output_start.toSec() << "\n";
//     return of;
// }
#endif

// typedef __gnu_cxx::__alloc_traits<_Tp_alloc_type>	_Alloc_traits;
// typedef typename _Alloc_traits::reference		reference;
#endif