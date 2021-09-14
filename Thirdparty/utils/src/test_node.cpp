#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
// #include <ctime>
#include <thread>
#include <mutex>
#include <chrono>
#include <fstream> 
#include <sys/stat.h>
// #include <direct.h>
// #include <X11/Xlib.h>


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "sfm.h"
#include "matcher.h"
#include "stl_utils.h"
#include "coverage_utils.h"

#include "optimizer/optimizer.h"

using namespace tergeo::visualodometry;
int main() {
    Optimizer op;
    op.getOptimizeObject(OPTIMIZE_TYPE::FULL_CAMERA);
}