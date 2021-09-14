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
#include <sys/stat.h>
#include <dirent.h>


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


ros::Subscriber imu_subscriber, image_subscriber1, odometey_subscriber1;
cv::Mat image, keypoints_image;
ros::Time time_now, header_time, last_time;
nav_msgs::Odometry odometry, last_odometry, current_odometry;
double distance, angle;

void getFiles(const char* path, std::vector<std::string>& files){

    const std::string path0 = path;
    DIR* pDir;
    struct dirent* ptr;

    struct stat s;
    lstat(path, &s);

    if(!S_ISDIR(s.st_mode)){
        cout << "not a valid directory: " << path << endl;
        return;
    }

    if(!(pDir = opendir(path))){
        cout << "opendir error: " << path << endl;
        return;
    }
    int i = 0;
    std::string subFile;
    while((ptr = readdir(pDir)) != 0){
        subFile = ptr -> d_name;
        if(subFile == "." || subFile == "..")
            continue;
        // subFile = path0 + "/" + subFile;
        cout << ++i << ": " << subFile << endl;
        files.push_back(subFile);
    }
    closedir(pDir);

}



int main(int argc, char** argv) {
    // XInitThreads();
    ros::init(argc, argv, "image_receiver_node");
    std::cout << "---------------------------\n";
    std::cout << "read image in directory\n";
    std::cout << "---------------------------\n";
    ros::NodeHandle nh;

    std::string folder = "/home/wz/VO-LOAM/data/visual_odometry/good_image/images";
    std::vector<std::string> files;
    getFiles(folder.c_str(), files);

    for_each(files.begin(), files.end(), [](const std::string &s){cout << s << endl; });
    for (size_t i = 0; i < files.size(); i++) {
        
    }
    

    cout << endl;
   
    std::string tt = "";
    while (ros::ok()) {
        ros::spin();
    }
    // processing = false;
    

}