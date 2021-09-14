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


ros::Subscriber imu_subscriber, image_subscriber1, odometey_subscriber1;
cv::Mat image, keypoints_image;
ros::Time time_now, header_time, last_time;
nav_msgs::Odometry odometry, last_odometry, current_odometry;
double distance, angle;

//image process
bool new_image, isprocessing;
std::mutex image_process_mutex, odometry_mutex;
sensor_msgs::Imu imu;
int keyframe_count, loop_cout;

cv::Mat descriptors, last_descriptors;
//for match
cv::Mat hough_image = cv::Mat::zeros(100, 100, CV_8U);
cv::Mat K;
cv::Mat R, T;
cv::Mat last_image, current_image, show_image;
cv::Point3f cur_odometry_point;


//matcher
tergeo::visualodometry::Matcher matcher;
std::vector<cv::Mat> _images = {};
void initialize() {
    new_image = false;
    // descriptor_for_all.clear();
    // colors_for_all.clear();
    // key_points_for_all.clear();
    time_now = ros::Time::now();
    keyframe_count = 0;
    isprocessing = false;
    K = cv::Mat(cv::Matx33d(
		615.60882,    0.     ,  313.11862,
            0.     ,  615.54421,  255.36951,
            0.     ,    0.     ,    1.     ));
}
// void = 
void imageProcess() {
    while (true) {
        image_process_mutex.lock();
        std::cout << "begin loop\n";
        ros::Time tmp_clock = ros::Time::now();
        isprocessing = true;
        loop_cout += 1;
        if (current_image.empty() || loop_cout < 3 || loop_cout > 10000) {
            isprocessing = false;
            std::cout << "end loop\n";
            continue;
        }
        // cur_odometry_point.x = current_odometry.pose.pose.position.x;
        // cur_odometry_point.y = current_odometry.pose.pose.position.z;
        // cur_odometry_point.z = current_odometry.pose.pose.position.y;
        matcher.load(K, current_image, angle, distance, cur_odometry_point);
        int ismatched = matcher.featureMatch(R, T, show_image);
        tergeo::visualodometry::Match match;
        matcher._time = time_now;
        // std::cout << "templateMatch\n";
        // int ismatched = matcher.templateMatch(R, T, show_image, match);
        std::cout << "match time: " << ros::Time::now().toSec() - time_now.toSec() << "\n";
        if (ismatched == 0) {
            isprocessing = false;
            std::cout << "end loop\n";
            continue;
        }
        // std::cout << "test\n";
        matcher._image_cout += 1;
        //threeD reconstruction
        if (!matcher.isinitilized)  {
            // cur_odometry_point.x = 0;
            // cur_odometry_point.y = loop_cout;
            // cur_odometry_point.z = 0;
            // matcher.initializeCoordinate();
        } else {
            // matcher.solvePnP();
            // matcher.solveHomography();
        }
        tergeo::visualodometry::Result result(matcher._image_cout - 1, distance, match, last_time.toSec(), header_time.toSec());
        // cv::imshow("show_image", show_image);
        std::string directory_path = "/home/wz/VO-LOAM/data/visual_odometry/good_image/"+ std::to_string(loop_cout);
        // if (0 != access(directory_path.c_str(), 0)){
        //     mkdir(directory_path.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO); 
        // } else {
        //     std::cout << "end loop\n";
        //     isprocessing = false;
        //     continue;
        // }
        // auto it = matcher.matchPairs.end();
        // it--;
        // int left_id = it->second._left_image_id;
        // int right_id = it->second._right_image_id;
        // const cv::Mat &_last_image = matcher._keyframes[left_id]._image;
        // const cv::Mat &_current_image = matcher._keyframes[right_id]._image;
        // const cv::Mat &_match_image = matcher._show_image;
        // cv::imwrite(directory_path + "/left_image.png", _last_image);
        // cv::imwrite(directory_path + "/right_image.png", _current_image);
        // cv::imwrite(directory_path + "/match_image.png", show_image);
        // cv::imwrite(directory_path + "/match_image1.png", _match_image);
        std::ofstream outfile(directory_path + "/match.conf", std::ios::app); 
        // result.left_image_id = left_id;
        // result.right_image_id = right_id;
        outfile << result << "\n";
        outfile << "isInitilizePoint" << matcher._keyframes[matcher._left_image_id]._is_initilize_frame << "\n";
        outfile.close();
        std::cout << "total time: " << ros::Time::now().toSec() - time_now.toSec() << "\n";
        std::cout << std::setprecision(15) << time_now.toSec() << "\n";
        std::cout << "end loop\n\n";
        isprocessing = false;
    }
}

bool keyFrame() {
    if (ros::Time::now().toSec() - time_now.toSec() > 0.5) {
        // std::cout << isprocessing << " " << loop_cout << "\n";
        if (!isprocessing)
        time_now = ros::Time::now();
        // std::cout << "new frame\n";
        // std::cout << std::setprecision(15) << time_now.toSec() << "\n";
        return true;
    }
    return false;
}


void imuCall(const sensor_msgs::Imu::ConstPtr &msg) {
    imu = *msg;
    // std::cout << imu << "\n";
}

void odometrycall(const nav_msgs::Odometry::ConstPtr &msg) {
    odometry = *msg;
}


void calTransform(const nav_msgs::Odometry &od1, const nav_msgs::Odometry &od2) {
    double dx = od2.pose.pose.position.z - od1.pose.pose.position.z;
    double dy = od2.pose.pose.position.x - od1.pose.pose.position.x;
    double dz = od2.pose.pose.position.y - od1.pose.pose.position.y;
    distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    tf::Quaternion quat1, quat2;
    tf::quaternionMsgToTF(od1.pose.pose.orientation, quat1);
    tf::quaternionMsgToTF(od2.pose.pose.orientation, quat2);
    double r1, p1, r2, p2, y1, y2;
    tf::Matrix3x3(quat1).getRPY(r1, p1, y1);
    tf::Matrix3x3(quat2).getRPY(r2, p2, y2);
    angle = p2 - p1;
    // std::cout << (r2) / M_PI * 180 << "\n";
    // std::cout << (p2) / M_PI * 180 << "\n";
    // std::cout << (y2) / M_PI * 180 << "\n";
    // std::cout << angle << "\n";
}

void imageCall(const sensor_msgs::Image::ConstPtr &msg) {
    auto image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    image.release();
    image = image_ptr->image.clone();
    // if (image_cout < 4718 )
    // cv::imwrite("/home/wz/VO-LOAM/data/image/" + std::to_string(image_cout) + ".png", image);
    loop_cout += 1;

    // _images.push_back(image);
    // // std::cout << "one\n";
    // if(image_cout == 10) {
    //     std::cout << "done\n";
    //     save_image();
    // }
    if(!isprocessing && keyFrame()) {
        last_image = current_image.clone();
        current_image = image.clone();
        header_time = msg->header.stamp;
        last_odometry = current_odometry;
        current_odometry = odometry;
        // std::cout << header_time << "\n";
        // std::cout << ros::Time::now() << "\n";
        // last_odometry
        calTransform(last_odometry, current_odometry);
        image_process_mutex.unlock();
        // processing = true;
    }
    cv::imshow("image", image);
    cv::waitKey(1);
}
void callback(const sensor_msgs::Image::ConstPtr &msg1, const nav_msgs::Odometry::ConstPtr &msg2) {
    odometry = *msg2;
    auto image_ptr = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::BGR8);
    image.release();
    image = image_ptr->image.clone();
    // if (image_cout < 4718 )
    // cv::imwrite("/home/wz/VO-LOAM/data/image/" + std::to_string(image_cout) + ".png", image);
    loop_cout += 1;

    // _images.push_back(image);
    // // std::cout << "one\n";
    // if(image_cout == 10) {
    //     std::cout << "done\n";
    //     save_image();
    // }
    if(keyFrame() && !isprocessing) {
        last_image = current_image.clone();
        current_image = image.clone();
        double time1 = ros::Time::now().toSec() - header_time.toSec();
        last_time = header_time;
        // std::cout << "last : " << last_time << "\n";
        header_time = msg1->header.stamp;
        last_odometry = current_odometry;
        current_odometry = odometry;
        double time2 = current_odometry.header.stamp.toSec() - last_odometry.header.stamp.toSec();
        std::cout << "\n-----\ntime: " << time1 << " ---  "<< time1 - time2 << "\n"; 
        // std::cout << header_time << "\n";
        // last_odometry
        calTransform(last_odometry, current_odometry);
        // std::cout << "angle: " << angle / M_PI * 180 << "distance: " << distance << "\n";
        image_process_mutex.unlock();
        // processing = true;
    }
    matcher.structure_publisher.publish(matcher.ros_cloud);
    cv::imshow("image", image);
    cv::waitKey(1);
}


int main(int argc, char** argv) {
    // XInitThreads();
    ros::init(argc, argv, "image_receiver_node");
    std::cout << "---------------------------\n";
    std::cout << "image receiver node\n";
    std::cout << "---------------------------\n";
    ros::NodeHandle nh;
    initialize();
    _images.resize(4718);
    loop_cout = 0;
    message_filters::Subscriber<sensor_msgs::Image> 
        image_subscriber(nh, "/camera/color/image_raw", 10, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<nav_msgs::Odometry> 
        odometey_subscriber(nh, "/integrated_to_init", 10, ros::TransportHints().tcpNoDelay());
    matcher.structure_publisher = nh.advertise<sensor_msgs::PointCloud2>("structure_pointCloud", 10);
    // imu_subscriber = nh.subscribe("/imu/data_raw", 10, imuCall);
    // image_subscriber = nh.subscribe("/camera/color/image_raw", 10, imageCall);
    // odometey_subscriber = nh.subscribe("/integrated_to_init", 10, odometrycall);
    typedef message_filters::sync_policies::ApproximateTime
        <sensor_msgs::Image,nav_msgs::Odometry> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), image_subscriber, odometey_subscriber);
    // sync.registerCallback(boost::bind(&callback, _1, _2));
    std::thread image_process_thread(imageProcess);
    cv::Mat R, T, matched_image;
    tergeo::visualodometry::Match result;
    // matcher.templateMatch(R, T, matched_image, result);
    // cv::namedWindow("image", cv::CV_WINDOW_NORMAL);
    std::string tt = "";
    // for off-line debug
    std::string path = "/home/wz/VO-LOAM/data/image1.0test/";
    std::vector<std::string> images = listdir(path);
    std::vector<int> int_file = {};
    for (int i = 0; i < images.size(); ++i) {
        std::string name = images[i];
        int size = name.size();
        int_file.push_back(std::atoi(name.substr(0, size - 4).c_str()));
    }
    std::sort(int_file.begin(), int_file.end());
    tergeo::visualodometry::printVec<int>(int_file);
    for (int i = 0; i < 300; ++i) {
        image = cv::imread(path + std::to_string(int_file[i]) + ".png");
        std::cout << i << "\n";
        // loop_cout += 1;
        if(!isprocessing) {
            last_image = current_image.clone();
            current_image = image.clone();
            angle = 0;
            distance = 0.12;
            isprocessing = true;
            image_process_mutex.unlock();
            // processing = true;
        }
        matcher.structure_publisher.publish(matcher.ros_cloud);
        while (isprocessing) {
            /* code */
        }
        cv::imshow("image", image);
        cv::waitKey(1);
    }
     
    while (ros::ok()) {
        ros::spin();
    }
    // processing = false;
    

}