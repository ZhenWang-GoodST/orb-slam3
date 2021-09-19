#ifndef TERGEO_VISUAL_ODOMETRY_STL_UTILS
#define TERGEO_VISUAL_ODOMETRY_STL_UTILS

#include "types.hpp"
#include "dirent.h"


int createFolders(const char* dir){
  char order[100] = "mkdir -p ";
  strcat(order, dir); 
  return system(order);
} 

std::vector<std::string> listdir(const std::string &path) {
    DIR *dp;
    struct dirent *dirp;
    std::vector<std::string> filename; //std::string不表示继承 而是c++11标准下的类型

    if ((dp = opendir(path.c_str())) == nullptr) //nullptr 表示空指针   c_str返回一个可读不可改的指针
        perror("open dir error");                // 显示打开文件错误  perror(string)将显示(string:No such..)冒号以及后边的内容是函数内置的

    while ((dirp = readdir(dp)) != nullptr)
    {
        if ("." == std::string(dirp->d_name) || std::string(dirp->d_name) == "..") //如果文件的名字为.或者.. 则继续
            continue;
        filename.push_back(dirp->d_name);
        // filename.emplace_back(dirp->d_name);                              //emplace_back基本可以代替push_back
    }

    closedir(dp);
    std::sort(filename.begin(), filename.end());
    return filename;
}
#endif