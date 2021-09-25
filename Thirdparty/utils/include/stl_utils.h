#ifndef TERGEO_VISUAL_ODOMETRY_STL_UTILS
#define TERGEO_VISUAL_ODOMETRY_STL_UTILS

#include "types.hpp"
#include "dirent.h"


int createFolders(const char* dir);

std::vector<std::string> listdir(const std::string &path);
#endif