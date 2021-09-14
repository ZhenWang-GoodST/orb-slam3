#ifndef ORB_UTILS_COMMON
#define ORB_UTILS_COMMON

#include <nlohmann/json.hpp>

template <typename T>
void readParameter(std::string key,  T &value) {
    nlohmann::json json;
    std::ifstream in("/home/tonglu/VO-LOAM/github/orb-slam3/conf/conf.json");
    in >> json;
    value = json[key];
    in.close();
}

#endif