#ifndef ORB_UTILS_COMMON
#define ORB_UTILS_COMMON

#include <nlohmann/json.hpp>

template <typename T>
void readParameter(std::string key,  T &value) {
    nlohmann::json json;
    std::ifstream in("/home/wz/VO-LOAM/github/orb-slam3/conf/conf.json");
    in >> json;
    value = json[key];
    in.close();
    std::cout << key << " : " << value << "\n";
}

template <typename T>
T readParameter(std::string key) {
    T value;
    nlohmann::json json;
    std::ifstream in("/home/wz/VO-LOAM/github/orb-slam3/conf/conf.json");
    in >> json;
    value = json[key];
    in.close();
    std::cout << key << " : " << value << "\n";
    return value;
}

#endif