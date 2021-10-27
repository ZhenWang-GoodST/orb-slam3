#ifndef ORB_UTILS_COMMON_HPP
#define ORB_UTILS_COMMON_HPP

#include <nlohmann/json.hpp>

template <typename T>
void readParameter(std::string key,  T &value, const std::string &file) {
    nlohmann::json json;
    std::ifstream in(file);
    in >> json;
    value = json[key];
    in.close();
    std::cout << key << " : " << value << "\n";
}

template <typename T>
T readParameter(std::string key, const std::string &file) {
    T value;
    nlohmann::json json;
    std::ifstream in(file);
    in >> json;
    value = json[key];
    in.close();
    std::cout << key << " : " << value << "\n";
    return value;
}

#endif