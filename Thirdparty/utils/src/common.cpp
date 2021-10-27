#include "common.h"//include common.h cause error 
#include<System.h>
std::string get_user() {
    char *HOME = get_current_dir_name();
    int id = 0;
    int count = 0;
    std::string USER = "";
    while (HOME[id] != '/n') {
        if (HOME[id++] == '/') count += 1;
        if (count == 3) break;
        if (count > 1) USER.push_back(HOME[id]);
    }
    return USER;
}