#ifndef CAMERA_DISTRIBUTE_HPP
#define CAMERA_DISTRIBUTE_HPP

#include "tinyxml2.h"
#include <string>

class AreaCoordinate {
    struct Coordinate {
    int x; // X坐标
    int y; // Y坐标
};
    public:
    Coordinate lleft_up; // 左上角坐标
    Coordinate lleft_down; // 左下角坐标
    Coordinate lright_down; // 右下角坐标
    Coordinate lright_up; // 右上角坐标

    Coordinate rleft_up; // 左上角坐标
    Coordinate rleft_down; // 左下角坐标
    Coordinate rright_down; // 右下角坐标
    Coordinate rright_up; // 右上角坐标

    AreaCoordinate() {
        efence_read(*this);
    }
    friend int efence_read(AreaCoordinate &area);
};

int camera_distribute();
int camera_index_read();

extern int camera_up_index;
extern int camera_jaw_index;

extern AreaCoordinate area3;
extern double car_length;


// /home/wtr2023/ros_ws/rc2024/RC24_R2V2/src/action_findball/include/action_findball/camera_distribute.hpp
#endif // CAMERA_DISTRIBUTE_HPP