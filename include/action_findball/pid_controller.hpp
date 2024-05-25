#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP
#include <string>
#include <filesystem>
#include <iostream>
#include <string>

#include "tinyxml2.h"

namespace action_findball {

class PIDController
{
    public:
        PIDController(float kp, float ki, float kd);
        float PID_Calc(float cur_error_);
        float PosePID_Calc(float cur_error_);
        void PID_MaxMin(float max, float min);
        void PID_setParam(float kp, float ki, float kd);
        int acquire_PID_variable(std::string variable_name);
        float integralMax;  // 积分上限
        float integralMin;  // 积分下限 用于积分饱和
    private:
        float KP;        // PID参数P
        float KI;        // PID参数I
        float KD;        // PID参数D
        float cur_error; // 当前误差
        float error[2];  // 前两次误差
        float integral;  // 积分
        float output;    // 输出值
        float outputMax; // 最大输出值的绝对值
        float outputMin; // 最小输出值的绝对值用于防抖
};

} // namespace action_findball

#endif // PID_CONTROLLER_HPP