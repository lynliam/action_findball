#include "pid_controller.hpp"


    /*增量式PID算法*/
float action_findball::PIDController::PID_Calc(float cur_error_)
{
    cur_error = cur_error_;
    output += KP * (cur_error - error[1]) + KI * cur_error + KD * (cur_error - 2 * error[1] + error[0]);
    error[0] = error[1];
    error[1] = cur_error;

    /*设定输出上限*/
    if (output > outputMax)
        output = outputMax;
    if (output < outputMin)
        output = outputMin;
    return output;
}

action_findball::PIDController::PIDController(float kp, float ki, float kd)
{
    KP = kp;
    KI = ki;
    KD = kd;
    cur_error = 0;
    error[0] = 0;
    error[1] = 0;
    integral = 0;
    integralMax = 10.3;
    integralMin = -10.3;
    output = 0;
    outputMax = 1.4;
    outputMin = -1.4;
}
void action_findball::PIDController::PID_setParam(float kp, float ki, float kd)
{
    KP = kp;
    KI = ki;
    KD = kd;
}
void action_findball::PIDController::PID_MaxMin(float max, float min)
{
    outputMax = max;
    outputMin = min;
}

float action_findball::PIDController::PosePID_Calc(float cur_error_)
{
     integral +=  cur_error_;

    /*防止积分饱和*/
    if ( integral >  integralMax)
         integral =  integralMax;
    if ( integral <  integralMin)
         integral =  integralMin;

     output =  KP *  cur_error_ +  KI *  integral +  KD * (error[1] - error[0]);
     error[0] =  error[1];
     error[1] =  cur_error_;

    /*设定输出上限*/
    if ( output >  outputMax)
         output =  outputMax;
    if ( output <  outputMin)
         output =  outputMin;
    return output;
}


int action_findball::PIDController::acquire_PID_variable(std::string variable_name)
{
    std::filesystem::path currentPath = std::filesystem::current_path();
    const std::string package_name = "action_findball";
    std::filesystem::path dirPath = currentPath / "install" / package_name / "share" / package_name / "xml" / "variable.xml";// 假设已经构造好基础路径
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(dirPath.c_str()) != tinyxml2::XML_SUCCESS)
    {
        std::cerr <<"Failed to load file: "<< dirPath.c_str() << std::endl;
        return -1;
    }
    
    tinyxml2::XMLElement* pidElement = doc.FirstChildElement("pid");
    if (pidElement == nullptr) {
        std::cerr << "Failed to find 'pid' element." << std::endl;
        return -1;
    }

    for (tinyxml2::XMLElement* axisElement = pidElement->FirstChildElement(); axisElement; axisElement = axisElement->NextSiblingElement()) {
        const char* axisName = axisElement->Name();

        tinyxml2::XMLElement* kpElement = axisElement->FirstChildElement("kp");
        tinyxml2::XMLElement* kiElement = axisElement->FirstChildElement("ki");
        tinyxml2::XMLElement* kdElement = axisElement->FirstChildElement("kd");

        if (kpElement && kiElement && kdElement) {
            if(axisName == variable_name)
            {
                PID_setParam(kpElement->FloatText(), kiElement->FloatText(), kdElement->FloatText());
            }
        }
    }
    return 0;
}