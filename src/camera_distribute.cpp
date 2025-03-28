#include "camera_distribute.hpp"
#include <filesystem>   // std::filesystem::current_path
#include <iostream>
#include <cstdlib>      // std::getenv
#include <sstream>
#include <string>   
#include <regex>        // std::regex

std::regex pattern("\\d+$");
int camera_up_index = 0;
int camera_jaw_index = 0;

AreaCoordinate area3;
double car_length;

int camera_index_read()
{
    // std::filesystem::path currentPath = std::filesystem::current_path();    
    // //const std::string package_name = "action_findball";
    // std::filesystem::path dirPath = currentPath / "xml" / "camera.xml";
    std::filesystem::path currentPath = std::filesystem::current_path();
    const std::string package_name = "action_findball";
    std::filesystem::path dirPath = currentPath / "install" / package_name / "share" / package_name / "xml" / "camera.xml";// 假设已经构造好基础路径

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(dirPath.c_str()) != tinyxml2::XML_SUCCESS)
    {
        std::cerr << "Failed to load XML file." << std::endl;
        return -1;
    }
    
    tinyxml2::XMLElement* cameraElement = doc.FirstChildElement("camera");
    if (cameraElement == nullptr) {
        std::cerr << "Failed to find 'camera' element." << std::endl;
        return -1;
    }

    for (tinyxml2::XMLElement* axisElement = cameraElement->FirstChildElement(); axisElement; axisElement = axisElement->NextSiblingElement()) {
        const char* axisName = axisElement->Name();
        tinyxml2::XMLElement* nameElement = axisElement->FirstChildElement("name");
        tinyxml2::XMLElement* index1Element = axisElement->FirstChildElement("index1");
        tinyxml2::XMLElement* index2Element = axisElement->FirstChildElement("index2");
        if (nameElement && index1Element && index2Element) {
            if(axisName == std::string("up"))
            {
                std::cout << nameElement->GetText() << " " << index1Element->GetText() << " " << index2Element->GetText() << std::endl;
                std::smatch match;
                std::string  inputStr(index1Element->GetText());

                if (std::regex_search(inputStr, match, pattern))
                    camera_up_index = std::stoi(match.str());
                else
                    std::cerr << "未找到匹配项" << std::endl;

            }else if(axisName == std::string("jaw"))
            {
                std::cout << nameElement->GetText() << " " << index1Element->GetText() << " " << index2Element->GetText() << std::endl;
                std::smatch match;
                std::string  inputStr(index1Element->GetText());

                if (std::regex_search(inputStr, match, pattern))
                    camera_jaw_index = std::stoi(match.str());
                else
                    std::cerr << "未找到匹配项" << std::endl;
            }
        }
    }
    return 0;
}

int camera_distribute()
{
    // std::filesystem::path currentPath = std::filesystem::current_path();    
    
    // std::filesystem::path dirPath = currentPath / "xml" / "camera.xml";
    std::filesystem::path currentPath = std::filesystem::current_path();
    const std::string package_name = "action_findball";
    std::filesystem::path dirPath = currentPath / "install" / package_name / "share" / package_name / "xml" / "camera.xml";// 假设已经构造好基础路径
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(dirPath.c_str()) != tinyxml2::XML_SUCCESS)
    {
        std::cerr << "Failed to load XML file." << std::endl;
        return -1;
    }
    tinyxml2::XMLElement* cameraElement = doc.FirstChildElement("camera");
    if (cameraElement == nullptr) {
        std::cerr << "Failed to find 'camera' element." << std::endl;
        return -1;
    }

    // 执行命令并打开输出流
    FILE* pipe = popen("v4l2-ctl --list-devices", "r");
    if (!pipe) {
        std::cerr << "Error: Failed to open pipe for command execution." << std::endl;
        return 1;
    }
    // 读取命令输出
    std::stringstream ss;
    char buffer[128];
    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != nullptr) {
            std::cout << buffer;
            ss << buffer;
        }
    }
    // 关闭管道
    pclose(pipe);
    std::string input = ss.str();

    // 定义正则表达式模式
    std::regex pattern(R"((?:[^:]+): ([^\n]+):\s*\n\s*(\/dev\/[^\n]+)\s*\n\s*(\/dev\/[^\n]+)\s*\n\s*(\/dev\/[^\n]+))");
    std::regex pattern2(R"((\d+\.\d+-\d+))");

    // 使用迭代器遍历匹配结果
    std::sregex_iterator iter(input.begin(), input.end(), pattern);
    std::sregex_iterator end;

    for (; iter != end; ++iter) {
        std::smatch match = *iter;
        std::string cameraName = match[1];
        std::string videoDevice1 = match[2];
        std::string videoDevice2 = match[3];
        std::string mediaDevice = match[4];

        std::cout << "Camera Name: " << cameraName << std::endl;
        std::cout << "Video Device 1: " << videoDevice1 << std::endl;
        std::cout << "Video Device 2: " << videoDevice2 << std::endl;

        //std::string prefix = cameraName.substr(0, cameraName.find(" "));
        std::smatch match_ID;
        if (std::regex_search(cameraName, match_ID, pattern2)) {
            std::cout << "找到的设备ID是: " << match_ID[1] << std::endl;
        } else {
            std::cout << "未找到匹配的设备ID。" << std::endl;
        }
        // 分别修改up和down中的index值
        // 修改up中的index
        if(match_ID[1] == "14.0-1")
        {
            tinyxml2::XMLElement* upElement = cameraElement->FirstChildElement("up");
            if (upElement) {
                tinyxml2::XMLElement* nameElement = upElement->FirstChildElement("name");
                tinyxml2::XMLElement* index1Element = upElement->FirstChildElement("index1");
                tinyxml2::XMLElement* index2Element = upElement->FirstChildElement("index2");
                if (nameElement) {
                    nameElement->SetText(cameraName.c_str());
                }
                if (index1Element) {
                    index1Element->SetText(videoDevice1.c_str());
                }
                if (index2Element) {
                    index2Element->SetText(videoDevice2.c_str());
                }
            }
        }else {
            // 修改jaw中的index
            tinyxml2::XMLElement* jawElement = cameraElement->FirstChildElement("jaw");
            if (jawElement) {
                tinyxml2::XMLElement* nameElement = jawElement->FirstChildElement("name");
                tinyxml2::XMLElement* index1Element = jawElement->FirstChildElement("index1");
                tinyxml2::XMLElement* index2Element = jawElement->FirstChildElement("index2");
                if (nameElement) {
                    nameElement->SetText(cameraName.c_str());
                }
                if (index1Element) {
                    index1Element->SetText(videoDevice1.c_str());
                }
                if (index2Element) {
                    index2Element->SetText(videoDevice2.c_str());
                }
            }
        }
    }
    
    // 保存修改后的XML到文件
    if (doc.SaveFile(dirPath.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to save XML file." << std::endl;
        return -1;
    }

    std::cout << "XML file updated successfully." << std::endl;

    return 0;
}


int efence_read(AreaCoordinate &area)
{
    std::filesystem::path currentPath = std::filesystem::current_path();
    const std::string package_name = "action_findball";
    std::filesystem::path dirPath = currentPath / "install" / package_name / "share" / package_name / "xml" / "electronic_fence.xml";// 假设已经构造好基础路径

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(dirPath.c_str()) != tinyxml2::XML_SUCCESS)
    {
        std::cerr << "Failed to load XML file." << std::endl;
        return -1;
    }
    
    tinyxml2::XMLElement* cameraElement = doc.FirstChildElement("root");
    if (cameraElement == nullptr) {
        std::cerr << "Failed to find 'root' element." << std::endl;
        return -1;
    }
    for (tinyxml2::XMLElement* axisElement_ = cameraElement->FirstChildElement(); axisElement_; axisElement_ = axisElement_->NextSiblingElement())
    {
        if(axisElement_->Name() == std::string("car"))
        {
            car_length = std::stod(axisElement_->Attribute("b"));
        }
        if(axisElement_->Name() == std::string("area3_left"))
        {
            std::cout << "area3_left" << std::endl;
        for (tinyxml2::XMLElement* axisElement = axisElement_->FirstChildElement(); axisElement; axisElement = axisElement->NextSiblingElement()) {
            const char* axisName = axisElement->Name();
            tinyxml2::XMLElement* index1Element = axisElement->FirstChildElement("x");
            tinyxml2::XMLElement* index2Element = axisElement->FirstChildElement("y");
            if (index1Element && index2Element) {
                if(axisName == std::string("left_up"))
                {
                    area.lleft_up.x = std::stoi(index1Element->GetText());
                    area.lleft_up.y = std::stoi(index2Element->GetText());
                    std::cout << index1Element->GetText() << " " << index2Element->GetText() << std::endl;
                }else if(axisName == std::string("left_down"))
                {
                    area.lleft_down.x = std::stoi(index1Element->GetText());
                    area.lleft_down.y = std::stoi(index2Element->GetText());
                    std::cout << index1Element->GetText() << " " << index2Element->GetText() << std::endl;
                }else if(axisName == std::string("right_down"))
                {
                    area.lright_down.x = std::stoi(index1Element->GetText());
                    area.lright_down.y = std::stoi(index2Element->GetText());
                    std::cout << index1Element->GetText() << " " << index2Element->GetText() << std::endl;
                }else if(axisName == std::string("right_up"))
                {
                    area.lright_up.x = std::stoi(index1Element->GetText());
                    area.lright_up.y = std::stoi(index2Element->GetText());
                    std::cout << index1Element->GetText() << " " << index2Element->GetText() << std::endl;
                }
            }
        }
        }
        
        if(axisElement_->Name() == std::string("area3_right"))
        {
            std::cout << "area3_right" << std::endl;
        for (tinyxml2::XMLElement* axisElement = axisElement_->FirstChildElement(); axisElement; axisElement = axisElement->NextSiblingElement()) {
            const char* axisName = axisElement->Name();
            tinyxml2::XMLElement* index1Element = axisElement->FirstChildElement("x");
            tinyxml2::XMLElement* index2Element = axisElement->FirstChildElement("y");
            if (index1Element && index2Element) {
                if(axisName == std::string("left_up"))
                {
                    area.rleft_up.x = std::stoi(index1Element->GetText());
                    area.rleft_up.y = std::stoi(index2Element->GetText());
                    std::cout << index1Element->GetText() << " " << index2Element->GetText() << std::endl;
                }else if(axisName == std::string("left_down"))
                {
                    area.rleft_down.x = std::stoi(index1Element->GetText());
                    area.rleft_down.y = std::stoi(index2Element->GetText());
                    std::cout << index1Element->GetText() << " " << index2Element->GetText() << std::endl;
                }else if(axisName == std::string("right_down"))
                {
                    area.rright_down.x = std::stoi(index1Element->GetText());
                    area.rright_down.y = std::stoi(index2Element->GetText());
                    std::cout << index1Element->GetText() << " " << index2Element->GetText() << std::endl;
                }else if(axisName == std::string("right_up"))
                {
                    area.rright_up.x = std::stoi(index1Element->GetText());
                    area.rright_up.y = std::stoi(index2Element->GetText());
                    std::cout << index1Element->GetText() << " " << index2Element->GetText() << std::endl;
                }
            }
        }
        }
    }
    return 0;
}