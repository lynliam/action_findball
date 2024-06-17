cmake_minimum_required(VERSION 3.8)
project(action_findball)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++23

set(CMAKE_CXX_STANDARD 23)

# colcon build --packages-select action_findball --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers 12

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(OpenCV 4.9.0 REQUIRED)
find_package(rc2024_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(CURL REQUIRED)
include_directories(${CURL_INCLUDE_DIRS})

set(Include_DIR "src/ximgproc/include/opencv2" "src/ximgproc/include" "src/ximgproc/src" "include/action_findball" "include" "src/tinyxml2")
set(SRC_DIR "src/ximgproc/src/*.cpp")
include_directories(${OpenCV_INCLUDE_DIRS} ${Include_DIR})
file(GLOB_RECURSE SRC_FILES ${SRC_DIR})
link_directories(${OpenCV_LIB_DIR})

add_library(spin_to STATIC "src/spin_to_func.cpp")
ament_target_dependencies(spin_to
    "rclcpp"
    "rc2024_interfaces"
    "geometry_msgs"
    "sensor_msgs"
    "nav_msgs"
    "nav2_util"
    "tf2_ros")


add_executable(publisher_findball src/publisher_findball.cpp ${SRC_FILES} "src/FindBall.cpp" "src/camera_distribute.cpp" "src/tinyxml2/tinyxml2.cpp" "src/calibration.cpp")
target_link_libraries(publisher_findball ${OpenCV_LIBS} ${CURL_LIBRARIES})
target_include_directories(publisher_findball PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(publisher_findball
    "rclcpp"
    "rclcpp_lifecycle"
    "lifecycle_msgs"
    "rc2024_interfaces")

add_executable(catchball_server src/action_catch.cpp "src/tinyxml2/tinyxml2.cpp" "src/pid_controller.cpp")
target_link_libraries(catchball_server ${OpenCV_LIBS})
ament_target_dependencies(catchball_server
    "rclcpp"
    "rclcpp_action"
    "lifecycle_msgs"
    "rclcpp_lifecycle"
    "rc2024_interfaces"
    "geometry_msgs"
    "sensor_msgs"
    "nav_msgs")

add_executable(approaching_ball "src/approaching_ball.cpp" "src/tinyxml2/tinyxml2.cpp" "src/pid_controller.cpp" "src/lifecycle_manager_client.cpp" "src/PTZ_angle.cpp")
target_link_libraries(approaching_ball ${OpenCV_LIBS} spin_to)
ament_target_dependencies(approaching_ball
    "rclcpp"
    "rclcpp_action"
    "lifecycle_msgs"
    "rclcpp_lifecycle"
    "rc2024_interfaces"
    "geometry_msgs"
    "sensor_msgs"
    "nav_msgs")

install(TARGETS
  publisher_findball
  catchball_server
  approaching_ball
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  xml
  python
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
