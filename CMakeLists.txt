cmake_minimum_required(VERSION 3.8)
project(action_findball)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


# colcon build --packages-select action_findball --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(OpenCV 4.9.0 REQUIRED)
find_package(rc2024_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(Include_DIR "src/ximgproc/include/opencv2" "src/ximgproc/include" "src/ximgproc/src" "include/action_findball" "include")
set(SRC_DIR "src/ximgproc/src/*.cpp")
include_directories(${OpenCV_INCLUDE_DIRS} ${Include_DIR})
file(GLOB_RECURSE SRC_FILES ${SRC_DIR})
link_directories(${OpenCV_LIB_DIR})

add_executable(publisher_findball src/publisher_findball.cpp ${SRC_FILES} "src/FindBall.cpp")
target_link_libraries(publisher_findball ${OpenCV_LIBS})
target_include_directories(publisher_findball PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(publisher_findball
    "rclcpp"
    "rclcpp_lifecycle"
    "lifecycle_msgs"
    "rc2024_interfaces")

add_executable(action_catchball src/action_catch.cpp)
ament_target_dependencies(action_catchball
    "rclcpp"
    "rclcpp_action"
    "lifecycle_msgs"
    "rclcpp_lifecycle"
    "rc2024_interfaces")

install(TARGETS
  publisher_findball
  action_catchball
  DESTINATION lib/${PROJECT_NAME})

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
