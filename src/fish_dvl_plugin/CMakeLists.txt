cmake_minimum_required(VERSION 3.8)
project(fish_dvl_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo REQUIRED)  # <-- Add this line

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  /usr/include/gazebo-11
)


add_library(${PROJECT_NAME} SHARED src/dvl_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs gazebo_ros gazebo_plugins)

# Link against Gazebo libraries
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
