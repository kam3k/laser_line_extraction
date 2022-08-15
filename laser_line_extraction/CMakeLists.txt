cmake_minimum_required(VERSION 3.8)
project(laser_line_extraction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(laser_line_extraction_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

set(ament_dependencies
   rclcpp 
   std_msgs 
   sensor_msgs
   visualization_msgs
   laser_line_extraction_interfaces
)




# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include
${EIGEN3_INCLUDE_DIRS})

add_library(line src/line.cpp)
ament_target_dependencies(line ${ament_dependencies})

add_library(line_extraction src/line_extraction.cpp)
ament_target_dependencies(line_extraction ${ament_dependencies})

add_library(line_extraction_ros src/line_extraction_ros.cpp)
ament_target_dependencies(line_extraction_ros ${ament_dependencies})
target_link_libraries(line_extraction_ros line line_extraction)

add_executable(line_extraction_node src/line_extraction_node.cpp)
ament_target_dependencies(line_extraction_node ${ament_dependencies})

target_link_libraries(line_extraction_node line line_extraction line_extraction_ros)

install(TARGETS
line_extraction_node
DESTINATION lib/${PROJECT_NAME})

ament_package()
