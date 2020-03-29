cmake_minimum_required(VERSION 2.8.3)
project(laser_line_extraction)

find_package(catkin QUIET COMPONENTS cmake_modules geometry_msgs 
    message_generation roscpp rospy sensor_msgs visualization_msgs)

find_package(Eigen3 REQUIRED)

if (catkin_FOUND)
    message("-- Catkin found. Compiling ROS components.")
    add_message_files(
      FILES
      LineSegment.msg
      LineSegmentList.msg
    )

    generate_messages(
      DEPENDENCIES
      sensor_msgs
    )

    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES line line_extraction line_extraction_ros
      CATKIN_DEPENDS geometry_msgs message_runtime roscpp sensor_msgs visualization_msgs
    )
else()
    message("-- Catkin NOT found. Ommitting ROS components.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(line src/line.cpp)
target_link_libraries(line ${catkin_LIBRARIES})

add_library(line_extraction src/line_extraction.cpp)
target_link_libraries(line_extraction ${catkin_LIBRARIES})

if (catkin_FOUND)
    include_directories(include ${catkin_INCLUDE_DIRS})

    add_library(line_extraction_ros src/line_extraction_ros.cpp)
    target_link_libraries(line_extraction_ros line line_extraction ${catkin_LIBRARIES})
    add_dependencies(line_extraction_ros laser_line_extraction_generate_messages_cpp)

    add_executable(line_extraction_node src/line_extraction_node.cpp)
    target_link_libraries(line_extraction_node line_extraction_ros ${catkin_LIBRARIES})
    
    #catkin_add_gtest(${PROJECT_NAME}-test test/test_laser_line_extraction.cpp)
    install(TARGETS line_extraction_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    install(TARGETS line_extraction_ros line_extraction line ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
endif()
