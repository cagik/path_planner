cmake_minimum_required(VERSION 3.0.2)
project(path_planner)

add_definitions(-std=c++11 -w)
set(CMAKE_CXX_FLAGS " -Wall -Wextra ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "Release")

set(catkin_deps
        roscpp
        grid_map_ros
        ros_viz_tools
        )

find_package(catkin REQUIRED COMPONENTS
        ${catkin_deps}
        )

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS ${catkin_deps}
        DEPENDS OpenCV
)

include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)


file(GLOB_RECURSE SRC_DIR_LIST "src/*.cpp" "src/*.c")

add_executable(${PROJECT_NAME}_demo ${SRC_DIR_LIST})


target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
#target_link_libraries(${PROJECT_NAME}_demo /usr/local/lib/libosqp.so)
