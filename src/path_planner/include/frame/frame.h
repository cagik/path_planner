#pragma once
#include <string>
#include <vector>
#include <math.h>  
#include <ctime>
#include <fstream>
#include <memory>

#include <ros/ros.h>
#include <ros/package.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include "plannerBase.h"
#include "planner/astar.h"
#include "planner/rrt.h"
#include "planner/rrtStar.h"
#include "planner/dubinsPlanner.h"

#include "common/dataStruct.hpp"
#include "common/constant.h"

#include "tool/KDTree/KDTree.hpp"

#include "ros_viz_tools/ros_viz_tools.h"

using namespace std;


class frame
{
public:
    frame();

    bool planReady();
    bool planReady_noWayPoints();
    bool Plan();
    void marker_Vis();
    void LoopAction();

    unique_ptr<ros::NodeHandle> nh_ptr_;

private:
    
    void wayPointCb(const geometry_msgs::PointStampedConstPtr &p);
    void startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start);
    void goalCb(const geometry_msgs::PoseStampedConstPtr &goal);

    void initRosFrame();
    void initFramePara();
    void initGridMap();

    unique_ptr<ros_viz_tools::RosVizTools> marker_ptr_;
    string marker_frame_id_;

    State3D start_;
    State3D end_;
    vector<State3D> wayPoints_;
    bool startStateFlag_;
    bool endStateFlag_;
    bool wayPointsFlag_;

    vector<vector<int>> grid_map_;
    double map_resolution_;
    pointVec obstaclePoints_;
    KDTree obstacleTree_;

    
    grid_map::GridMap grid_map_for_rviz_;
    ros::Subscriber wayPoint_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber end_sub_;
    ros::Publisher map_publisher_;


    unique_ptr<plannerBase> planner_ptr_;

    vector<State3D> frontEnd_Path_;

};

