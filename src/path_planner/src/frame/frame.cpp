#include "frame/frame.h"

frame::frame()
{
    this->initRosFrame();
    this->initFramePara();
    this->initGridMap();
}

void frame::initRosFrame()
{
    this->nh_ptr_ = make_unique<ros::NodeHandle>("~");
    this->marker_ptr_ = make_unique<ros_viz_tools::RosVizTools>(*nh_ptr_, "markers");
    map_publisher_ = nh_ptr_->advertise<nav_msgs::OccupancyGrid>("grid_map", 1, true);
    wayPoint_sub_ = nh_ptr_->subscribe("/clicked_point", 1, &frame::wayPointCb, this);
    start_sub_ = nh_ptr_->subscribe("/initialpose", 1, &frame::startCb, this);
    end_sub_ = nh_ptr_->subscribe("/move_base_simple/goal", 1, &frame::goalCb,this);
    marker_frame_id_ = "/map";

}

void frame::initFramePara()
{
    this->map_resolution_ = constants::map_resolution;
    this->startStateFlag_ = false;
    this->endStateFlag_ = false;
    this->wayPointsFlag_ = false;
}

void frame::initGridMap()
{
    string image_dir = ros::package::getPath("path_planner");
    string image_file = "gridmap.png";
    image_dir.append("/" + image_file);
    cv::Mat img_src = cv::imread(image_dir, CV_8UC1);

    grid_map_for_rviz_ = grid_map::GridMap(std::vector<std::string>{"obstacle"});
    grid_map::Position p(0.5 * img_src.rows - 0.5, 0.5 * img_src.cols - 0.5);
    grid_map::GridMapCvConverter::initializeFromImage(img_src, map_resolution_, grid_map_for_rviz_, p);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(img_src, "obstacle", grid_map_for_rviz_, 0, 255, 1);
    grid_map_for_rviz_.setFrameId("/map");


    int width = img_src.cols;
    int height = img_src.rows;
    this->grid_map_ = vector<vector<int>>(width, vector<int>(height));

    threshold(img_src.clone(), img_src, 0, 255, cv::THRESH_OTSU);

    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            if(img_src.at<uchar>(y, x) == 0)
            {
                this->grid_map_[width-1-y][height-1-x] = constants::grid_type_obstacle;
                point_t tmp = {double(width-1-y), double(height-1-x)};
                obstaclePoints_.push_back(tmp);
            }
            else
            {   
                this->grid_map_[width-1-y][height-1-x] = constants::grid_type_free;
            }
        }
    }

    KDTree tmpTree(obstaclePoints_);
    this->obstacleTree_ = tmpTree;
}

bool frame::Plan()
{
    //unique_ptr<plannerBase> planner_ptr_ = make_unique<planner::Astar>();
    //unique_ptr<plannerBase> planner_ptr_ = make_unique<planner::RRT>();
    //unique_ptr<plannerBase> planner_ptr_ = make_unique<planner::RRTStar>();
    unique_ptr<plannerBase> planner_ptr_ = make_unique<planner::dubinsPlanner>();
    planner_ptr_->setMap(this->grid_map_, this->obstacleTree_);
    planner_ptr_->plan(start_, end_, &frontEnd_Path_);
    startStateFlag_ = false;
}


bool frame::planReady()
{
    return startStateFlag_ && endStateFlag_ && wayPointsFlag_;
}

bool frame::planReady_noWayPoints()
{
    return startStateFlag_ && endStateFlag_;
}

void frame::marker_Vis()
{
    int marker_id = 0;

    marker_ptr_->clear();

    if(wayPoints_.size() >= 2)
    {
        auto &p1 = wayPoints_[wayPoints_.size() - 2];
        auto &p2 = wayPoints_.back();
        if (p1.distance(p2) <= 0.001) 
        {
            wayPoints_.clear();
        }
    }

    visualization_msgs::Marker wayPoints_marker = marker_ptr_->newSphereList(1, "reference point", marker_id++, ros_viz_tools::RED, marker_frame_id_);
        for(size_t i = 0; i != wayPoints_.size(); i++){
            geometry_msgs::Point p;
            p.x = wayPoints_[i].x;
            p.y = wayPoints_[i].y;
            p.z = 1.0;
            wayPoints_marker.points.push_back(p);
        }
    marker_ptr_->append(wayPoints_marker);

    geometry_msgs::Vector3 scale;
    scale.x = 2.0;
    scale.y = 0.3;
    scale.z = 0.3;
    geometry_msgs::Pose start_pose;
    start_pose.position.x = start_.x;
    start_pose.position.y = start_.y;
    start_pose.position.z = 1.0;
    auto start_quat = tf::createQuaternionFromYaw(start_.heading);
    start_pose.orientation.x = start_quat.x();
    start_pose.orientation.y = start_quat.y();
    start_pose.orientation.z = start_quat.z();
    start_pose.orientation.w = start_quat.w();
    visualization_msgs::Marker start_marker =  marker_ptr_->newArrow(scale, start_pose, "start point", marker_id++, ros_viz_tools::CYAN, marker_frame_id_);
    marker_ptr_->append(start_marker);
    geometry_msgs::Pose end_pose;
    end_pose.position.x = end_.x;
    end_pose.position.y = end_.y;
    end_pose.position.z = 1.0;
    auto end_quat = tf::createQuaternionFromYaw(end_.heading);
    end_pose.orientation.x = end_quat.x();
    end_pose.orientation.y = end_quat.y();
    end_pose.orientation.z = end_quat.z();
    end_pose.orientation.w = end_quat.w();
    visualization_msgs::Marker end_marker = marker_ptr_->newArrow(scale, end_pose, "end point", marker_id++, ros_viz_tools::CYAN, marker_frame_id_);
    marker_ptr_->append(end_marker);


    ros_viz_tools::ColorRGBA path_color;
    path_color.r = 1.0;
    path_color.g = 0.0;
    path_color.b = 0.0;
    path_color.a = 1;

    visualization_msgs::Marker refer_path_marker = marker_ptr_->newLineStrip(0.2, "ref path", marker_id++, path_color, marker_frame_id_);
    for(size_t i = 0; i < frontEnd_Path_.size() ; ++i)
    {
        geometry_msgs::Point p;
        p.x = frontEnd_Path_[i].x;
        p.y = frontEnd_Path_[i].y;
        p.z = 1.0;
        refer_path_marker.points.push_back(p);
        refer_path_marker.colors.emplace_back(path_color);
    }
    marker_ptr_->append(refer_path_marker);

    // path_color.r = 0.063;
    // path_color.g = 0.305;
    // path_color.b = 0.545;
    // visualization_msgs::Marker opt_path_marker = marker_.newLineStrip(0.5, "opt path", marker_id++, path_color, marker_frame_id_);
    // for(size_t i = 0; i < optPath_.size(); ++i)
    // {
    //     geometry_msgs::Point p;
    //     p.x = optPath_[i].x;
    //     p.y = optPath_[i].y;
    //     p.z = 1.0;
    //     opt_path_marker.points.push_back(p);
    //     path_color.a = 1;
    //     opt_path_marker.colors.emplace_back(path_color);
    // }
    // marker_.append(opt_path_marker); 

    path_color.r = 0.1;
    path_color.g = 0.1;
    path_color.b = 0.8;
    // for(size_t i = 0; i < Corridor_.vis_boxes.size(); i++)
    // {
    //     visualization_msgs::Marker corridor_marker = marker_.newLineStrip(1, "corridor path", marker_id++, path_color, marker_frame_id_);
    //     for(size_t j = 0; j <= Corridor_.vis_boxes[i].size() ; ++j)
    //     {
    //         geometry_msgs::Point p;
    //         if(j == 0){
    //             p.x = Corridor_.vis_boxes[i][0];
    //             p.y = Corridor_.vis_boxes[i][1];
    //             p.z = 1.0;
    //         }
    //         if(j == 1){
    //             p.x = Corridor_.vis_boxes[i][0];
    //             p.y = Corridor_.vis_boxes[i][3];
    //             p.z = 1.0;
    //         }
    //         if(j == 2){
    //             p.x = Corridor_.vis_boxes[i][2];
    //             p.y = Corridor_.vis_boxes[i][3];
    //             p.z = 1.0;
    //         }
    //         if(j == 3){
    //             p.x = Corridor_.vis_boxes[i][2];
    //             p.y = Corridor_.vis_boxes[i][1];
    //             p.z = 1.0;
    //         }
    //         if(j == 4){
    //             p.x = Corridor_.vis_boxes[i][0];
    //             p.y = Corridor_.vis_boxes[i][1];
    //             p.z = 1.0;
    //         }
    //         corridor_marker.points.push_back(p);
    //         path_color.a = 1;
    //         corridor_marker.colors.emplace_back(path_color);
    //     }
    //     marker_.append(corridor_marker); 
    // }

}

void frame::LoopAction()
{
    ros::Time time = ros::Time::now();
    grid_map_for_rviz_.setTimestamp(time.toNSec());
    nav_msgs::OccupancyGrid message;
    grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_for_rviz_, "obstacle", 255, 0, message);
    map_publisher_.publish(message);
    marker_ptr_->publish();
}

void frame::wayPointCb(const geometry_msgs::PointStampedConstPtr &p) 
{
    State3D wayPoint;
    wayPoint.x = p->point.x;
    wayPoint.y = p->point.y;
    wayPoints_.emplace_back(wayPoint);
    // cout << "wpState x: " << wayPoint.x << "  y:" << wayPoint.y << endl;
    // cout << "state: " << grid_map_[int(wayPoint.x)][int(wayPoint.y)] << endl;
    wayPointsFlag_ = true;
}


void frame::startCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &start) 
{
    start_.x = start->pose.pose.position.x;
    start_.y = start->pose.pose.position.y;
    start_.heading = tf::getYaw(start->pose.pose.orientation);
    startStateFlag_ = true;
    cout << "startState x: " << start_.x << "  y:" << start_.y << "  head: " << start_.heading << endl;
    std::cout << "get initial state." << std::endl;
}


void frame::goalCb(const geometry_msgs::PoseStampedConstPtr &goal) 
{
    end_.x = goal->pose.position.x;
    end_.y = goal->pose.position.y;
    end_.heading = tf::getYaw(goal->pose.orientation);
    endStateFlag_ = true;
    cout << "goalState x: " << end_.x << "  y:" << end_.y << "  head: " << end_.heading << endl;
    std::cout << "get the goal." << std::endl;
}