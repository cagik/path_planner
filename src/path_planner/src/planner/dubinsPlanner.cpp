#include "planner/dubinsPlanner.h"

namespace planner{

bool dubinsPlanner::plan(const State3D &start, const State3D &end, vector<State3D> *resultPath)
{
    tool::dubinsPathGenerator dpg;
    //dpg.dubinsPlan_no_collsion(start, end, *resultPath, this->grid_map_);
    dpg.set_Radius(10);
    dpg.set_Step_size(0.2);
    dpg.dubinsPlan(start, end, *resultPath);

    return true;
}

}