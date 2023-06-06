#include "planner/dubinsPlanner.h"

namespace planner{

bool dubinsPlanner::plan(const State3D &start, const State3D &end, vector<State3D> *resultPath)
{
    tool::dubinsPathGenerator dpg;
    dpg.dubinsPlan(start, end, *resultPath);
    //reverse(resultPath->begin(), resultPath->end());

    for(auto state: (*resultPath))
    {
        //cout << "x : " <<  state.x << " y : " << state.y << "  head:  " << state.heading << endl;
    }

    //cout << resultPath->size() << endl;

    return true;
}

}