#pragma once
#include <cmath>
#include <iostream>

#include "frame/plannerBase.h"

#include "tool/dubins/dubinsCurve.h"

using namespace std;

namespace planner{


class dubinsPlanner : public plannerBase
{

public:

    bool plan(const State3D &start, const State3D &end, vector<State3D>* resultPath) override;
    


};
}

