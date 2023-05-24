#include "frame/plannerBase.h"

plannerBase::plannerBase(){}

void plannerBase::setMap(const vector<vector<int>> &map, KDTree &obstacletree)
{
    this->grid_map_ = map;
    this->obstacleTree_ = obstacletree;
}

bool plannerBase::plan(const State3D &start, const State3D &end, vector<State3D>* resultPath){}