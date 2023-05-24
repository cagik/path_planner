#pragma once
#include <vector>

#include "tool/KDTree/KDTree.hpp"
#include "common/dataStruct.hpp"

using namespace std;

class plannerBase
{
public:
    plannerBase();
    void setMap(const vector<vector<int>> &map, KDTree &obstacletree);
    virtual bool plan(const State3D &start, const State3D &end, vector<State3D>* resultPath);

protected:

    vector<vector<int>> grid_map_;
    KDTree obstacleTree_;

};


