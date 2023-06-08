#pragma once
#include <vector>
#include <limits>
#include <string>
#include <iostream>
#include <queue>

#include "common/dataStruct.hpp"

#include "tool/toolfunc.h"

using namespace std;

namespace tool
{

struct dubinsPath
{
    double d1, d2, d3, cost;
    string pathType;
};


class dubinsPathGenerator
{

    struct compare_Cost
    {
        bool operator() (const dubinsPath &a, const dubinsPath &b) // Comparison function for priority queue
        {
            return a.cost > b.cost; // min F
        }
    };   


public:

    void dubinsPlan(const State3D &start, const State3D &end, vector<State3D> &result);

    void dubinsPlan_no_collsion(const State3D &start, const State3D &end, vector<State3D> &result, const vector<vector<int>> &map);
    
    void set_Radius(const double &radius){radius_ = radius;}
    
    void set_Step_size(const double &step_size){step_size_ = step_size;}

    double mod2pi(double theta){ auto fun = [](double x,double y){return x - y*floor(x/y);}; return fun(theta,2 * M_PI);}

private:

    dubinsPath getBestPath(const double &alpha, const double &beta, const double &d);

    dubinsPath getBestPath_cost(const double &alpha, const double &beta, const double &d);

    vector<State3D> getBestPath_no_collsion(const double &alpha, const double &beta, const double &d, const vector<vector<int>> &map);

    vector<State3D> sampleDubinsPath(const dubinsPath& dubinsPath);

    State3D sampleDubsinPathPoint(const double &cur_length, const char &mode, const State3D &origin_state);

    bool is_path_collsion(const vector<State3D> &states, const vector<vector<int>> &map);

    dubinsPath LSL(const double &alpha, const double &beta, const double &d);
    dubinsPath RSR(const double &alpha, const double &beta, const double &d);
    dubinsPath LSR(const double &alpha, const double &beta, const double &d);
    dubinsPath RSL(const double &alpha, const double &beta, const double &d);
    dubinsPath RLR(const double &alpha, const double &beta, const double &d);
    dubinsPath LRL(const double &alpha, const double &beta, const double &d);

    double step_size_ = 1;
    double radius_ = 10; 

    

};




}