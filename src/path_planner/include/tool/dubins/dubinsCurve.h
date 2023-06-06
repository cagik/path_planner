#pragma once
#include <vector>
#include <limits>
#include <string>
#include <iostream>
#include "common/dataStruct.hpp"

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
public:

    void dubinsPlan(const State3D &start, const State3D &end, vector<State3D> &result);
    void set_Radius(const double &radius){radius_ = radius;}
    void set_Step_size(const double &step_size){step_size_ = step_size;}

    double mod2pi(double theta){ auto fun = [](double x,double y){return x - y*floor(x/y);}; return fun(theta,2 * M_PI);}

private:

    dubinsPath getBestPath(const double &alpha, const double &beta, const double &d);

    void translateBack(double &x, double &y, double tx, double ty);
    void rotateBack(double &x, double &y, double angle);


    vector<State3D> sampleDubinsPath(const dubinsPath& dubinsPath);

    State3D sampleDubsinPathPoint(const double &cur_length, const char &mode, const State3D &origin_state);

    dubinsPath LSL(const double &alpha, const double &beta, const double &d);
    dubinsPath RSR(const double &alpha, const double &beta, const double &d);
    dubinsPath LSR(const double &alpha, const double &beta, const double &d);
    dubinsPath RSL(const double &alpha, const double &beta, const double &d);
    dubinsPath RLR(const double &alpha, const double &beta, const double &d);
    dubinsPath LRL(const double &alpha, const double &beta, const double &d);

    double step_size_ = 0.1;
    double radius_ = 5; 

};




}