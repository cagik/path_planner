#pragma once

#include <cmath>

using namespace std;

struct State2D
{
    State2D() = default;
    State2D(double x, double y) : x(x), y(y){}
    double x{};
    double y{};
    double distance(const State2D &s){return hypot(this->x - s.x, this->y - s.y);}
};
struct State3D
{
    State3D() = default;
    State3D(double x, double y, double head) : x(x), y(y), heading(head){}
    double x{};
    double y{};
    double heading{};
    double distance(const State3D &s){return hypot(this->x - s.x, this->y - s.y);}
};

enum NodeType{
    freeGrid = 0,
    obstacle,
    inOpenList,
    inCloseList
};
