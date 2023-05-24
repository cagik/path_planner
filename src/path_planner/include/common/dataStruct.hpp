#pragma once

#include <cmath>

using namespace std;

struct State3D
{
    State3D() = default;
    State3D(double x, double y, double head) : x(x), y(y), heading(head){}
    double x{};
    double y{};
    double heading{};
    double distance(const State3D &s){return hypot(this->x - s.x, this->y - s.y);}
};