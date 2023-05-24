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

enum NodeType{
    freeGrid = 0,
    obstacle,
    inOpenList,
    inCloseList
};
struct astarNode{
    
    pair<int, int> point;

    int F, G, H; 

    astarNode* parent; 

    astarNode(pair<int, int> _point = make_pair(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
    {
    }
};