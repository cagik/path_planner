#pragma once

#include <cmath>
#include <iostream>
#include <queue>

#include "frame/plannerBase.h"

using namespace std;

namespace planner{

struct RRTNode 
{
    vector<shared_ptr<RRTNode>> children;
    shared_ptr<RRTNode> parent;
    State2D position;
};


class RRT : public plannerBase
{

public:

    bool plan(const State3D &start, const State3D &end, vector<State3D>* resultPath) override;
    
private:

    void PlannerParaInit();
    void getPath(vector<State3D> *result);

    shared_ptr<RRTNode> getRandomNode();
    shared_ptr<RRTNode> nearest(State2D &pos);
    State2D newPostion(State2D &a, State2D &b);
    bool isThereObstacleBetween(State2D &a, State2D &b);
    bool isPostionInObstacle(State2D &p);
    bool isPointInMap(State2D &p);
 
    void add(shared_ptr<RRTNode> qNearest, shared_ptr<RRTNode> qNew);

    bool reached();
    void deleteNodes(shared_ptr<RRTNode> root);


    State2D start_, end_;

    float end_dist_therehold_;
    int max_iter_;
    int step_size_;

    vector<shared_ptr<RRTNode>> node_ptr_vector_;
    shared_ptr<RRTNode> rootNode_ptr_;
    shared_ptr<RRTNode> lastNode_ptr_;

};

}