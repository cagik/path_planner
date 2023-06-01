#pragma once

#include <cmath>


#include "frame/plannerBase.h"

using namespace std;

namespace planner{

struct RRTStarNode 
{
    shared_ptr<RRTStarNode> parent;
    State2D position;
    float cost;
};


class RRTStar : public plannerBase
{

public:

    bool plan(const State3D &start, const State3D &end, vector<State3D>* resultPath) override;
    
private:

    void PlannerParaInit();
    void getPath(vector<State3D> *result);

    shared_ptr<RRTStarNode> getRandomNode();
    shared_ptr<RRTStarNode> nearest(State2D &pos);
    State2D newPostion(State2D &a, State2D &b);
    bool isThereObstacleBetween(State2D &a, State2D &b);
    bool isPostionInObstacle(State2D &p);
    bool isPointInMap(State2D &p);
 
    void add(shared_ptr<RRTStarNode> qNearest, shared_ptr<RRTStarNode> qNew);
    bool reached();


    State2D start_, end_;

    float rewire_radius_;
    float end_dist_therehold_;
    int max_iter_;
    int step_size_;

    vector<shared_ptr<RRTStarNode>> node_ptr_vector_;
    shared_ptr<RRTStarNode> rootNode_ptr_;
    shared_ptr<RRTStarNode> lastNode_ptr_;

};

}