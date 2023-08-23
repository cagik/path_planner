#pragma once
#include <random>
#include <cmath>

#include "tool/dubins/dubinsCurve.h"
#include "frame/plannerBase.h"

using namespace std;

namespace planner{

struct RRTDubinsNode
{
    shared_ptr<RRTDubinsNode> parent;
    State3D position;
};


class RRTDubins : public plannerBase
{

public:

    bool plan(const State3D &start, const State3D &end, vector<State3D>* resultPath) override;
    
private:


    void PlannerParaInit();
    bool getPath(vector<State3D> *result);

    shared_ptr<RRTDubinsNode> getRandomNode();
    shared_ptr<RRTDubinsNode> nearest(State3D &pos);
    State3D newPostion(State3D &a, State3D &b);
    bool isThereObstacleBetween(State3D &a, State3D &b);

    bool isPostionInObstacle(const State3D &p);

    bool isPointInMap(const State3D &p);
 
    void add(shared_ptr<RRTDubinsNode> qNearest, shared_ptr<RRTDubinsNode> qNew);
    bool reached();

    bool check_path_collsion(const vector<State3D> &path);

    State3D start_, end_;

    float end_dist_therehold_;
    int max_iter_;
    int step_size_;

    vector<shared_ptr<RRTDubinsNode>> node_ptr_vector_;
    shared_ptr<RRTDubinsNode> rootNode_ptr_;
    shared_ptr<RRTDubinsNode> lastNode_ptr_;

    shared_ptr<tool::dubinsPathGenerator> dubinsPathGen_ptr_;


};

}