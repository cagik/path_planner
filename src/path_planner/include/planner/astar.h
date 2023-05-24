#ifndef ASTAR_H
#define ASTAR_H
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_map>

#include "frame/plannerBase.h"

using namespace std;

namespace planner{


class Astar : public plannerBase
{

    struct compare_Astar_F
    {
        bool operator() (pair<int, pair<int, int>> a, pair<int, pair<int, int>> b) // Comparison function for priority queue
        {
            return a.first > b.first; // min F
        }
    };    

public:

    bool plan(const State3D &start, const State3D &end, vector<State3D>* resultPath) override;
    
private:

    astarNode* FindPath();

    void GetPath(astarNode* TailNode, vector<State3D>* path);

    int point2index(pair<int, int> point) {return point.first * width_ + point.second;}

    pair<int, int> index2point(int index) {return pair<int, int>(int(index / width_), index % width_);}

    void releaseMemory();
    

public:

    vector<pair<int, int>> updateObsPoints;

private:
    
    pair<int, int> startPoint_, targetPoint_;
    vector<vector<int>> neighbor_;

    vector<astarNode*> PathList;

    priority_queue<pair<long, pair<int, int>>, vector<pair<int, pair<int, int>>>, compare_Astar_F> OpenList;

    unordered_map<long, astarNode*> OpenDict;

    int width_;
    int height_;

};
}

#endif