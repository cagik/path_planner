#ifndef ASTAR_H
#define ASTAR_H
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_map>

#include "frame/plannerBase.h"

using namespace std;

namespace planner{

struct astarNode
{
    pair<int, int> point;
    int F, G, H; 
    astarNode* parent; 
    astarNode(pair<int, int> _point = make_pair(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL){}
};


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

    int point2index(pair<int, int> point) {return point.first * this->map_width_ + point.second;}

    pair<int, int> index2point(int index) {return pair<int, int>(int(index / this->map_width_), index % this->map_width_);}

    void releaseMemory();
    
private:
    
    pair<int, int> startPoint_, targetPoint_;
    
    vector<vector<int>> neighbor_;

    vector<astarNode*> PathList;

    priority_queue<pair<long, pair<int, int>>, vector<pair<int, pair<int, int>>>, compare_Astar_F> OpenList;

    unordered_map<long, astarNode*> OpenDict;



};
}

#endif