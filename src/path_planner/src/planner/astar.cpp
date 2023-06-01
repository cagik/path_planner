#include "planner/astar.h"


namespace planner{

bool Astar::plan(const State3D &start, const State3D &end, vector<State3D>* resultPath)
{
    this->neighbor_ = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},            {0, 1},
            {1, -1},   {1, 0},  {1, 1}
    };

    this->startPoint_ = make_pair(int(start.x), int(start.y));
    this->targetPoint_ = make_pair(int(end.x), int(end.y));

    // cout << "start : " << start.x << "  " << start.y << endl;
    // cout << "end : " << end.x << "  " << end.y << endl;

    astarNode* TailNode = FindPath();
    GetPath(TailNode, resultPath);
    releaseMemory();

    return true;
}


astarNode* Astar::FindPath()
{

    astarNode* startPointNode = new astarNode(startPoint_);
    OpenList.push(pair<int, pair<int, int>>(startPointNode->F, startPointNode->point));
    int index = point2index(startPointNode->point);
    OpenDict[index] = startPointNode;
    grid_map_[startPoint_.first][startPoint_.second] = inOpenList;

    int step = 0;
    
    while(!OpenList.empty())
    {
        // Find the node with least F value
        //cout << "start :" << step << endl;
        pair<int, int> CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        astarNode* CurNode = OpenDict[index];
        OpenDict.erase(index);

        int curX = CurPoint.first;
        int curY = CurPoint.second;

        grid_map_[curX][curY] = inCloseList;


        if(curX == targetPoint_.first && curY == targetPoint_.second)
        {
            return CurNode; // Find a valid path
        }
       
        // Traversal the neighborhood
        for(int k = 0;k < neighbor_.size();k++)
        {   
            int y = curY + neighbor_[k][1];
            int x = curX + neighbor_[k][0];

            if(x < 0 || x >= this->map_height_ || y < 0 || y >= this->map_width_){
                continue;
            }

            if(grid_map_[x][y] == freeGrid || grid_map_[x][y] == inOpenList)
            {
                // Determine whether a diagonal line can pass
                int dist1 = abs(neighbor_[k][0]) + abs(neighbor_[k][1]);
                if(dist1 == 2 && grid_map_[x][curY] == obstacle && grid_map_[curX][y] == obstacle)
                    continue;

                int addG, G, H, F;
                if(dist1 == 2)
                {
                    addG = 14;
                }
                else
                {
                    addG = 10;
                }
                G = CurNode->G + addG;
                
                int dist2 = (x - targetPoint_.first) * (x - targetPoint_.first) + (y - targetPoint_.second) * (y - targetPoint_.second);
                H = round(10 * sqrt(dist2));
               // H = 10 * (abs(x - targetPoint.x) + abs(y - targetPoint.y));
                F = G + H;
             
                // Update the G, H, F value of node
                if(grid_map_[x][y] == freeGrid)
                {
                    astarNode* node = new astarNode();
                    node->point = make_pair(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    OpenList.push(pair<int, pair<int, int>>(node->F, node->point));
                    int index = point2index(node->point);
                    OpenDict[index] = node;
                    grid_map_[x][y] = inOpenList;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
                     
                    int index = point2index(make_pair(x, y));
                    astarNode* node = OpenDict[index];

                    if(G < node->G)
                    {
                        node->G = G;
                        node->F = F;
                        node->parent = CurNode;
                    }
                }
            }
        }
        step++;
    }
    cout << "astar failure" << endl;
    return NULL; 
}


/**
 * @description: 反转路径
 * @param {Node*} TailNode
 * @return {*}
 */
void Astar::GetPath(astarNode* TailNode, vector<State3D>* path)
{
    PathList.clear();
    path->clear();

    astarNode* CurNode = TailNode;
    while(CurNode != NULL)
    {
        PathList.push_back(CurNode);
        CurNode = CurNode->parent;
    }

    int length = PathList.size();
    for(int i = 0;i < length;i++)
    {
        State3D tmp(PathList.back()->point.first, PathList.back()->point.second, 0);
        path->push_back(tmp);
        PathList.pop_back();
    }

}

void Astar::releaseMemory()
{
    while(OpenList.size()) {
        pair<int, int> CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        astarNode* CurNode = OpenDict[index];
        delete CurNode;
    }
    OpenDict.clear();
}



} 