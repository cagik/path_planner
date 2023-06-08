#pragma once
#include <cmath>
#include <vector>


namespace toolFunc
{
    using namespace std;

bool isPointInMap(const int &x, const int &y, const vector<vector<int>> &map);

void translateBack(double &x, double &y, double tx, double ty);
    
void rotateBack(double &x, double &y, double angle);


}
