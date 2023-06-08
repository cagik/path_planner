#include "tool/toolfunc.h"

namespace toolFunc
{
    using namespace std;

bool isPointInMap(const int &x, const int &y, const vector<vector<int>> &map)
{
    return x > 0 && x < map.size() && y > 0 && y < map[0].size();
}

void translateBack(double &x, double &y, double tx, double ty)
{
    x -= tx;
    y -= ty;
}
void rotateBack(double &x, double &y, double angle)
{
    double new_x = x * cos(angle) + y * sin(angle);
    double new_y = x * (-sin(angle)) + y * cos(angle);
    x = new_x;
    y = new_y;
}

}