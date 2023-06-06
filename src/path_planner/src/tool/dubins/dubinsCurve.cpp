#include "tool/dubins/dubinsCurve.h"

namespace tool
{


void dubinsPathGenerator::dubinsPlan(const State3D &start, const State3D &end, vector<State3D> &result)
{
    cout << "###############" << endl << endl;
    cout << "start_x = " << start.x << endl << "start_y = " << start.y << endl << "start_yaw = " << start.heading <<  endl;
    cout << "###############" << endl << endl;
    cout << "end_x = " << end.x << endl << "end_y = " << end.y << endl << "end_yaw = " << end.heading <<  endl;
    cout << "###############" << endl << endl;

    double rotate_angle = start.heading;
    double local_x_end = end.x;
    double local_y_end = end.y;

    translateBack(local_x_end, local_y_end, start.x, start.y);

    cout << "local x: " <<  local_x_end << " y: " <<  local_y_end  << endl;
    cout << "###############" << endl << endl;

    rotateBack(local_x_end, local_y_end, rotate_angle);

    cout << cos(rotate_angle) << "  " <<  -sin(rotate_angle) << endl;
    cout << sin(rotate_angle) << "  " <<  cos(rotate_angle) << endl;
    cout << endl << "#################" << endl;


    double local_heading_end = end.heading - rotate_angle;

    cout << "local x: " <<  local_x_end << " y: " <<  local_y_end << "  head: " <<  local_heading_end << endl;

    
    double d = hypot(local_x_end, local_y_end) / radius_;
    double theta = mod2pi(atan2(local_y_end, local_x_end));
    double alpha = mod2pi(-theta);
    double beta = mod2pi(local_heading_end - theta);

    dubinsPath best_path = getBestPath(alpha, beta, d);

    cout << endl << "#################" << endl;

    cout << "d1 : " <<  best_path.d1 << " d2: " <<  best_path.d2 << "  d3: " <<  best_path.d3 << endl;
    cout << "mode : " <<  best_path.pathType << endl;

    result = sampleDubinsPath(best_path);

    State3D end_state = result.back();
    cout << "end x: " <<  end_state.x << " y: " <<  end_state.y << "  head: " <<  end_state.heading << endl;

    for(auto &state :result)
    {
        rotateBack(state.x, state.y, -rotate_angle);
        translateBack(state.x, state.y, -start.x, -start.y);
        state.heading += rotate_angle;
    }
    

}

dubinsPath dubinsPathGenerator::getBestPath(const double &alpha, const double &beta, const double &d)
{
    vector<dubinsPath> all_dubsin_path = {LSL(alpha, beta, d), RSR(alpha, beta, d), LSR(alpha, beta, d),
                                          RSL(alpha, beta, d), RLR(alpha, beta, d), LRL(alpha, beta, d)};
    double best_cost = numeric_limits<double>::max();
    dubinsPath best_path;
    for(const auto &path : all_dubsin_path)
    {
        if(path.cost < 0)
        {
            continue;
        }
        if(path.cost < best_cost)
        {
            best_path = path;
            best_cost = path.cost;
        }
    }
    return best_path;
}

vector<State3D> dubinsPathGenerator::sampleDubinsPath(const dubinsPath& path)
{
    State3D orgin_state3D;
    orgin_state3D.x = 0.0;
    orgin_state3D.y = 0.0;
    orgin_state3D.heading = 0;
    vector<State3D> result = {orgin_state3D};

    vector<char> path_mode = {path.pathType[0], path.pathType[1],path.pathType[2]};
    vector<double> path_length = {path.d1, path.d2, path.d3};

    for(int i = 0; i < 3; i++)
    {
        char mode = path_mode[i];
        double length = path_length[i];
        if(length == 0)
        {
            continue;
        }

        State3D orgin_state = result.back();

        double cur_length = step_size_;

        while(abs(cur_length + step_size_) <= abs(length))
        {
            result.push_back(sampleDubsinPathPoint(cur_length, mode, orgin_state));
            State3D debug_s = result.back();
            cur_length += step_size_;
        }
        result.push_back(sampleDubsinPathPoint(length, mode, orgin_state));
    }
    return result;
}


State3D dubinsPathGenerator::sampleDubsinPathPoint(const double &cur_length, const char &mode, const State3D &origin_state)
{
    State3D state_tmp;
    if(mode == 'S')
    {
        state_tmp.x = origin_state.x + cur_length * radius_ * cos(origin_state.heading);
        state_tmp.y = origin_state.y + cur_length * radius_ * sin(origin_state.heading);
        state_tmp.heading = origin_state.heading;
    }
    else
    {
        double ldx = sin(cur_length) * radius_;
        double ldy = 0.0;
        double dhead = 0.0;
        if(mode == 'L')
        {
            ldy = (1.0 - cos(cur_length)) * radius_;
            dhead = origin_state.heading + cur_length;
        }
        else
        {
            ldy = (1.0 - cos(cur_length)) * radius_ * -1;
            dhead = origin_state.heading - cur_length;
        }
        double gdx = cos(- origin_state.heading) * ldx + sin(- origin_state.heading) * ldy;
        double gdy = - sin(- origin_state.heading) * ldx + cos(- origin_state.heading) * ldy;
        state_tmp.x = origin_state.x + gdx;
        state_tmp.y = origin_state.y + gdy;
        state_tmp.heading = dhead;
    }
    return state_tmp;
}



void dubinsPathGenerator::translateBack(double &x, double &y, double tx, double ty)
{
    x -= tx;
    y -= ty;
}
void dubinsPathGenerator::rotateBack(double &x, double &y, double angle)
{
    double new_x = x * cos(angle) + y * sin(angle);
    double new_y = x * (-sin(angle)) + y * cos(angle);
    x = new_x;
    y = new_y;
}

dubinsPath dubinsPathGenerator::LSL(const double &alpha, const double &beta, const double &d)
{
    dubinsPath dubins;
    double sin_a = sin(alpha);
    double sin_b = sin(beta);
    double cos_a = cos(alpha);
    double cos_b = cos(beta);
    double cos_ab = cos(alpha - beta);
    double p_squared = 2 + pow(d, 2) - (2 * cos_ab) + (2 * d * (sin_a - sin_b));
    if(p_squared < 0)
    {
        dubins.cost = -1;
        return dubins;
    }
    double tmp = atan2((cos_b - cos_a), d + sin_a - sin_b);
    dubins.d1 = mod2pi(-alpha + tmp);
    dubins.d2 = sqrt(p_squared);
    dubins.d3 = mod2pi(beta - tmp);
    dubins.pathType = "LSL";
    dubins.cost = abs(dubins.d1) + abs(dubins.d2) + abs(dubins.d3);
    return dubins;
}

dubinsPath dubinsPathGenerator::RSR(const double &alpha, const double &beta, const double &d)
{
    dubinsPath dubins;
    double sin_a = sin(alpha);
    double sin_b = sin(beta);
    double cos_a = cos(alpha);
    double cos_b = cos(beta);
    double cos_ab = cos(alpha - beta);
    double p_squared = 2 + pow(d, 2) - (2 * cos_ab) + (2 * d * (- sin_a + sin_b));
    if(p_squared < 0)
    {
        dubins.cost = -1;
        return dubins;
    }
    double tmp = atan2((cos_a - cos_b), d - sin_a + sin_b);
    dubins.d1 = mod2pi(alpha - tmp);
    dubins.d2 = sqrt(p_squared);
    dubins.d3 = mod2pi(-beta + tmp);
    dubins.pathType = "RSR";
    dubins.cost = abs(dubins.d1) + abs(dubins.d2) + abs(dubins.d3);
    return dubins;
}

dubinsPath dubinsPathGenerator::LSR(const double &alpha, const double &beta, const double &d)
{
    dubinsPath dubins;
    double sin_a = sin(alpha);
    double sin_b = sin(beta);
    double cos_a = cos(alpha);
    double cos_b = cos(beta);
    double cos_ab = cos(alpha - beta);
    double p_squared = - 2 + pow(d, 2) + (2 * cos_ab) + (2 * d * (sin_a + sin_b));
    if(p_squared < 0)
    {
        dubins.cost = -1;
        return dubins;
    }
    dubins.d2 = sqrt(p_squared);
    double tmp = atan2(( - cos_a - cos_b), d + sin_a + sin_b) - atan2(-2.0, dubins.d2);
    dubins.d1 = mod2pi(-alpha + tmp);
    dubins.d3 = mod2pi(- mod2pi(beta) + tmp);
    dubins.pathType = "LSR";
    dubins.cost = abs(dubins.d1) + abs(dubins.d2) + abs(dubins.d3);
    return dubins;
}

dubinsPath dubinsPathGenerator::RSL(const double &alpha, const double &beta, const double &d)
{
    dubinsPath dubins;
    double sin_a = sin(alpha);
    double sin_b = sin(beta);
    double cos_a = cos(alpha);
    double cos_b = cos(beta);
    double cos_ab = cos(alpha - beta);
    double p_squared = - 2 + pow(d, 2) + (2 * cos_ab) - (2 * d * (sin_a + sin_b));
    if(p_squared < 0)
    {
        dubins.cost = -1;
        return dubins;
    }
    dubins.d2 = sqrt(p_squared);
    double tmp = atan2((cos_a + cos_b), d - sin_a - sin_b) - atan2(2.0, dubins.d2);
    dubins.d1 = mod2pi(alpha - tmp);
    dubins.d3 = mod2pi(beta - tmp);
    dubins.pathType = "RSL";
    dubins.cost = abs(dubins.d1) + abs(dubins.d2) + abs(dubins.d3);
    return dubins;
}

dubinsPath dubinsPathGenerator::RLR(const double &alpha, const double &beta, const double &d)
{
    dubinsPath dubins;
    double sin_a = sin(alpha);
    double sin_b = sin(beta);
    double cos_a = cos(alpha);
    double cos_b = cos(beta);
    double cos_ab = cos(alpha - beta);
    double tmp = (6.0 - pow(d, 2) + 2.0 * cos_ab + 2.0 * d * (sin_a - sin_b)) / 8.0;
    if(abs(tmp) > 1)
    {
        dubins.cost = -1;
        return dubins;
    }
    dubins.d2 = mod2pi(2 * M_PI - acos(tmp));
    dubins.d1 = mod2pi(alpha - atan2(cos_a - cos_b, d - sin_a + sin_b) + dubins.d2 / 2.0);
    dubins.d3 = mod2pi(alpha - beta - dubins.d1 + dubins.d2);
    dubins.pathType = "RLR";
    dubins.cost = abs(dubins.d1) + abs(dubins.d2) + abs(dubins.d3);
    return dubins;
}

dubinsPath dubinsPathGenerator::LRL(const double &alpha, const double &beta, const double &d)
{
    dubinsPath dubins;
    double sin_a = sin(alpha);
    double sin_b = sin(beta);
    double cos_a = cos(alpha);
    double cos_b = cos(beta);
    double cos_ab = cos(alpha - beta);
    double tmp = (6.0 - pow(d, 2) + 2.0 * cos_ab + 2.0 * d * (- sin_a + sin_b)) / 8.0;
    if(abs(tmp) > 1)
    {
        dubins.cost = -1;
        return dubins;
    }
    dubins.d2 = mod2pi(2 * M_PI - acos(tmp));
    dubins.d1 = mod2pi(-alpha - atan2(cos_a - cos_b, d + sin_a - sin_b) + dubins.d2 / 2.0);
    dubins.d3 = mod2pi(mod2pi(beta) - alpha - dubins.d1 + mod2pi(dubins.d2));
    dubins.pathType = "LRL";
    dubins.cost = abs(dubins.d1) + abs(dubins.d2) + abs(dubins.d3);
    return dubins;
}

}