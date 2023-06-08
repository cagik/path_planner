#include "planner/rrtDubins.h"

namespace planner
{

bool RRTDubins::plan(const State3D &start, const State3D &end, vector<State3D> *resultPath)
{

    dubinsPathGen_ptr_ = make_shared<tool::dubinsPathGenerator>();
    dubinsPathGen_ptr_->set_Radius(10);
    dubinsPathGen_ptr_->set_Step_size(0.2);

    start_.x = start.x;
    start_.y = start.y;
    start_.heading = start.heading;
    
    end_.x = end.x;
    end_.y = end.y;
    end_.heading = end.heading;

    PlannerParaInit();
    getPath(resultPath);

    return true;
}

void RRTDubins::PlannerParaInit()
{
    end_dist_therehold_ = 5;
    step_size_ = 3;
    max_iter_ = 10000;
}

void RRTDubins::getPath(vector<State3D> *result)
{

    rootNode_ptr_ = make_shared<RRTDubinsNode>();
    rootNode_ptr_->parent = nullptr;
    rootNode_ptr_->position = start_;
    lastNode_ptr_ = rootNode_ptr_;
    node_ptr_vector_.push_back(rootNode_ptr_);

    for (int i = 0; i < max_iter_; i++)
    {
        shared_ptr<RRTDubinsNode> n = this->getRandomNode();
        if(n != nullptr)
        {
            shared_ptr<RRTDubinsNode> n_nearest = this->nearest(n->position);
            if(n->position.distance(n_nearest->position) > this->step_size_)
            {
                State3D new_pos = this->newPostion(n_nearest->position, n->position);

                if(!isThereObstacleBetween(new_pos, n_nearest->position))
                {
                    shared_ptr<RRTDubinsNode> n_new = make_shared<RRTDubinsNode>();
                    n_new->position = new_pos;
                    this->add(n_nearest, n_new);
                }
            }
        }
        if(this->reached())
        {
            break;
        }
    }

    shared_ptr<RRTDubinsNode> node;

    if (this->reached()) 
    {
        node = this->lastNode_ptr_;
    }
    else
    {
        node = this->nearest(this->end_);
    }


    vector<State3D> Nodes_state;

    while (node != nullptr) 
    {
        Nodes_state.push_back(node->position);
        node = node->parent;
    }
    reverse(Nodes_state.begin(), Nodes_state.end());

    for (size_t i = 0; i < Nodes_state.size() - 1; i++)
    {
        State3D from_state = Nodes_state[i];
        State3D to_state = Nodes_state[i + 1];
        vector<State3D> tmp_dubinsPath;
        dubinsPathGen_ptr_->dubinsPlan(from_state, to_state, tmp_dubinsPath);
        result->insert(result->end(), tmp_dubinsPath.begin(), tmp_dubinsPath.end());
    }
    
    
    // result->clear();

    // while (node != nullptr) 
    // {
    //     State3D tmp;
    //     tmp.x = node->position.x;
    //     tmp.y = node->position.y;
    //     result->push_back(tmp);
    //     node = node->parent;
    // }

    // reverse(result->begin(), result->end());    

}

shared_ptr<RRTDubinsNode> RRTDubins::getRandomNode()
{   
    shared_ptr<RRTDubinsNode> randomNode;
    State3D pos;

    double is_use_goal = drand48() * 100;
    if(is_use_goal > 5)
    {
        pos.x = drand48() * this->map_width_;
        pos.y = drand48() * this->map_height_;
        pos.heading = drand48() * M_PI * 2 - M_PI;

        if (pos.x >= 0 && pos.x <= this->map_width_ && pos.y >= 0 && pos.y <= this->map_height_) 
        {
        randomNode = make_shared<RRTDubinsNode>();
        randomNode->position = pos;
        return randomNode;
        }
        return nullptr;
    }
    else{
        pos = end_;
    }
}

shared_ptr<RRTDubinsNode> RRTDubins::nearest(State3D &pos)
{
    float minDist = 1e9;
    shared_ptr<RRTDubinsNode> closest = shared_ptr<RRTDubinsNode>();
    closest = nullptr;
    for(int i = 0; i < this->node_ptr_vector_.size(); i++) {
        float dist = pos.distance(node_ptr_vector_[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = node_ptr_vector_[i];
        }
    }
    return closest;
}

State3D RRTDubins::newPostion(State3D &a, State3D &b)
{
    double theta = atan2(b.y - a.y, b.x - a.x);
    State3D new_Pos;
    new_Pos.x = a.x + this->step_size_ * cos(theta);
    new_Pos.y = a.y + this->step_size_ * sin(theta);
    new_Pos.heading = b.heading;
    
    return new_Pos;
}

bool RRTDubins::check_path_collsion(const vector<State3D> &path)
{
    for(const auto &state: path)
    {
        if(!isPointInMap(state))
        {
            return true;
        }
        if(isPostionInObstacle(state))
        {
            return true;
        }
    }
    return false;
}


bool RRTDubins::isThereObstacleBetween(State3D &a, State3D &b)
{
    double dist = a.distance(b);
   if(dist < 1){
        if(isPostionInObstacle(a) || isPostionInObstacle(b)){
            return true;
        }
        else{
            return false;;
        }
    }
    else{
        vector<State3D> dubinsPath;  
        dubinsPathGen_ptr_->dubinsPlan(a, b, dubinsPath);
        if(check_path_collsion(dubinsPath))
        {
            return true;
        }
        else{
            return false;
        }
    }
}

bool RRTDubins::isPostionInObstacle(const State3D &p)
{
    if(!isPointInMap(p))
    {
        return true;
    }

    if(this->grid_map_[(int)p.x][(int)p.y] == obstacle)
    {

        return true;
    }
    else{
        return false;
    }
}

bool RRTDubins::isPointInMap(const State3D &p)
{
    return p.x > 0 && p.x < this->map_width_ && p.y > 0 && p.y < this->map_height_;
}

void RRTDubins::add(shared_ptr<RRTDubinsNode> qNearest, shared_ptr<RRTDubinsNode> qNew)
{
    qNew->parent = qNearest;
    node_ptr_vector_.push_back(qNew);
    lastNode_ptr_ = qNew;
}

bool RRTDubins::reached()
{
    return lastNode_ptr_->position.distance(end_) < end_dist_therehold_;
}




}