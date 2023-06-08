#include "planner/rrtStar.h"


namespace planner{

bool RRTStar::plan(const State3D &start, const State3D &end, vector<State3D> *resultPath)
{
    start_.x = start.x;
    start_.y = start.y;
    end_.x = end.x;
    end_.y = end.y;
    
    PlannerParaInit();
    getPath(resultPath);


    return true;
}

void RRTStar::PlannerParaInit()
{
    rewire_radius_ = 20;
    end_dist_therehold_ = 5;
    step_size_ = 3;
    max_iter_ = 10000;
}

void RRTStar::getPath(vector<State3D> *result)
{

    rootNode_ptr_ = make_shared<RRTStarNode>();
    rootNode_ptr_->parent = nullptr;
    rootNode_ptr_->position = start_;
    rootNode_ptr_->cost = 0.0;
    lastNode_ptr_ = rootNode_ptr_;
    node_ptr_vector_.push_back(rootNode_ptr_);

    for (int i = 0; i < max_iter_; i++)
    {
        shared_ptr<RRTStarNode> n = this->getRandomNode();
        if(n != nullptr)
        {
            shared_ptr<RRTStarNode> n_nearest = this->nearest(n->position);
            if(n->position.distance(n_nearest->position) > this->step_size_)
            {
                State2D new_pos = this->newPostion(n_nearest->position, n->position);
                if(!isThereObstacleBetween(new_pos, n_nearest->position))
                {
                    shared_ptr<RRTStarNode> n_new = make_shared<RRTStarNode>();
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

    shared_ptr<RRTStarNode> node;

    if (this->reached()) 
    {
        node = this->lastNode_ptr_;
    }
    else
    {
        node = this->nearest(this->end_);
    }

    
    result->clear();

    while (node != nullptr) 
    {
        State3D tmp;
        tmp.x = node->position.x;
        tmp.y = node->position.y;
        result->push_back(tmp);
        node = node->parent;
    }

    reverse(result->begin(), result->end());    

}

shared_ptr<RRTStarNode> RRTStar::getRandomNode()
{   
    shared_ptr<RRTStarNode> randomNode;
    State2D pos;
    pos.x = drand48() * this->map_width_;
    pos.y = drand48() * this->map_height_;
    if (pos.x >= 0 && pos.x <= this->map_width_ && pos.y >= 0 && pos.y <= this->map_height_) {
        randomNode = make_shared<RRTStarNode>();
        randomNode->position = pos;
        return randomNode;
    }
    return nullptr;

}

shared_ptr<RRTStarNode> RRTStar::nearest(State2D &pos)
{
    float minDist = 1e9;
    shared_ptr<RRTStarNode> closest = shared_ptr<RRTStarNode>();
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

State2D RRTStar::newPostion(State2D &a, State2D &b)
{
    double theta = atan2(b.y - a.y, b.x - a.x);
    State2D new_Pos;
    new_Pos.x = a.x + this->step_size_ * cos(theta);
    new_Pos.y = a.y + this->step_size_ * sin(theta);
    return new_Pos;
}

bool RRTStar::isThereObstacleBetween(State2D &a, State2D &b)
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
        int step_number = static_cast<int>(dist/1);
        double dx = (a.x - b.x) / step_number;
        double dy = (a.y - b.y) / step_number;

        State2D p_tmp;
        for(int i = 0; i < step_number; i++){
            p_tmp.x = b.x + i * dx;
            p_tmp.y = b.y + i * dy;

            if(isPostionInObstacle(p_tmp)){
                return true;
            }
        }
        return false;
    }
}

bool RRTStar::isPostionInObstacle(State2D &p)
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

bool RRTStar::isPointInMap(State2D &p)
{
    return p.x > 0 && p.x < this->map_width_ && p.y > 0 && p.y < this->map_height_;
}

void RRTStar::add(shared_ptr<RRTStarNode> qNearest, shared_ptr<RRTStarNode> qNew)
{
    float cost_new_node, cost_other_parent;
    cost_new_node = qNearest->cost + qNearest->position.distance(qNew->position);

    for(auto &node : node_ptr_vector_)
    {
        float dist = node->position.distance(qNew->position);

        cost_other_parent = node->cost + dist;

        if (dist < rewire_radius_ && cost_other_parent < cost_new_node 
            && !isThereObstacleBetween(node->position, qNew->position))
        {
            qNew->parent = node;
            qNew->cost = cost_other_parent;
        }
        else{
            qNew->parent = qNearest;
            qNew->cost = cost_new_node;
        }
    }
    node_ptr_vector_.push_back(qNew);
    lastNode_ptr_ = qNew;

    for(auto &node : node_ptr_vector_){
        double dist = node->position.distance(lastNode_ptr_->position);
        if(node->parent != nullptr && dist < rewire_radius_){
            double cost_node = lastNode_ptr_->cost + dist;
            if(cost_node < node->cost && !isThereObstacleBetween(node->position, lastNode_ptr_->position)){
                node->parent = lastNode_ptr_;
                node->cost = cost_node;
            }
        }
    }

}

bool RRTStar::reached()
{
    shared_ptr<RRTStarNode> tmp = node_ptr_vector_.back();
    return tmp->position.distance(end_) < end_dist_therehold_  && !isThereObstacleBetween(tmp->position, end_);
}

}