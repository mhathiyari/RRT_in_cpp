#pragma once

#include "kdTreeNode.hpp"
#include "dynamics.hpp"

// Uncomment for visualization 
// #define VISUALIZATION
#ifdef VISUALIZATION
#include "visualizer.hpp"
#endif
//

using namespace std; 
using namespace Eigen; 

class Planner
{
private:

    planner_params params; 

    kdTreeNode tree;
    Dynamics   dynamic;
    
    kdNodePtr qNewPtr; 
    kdNodePtr qNearestPtr;

    Node q_new;
    Node q_goal;  
    Node q_origin; 
    double steering_max; 
    double steering_inc; 

    Node random_point();
    bool steerForRewire(const kdNodePtr& p1, const kdNodePtr& p2); 
    void rewire(vector<kdNodePtr>& nearby_nodes);
    void revise_nearest(const vector<kdNodePtr>& nearby_nodes);
    bool goal_prox(); 


    #ifdef VISUALIZATION
    Visualizer visualizer; 
    #endif

public:

    Planner(const planner_params& params_in); 
    
    void steer(); 
    void RRTstar(); 
    void print(); 
    // friend bool collision_check(Node qa, Node qb, MatrixXd obstacle); 

};

default_random_engine generator(time(0));
uniform_real_distribution<double> distribution(0,1);
