#pragma once

#include "kdTreeNode.hpp"
#include "dynamics.hpp"
#include "dubins.h"

// Uncomment for visualization 
#define VISUALIZATION
#ifdef VISUALIZATION
#include "visualizer.hpp"
#endif
//

// Uncomment for Dubins Curve
#define DUBINSCURVE

#ifndef DUBINSCURVE
#define DYNAMICS
#endif

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
    kdNodePtr qGoalPtr;

    Node q_new;
    Node q_goal;  
    Node q_origin; 
    double steering_max; //static or some better way to declare
    double steering_inc; 
    double optimal_cost;
    double maximum_cost; 
    double maxDist; 

    static double distCoeff; 

    Node random_point();                                            // without goal bais
    Node random_point(int k);                                       // with Gb
    bool steerForRewire(const kdNodePtr& p1, const kdNodePtr& p2); 
    bool dubinForRewire(const kdNodePtr& p1, const kdNodePtr& p2, DubinsPath* path); 
    bool collisionCheckDubins(DubinsPath* path); 
    void rewire(vector<kdNodePtr>& nearby_nodes);
    void revise_nearest(const vector<kdNodePtr>& nearby_nodes);
    bool goal_prox(); 
    bool goalProxDubins();

    #ifdef VISUALIZATION
    Visualizer visualizer; 
    #endif

public:

    Planner(const planner_params& params_in); 
    
    void steer(); 
    void RRTstar(); 
    void print(); 
    int dubinsCurve(DubinsPath* path); 
    // friend bool collision_check(Node qa, Node qb, MatrixXd obstacle); 

};

default_random_engine generator(time(0));
uniform_real_distribution<double> distribution(0,1);
