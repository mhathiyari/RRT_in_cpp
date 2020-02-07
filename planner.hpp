#pragma once

#include "kdTreeNode.hpp"
#include "dynamics.hpp"
#include "dubins.h"

// Uncomment for visualization 
#define VISUALIZATION
#ifdef VISUALIZATION
// #include "visualizer.hpp"
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

    PlannerParams params; 

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

    kdNodePtr findNearestLeastCost(std::vector<kdNodePtr> nearbyNodes);
    Node RandomPoint();                                            // without goal bais
    Node RandomPoint(int k);                                       // with Gb
    bool SteerForRewire(const kdNodePtr& p1, const kdNodePtr& p2); 
    bool dubinForRewire(const kdNodePtr& p1, const kdNodePtr& p2, DubinsPath* path); 
    bool collisionCheckDubins(DubinsPath* path); 
    void Rewire(vector<kdNodePtr>& nearby_nodes);
    void ReviseNearest(const vector<kdNodePtr>& nearby_nodes);
    bool GoalProx(); 
    bool GoalProxDubins();

    #ifdef VISUALIZATION
    Visualizer visualizer; 
    #endif

public:

    Planner(const PlannerParams& params_in); 
    
    void Steer(); 
    int  DubinsCurve(DubinsPath* path); 
    void RRTstar(); 
    void ExtractPath(Path& path);
    void print(); 
    
    // friend bool CollisionCheck(Node qa, Node qb, MatrixXd obstacle); 

};


