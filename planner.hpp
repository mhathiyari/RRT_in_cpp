#ifndef planner_h
#define planner_h
#include<Eigen/Dense>
#include<vector>
#include<random>
#include <math.h>
#include <limits>
#include "point.hpp"
#include"dynamics.hpp"


using namespace std;
using namespace Eigen;

class Planner :public Dynamics
{
    planner_params params;
    Node q_new;
    Node q_nearest;
    Node q_goal;
    vector<Node> node_list;
    Node random_point();
    Node nearest_pt();
    void rewire(vector<Node> nearby_nodes);
    void revise_nearest(vector<Node> nearby_nodes);
    vector<Node> nearby();
    Node steer();
    public:
    Planner(planner_params params);
    vector<Node> RRTstar();
    virtual Node new_state(Node q_old, double input, double time){};

};
// struct Planner_params
// {
//     Point origin;
//     Point goal;
//     Obstacle obstacle;
//     double iterations;
//     int width;
//     int height;
// };
// struct Node 
// {
//     State state;
//     double input;
//     double cost;
//     Point parent;
// };
// class planner
// {
//     Planner_params params;
//     Node q_new;
//     Node q_nearest;
//     vector<Node> node_list;
//     State random_point(void);

//     public:
//     planner(Planner_params params);
// };
#endif