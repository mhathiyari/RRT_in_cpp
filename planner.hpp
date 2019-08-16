#ifndef planner_h
#define planner_h

#include "dynamics.hpp"


using namespace std;
using namespace Eigen;

typedef struct Planner_params
{
    Point origin;
    Point goal;
    MatrixXd obstacle;
    double iterations;
    int width;
    int height;
}planner_params;

// typedef struct node 
// {
//     double x; //Need to make a state struct do can use it for any dynamics
//     double y;
//     double theta;
//     double vy;
//     double theta_dot;
//     double input;
//     double cost;
//     Point parent;
//     Point getcoord(){
//         Point A(this->x,this->y);
//         return A;
//     }
//     bool operator==(const node& A) const{
//     return (x==A.x&&y==A.y);}
//     bool operator!=(const node& A) const{
//     return (x!=A.x&&y!=A.y);}
//     void operator=(const node& A) {
//     x = A.x;
//     y = A.y;
//     theta = A.theta;
//     vy = A.vy;
//     theta_dot = A.theta_dot;
//     input = A.input;
//     cost = A.cost;
//     parent = A.parent;
//     }
// }Node;

class Planner 
{
    planner_params params;
    Node q_new;
    Node q_nearest;
    Node q_goal;
    Node q_origin;
    vector<Node> node_list;
    Node random_point();
    Node nearest_pt();
    void rewire(vector<Node> nearby_nodes);
    void revise_nearest(vector<Node> nearby_nodes);
    vector<Node> nearby();
    public:
    Node steer();
    Planner(planner_params params);
    vector<Node> RRTstar();

    // virtual Node new_state(Node q_old, double input, double time){};

};

default_random_engine generator;
uniform_real_distribution<double> distribution(0,1);
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