#ifndef planner_h
#define planner_h

#include "dynamics.hpp"
#include "common.hpp"

//---------inlcude only for code evaluation---------//
#include <ctime>
#include <fstream>
// Uncomment for visualization 
#define VISUALIZATION 
#ifdef VISUALIZATION
#include "visualizer.hpp"
#endif
//--------------------------------------------------//

using namespace std;
using namespace Eigen;



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
//     Point GetCoord(){
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
private:

    planner_params params;
    Node q_new;
    Node q_nearest;
    Node q_goal;
    Node q_origin;
    vector<Node> node_list;
    Node RandomPoint();
    Node NearestPoint();
    bool SteerForRewire(const Node& q1, Node& q2);
    void Rewire(vector<Node> nearby_nodes);
    void ReviseNearest(const vector<Node>& nearby_nodes);
    vector<Node> Nearby();
    vector<Node> GoalPath();
    bool GoalProx(Node q_new);
    
    #ifdef VISUALIZATION
    Visualizer visualizer; 
    #endif

public:
    
    vector<Node> path_goal; 
    Node Steer();
    Planner(planner_params params);
    vector<Node> RRTstar();
    friend bool CollisionCheck(Node qa,Node qb);
    // virtual Node new_state(Node q_old, double input, double time){};

};

default_random_engine generator(time(0));
uniform_real_distribution<double> distribution(0,1);

int Orientation(Point p,Point q,Point r);
bool OnSegment(Point p, Point q, Point r) ;
bool CollisionCheck(Node qa,Node qb,MatrixXd obstacle);
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
//     State RandomPoint(void);

//     public:
//     planner(Planner_params params);
// };
#endif