#ifndef dynamics_h
#define dynamics_h

// #include "planner.hpp"
#include "point.hpp"
#include<Eigen/Dense>
#include<vector>
#include<random>
#include <math.h>
#include <limits>
using namespace std;
using namespace Eigen;



// typedef struct Planner_params
// {
//     Point origin;
//     Point goal;
//     MatrixXd obstacle;
//     double iterations;
//     int width;
//     int height;
// }planner_params;

typedef struct node 
{
    double x; //Need to make a state struct do can use it for any dynamics
    double y;
    double theta;
    double vy;
    double theta_dot;
    double input;
    double cost;
    Point parent;
    Point getcoord(){
        Point A(this->x,this->y);
        return A;
    }
    bool operator==(const node& A) const{
    return (x==A.x&&y==A.y);}
    bool operator!=(const node& A) const{
    return (x!=A.x||y!=A.y);}
    void operator=(const node& A) {
    x = A.x;
    y = A.y;
    theta = A.theta;
    vy = A.vy;
    theta_dot = A.theta_dot;
    input = A.input;
    cost = A.cost;
    parent = A.parent;
    }
}Node;

class Dynamics 
{
Matrix<double,5,1> dynamics( Matrix<double,5,1> state ,double s);

public:
Dynamics(){ };
Node new_state(Node q_old, double input, double time);//
friend class Planner;
};
#endif