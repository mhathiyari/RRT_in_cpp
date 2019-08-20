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

typedef struct states
{
    double x; 
    double y;
    double theta;
    double vy;
    double theta_dot;
    bool operator==(const states& A) const{
    return (x==A.x&&y==A.y);}
    bool operator!=(const states& A) const{
    return (x!=A.x||y!=A.y);}
    void operator=(const states& A) {
    x = A.x;
    y = A.y;
    theta = A.theta;
    vy = A.vy;
    theta_dot = A.theta_dot;}
    void setcoord(Point& A){
    x = A.x;
    y = A.y;
    theta = 0;
    vy = 0;
    theta_dot = 0;}
    states random_state(const double& Random){
        theta =  2*M_PI*Random;
        vy = 0;
        theta_dot = 0;
    }
    double cost(const states& q2){
    return (sqrt(pow((x-q2.x),2)+pow((y-q2.y),2)));
    }
}States;

class Dynamics 
{
Matrix<double,5,1> dynamics(Matrix<double,5,1> state ,double s);

public:
Dynamics(){ };
States new_state(States q_old, double input, double time);//
friend class Planner;
};
#endif