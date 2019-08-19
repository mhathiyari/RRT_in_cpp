#ifndef dynamics_h
#define dynamics_h

// #include "planner.hpp"
#include "common.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <random>
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

class Dynamics 
{
Matrix<double,5,1> dynamics( Matrix<double,5,1> state ,double s);

public:
Dynamics(){ };
Node new_state(Node q_old, double input, double time);//
friend class Planner;
};

#endif