#ifndef dynamics_h
#define dynamics_h

// #include "planner.hpp"
#include "common.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <random>
#include <math.h>
#include <limits>
// #include "point.hpp"

using namespace std;
using namespace Eigen;



class Dynamics 
{
Matrix<double,5,1> dynamics(Matrix<double,5,1> state ,double s);

public:
Dynamics(){ };
States new_state(States q_old, double input, double time);//
friend class Planner;
};

#endif