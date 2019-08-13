#include<Eigen/Dense>
#include<math.h>
#include<iostream>
#include "planner.hpp"
using namespace Eigen;
using namespace std;
class Dynamics
{
Matrix<float,5,1> dynamics( Matrix<float,5,1> state ,float s);

public:
void new_state(Node q_old, float input, float time);
};
void Dynamics::new_state(Node q_old, float input, float time){
 //  RK4
 float dt = 0.01;
 Matrix<float,5,1> state,k1,k2,k3,k4;
 state(0,0) = q_old.x;
 state(1,0) = q_old.y;
 state(2,0) = q_old.theta;
 state(3,0) = q_old.vy;
 state(4,0) = q_old.r;


 for (float i = 0,i<=dt; i+=0.5){
 k1  = dynamics(state ,s);
 k2 = dynamics(state +k1 /2,s);
 k3 = dynamics(state +k2 ./2,s);
 k4 = dynamics(state +k3 ,s);
 
 state  = state  + dt/6*(k1 +2*k2 +2*k3 +k4 );
 }
// Old code for generating point list for the path==========
//  point_list(point_indx,:) = q_old ;
//  point_indx = 1 + point_indx;
//  end
//  q_f  = q_old ;
}

Matrix<float,5,1> Dynamics::dynamics( Matrix<float,5,1> state ,float s)
{
float mass = 760; // TODO make a seprate place to define constants and see what are better ways to declare
float lf = 1.025;
float lr = 0.787;
float inertia = 1490.3;
float cf = 5146/2;
float cr = 3430/2;
float speed = 10.1;
  

float theta = state(3,0);
float vy = state(4,0);
float r = state(5,0);
 
float cosInput = cos(u);
float cosTheta = cos(theta);
float sinTheta = sin(theta);

float a = -(cf*cosInput+cr)/(mass*speed);
float b = (-lf*cf*cosInput+lr*cr)/(mass*speed)-speed;
float c = (-lf*cf*cosInput+lr*cr)/(inertia*speed);
float d = -(lf*lf*cf*cosInput+lr*lr*cr)/(inertia*speed);
float e = cf*cosInput/mass;
float f = lf*cf*cosInput/inertia;

float vyDot = a*vy + c*r + e*u;
float rDot = b*vy + d*r + f*u;
float xDot = speed*cosTheta - vy*sinTheta;
float yDot = speed*sinTheta + vy*cosTheta;
float thetaDot = r;
 
Matrix<float,5,1> x_dot;
x_dot(0,0) = xDot;
x_dot(1,0) = yDot;
x_dot(2,0) = thetaDot;
x_dot(3,0) = vyDot;
x_dot(4,0) = rDot;
 
return x_dot;
}