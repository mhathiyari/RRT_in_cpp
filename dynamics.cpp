#include "dynamics.hpp"
// #include "planner.hpp"
// variables comment
States Dynamics::new_state(States q_old, double input, double time){
 //  RK4
    double dt = 0.01; // add check min dt or time
    Matrix<double,5,1> state,k1,k2,k3,k4;
    state(0,0) = q_old.x;
    state(1,0) = q_old.y;
    state(2,0) = q_old.theta;
    state(3,0) = q_old.v;
    state(4,0) = q_old.theta_dot;

    for (double i = 0;i<=time; i+=dt){
    k1  = dynamics(state, input);
    k2 = dynamics(state +k1/2, input);
    k3 = dynamics(state +k2/2, input);
    k4 = dynamics(state +k3 , input);
    
    state  = state  + dt/6*(k1 +2*k2 +2*k3 +k4 );
    }
    double theta = state(2,0);
    theta = fmod(theta,2*3.14);
    if (theta < 0)
    theta += 2*3.14;

    q_old.x = state(0,0);
    q_old.y = state(1,0);
    q_old.theta = theta;
    q_old.v = state(3,0);
    q_old.theta_dot =  state(4,0);
    return q_old;
}

Matrix<double,5,1> Dynamics::dynamics(Matrix<double,5,1> state ,double u)
{
double mass = 760; // TODO make a seprate place to define constants and see what are better ways to declare
double lf = 1.025; // make it static
double lr = 0.787;
double inertia = 1490.3;
double cf = 5146/2;
double cr = 3430/2;
double speed = 20.1;
  

double theta = state(2,0);

theta = fmod(theta,2*3.14);
if (theta < 0)
    theta += 2*3.14;


double vy = state(3,0);
double r = state(4,0); //theta_dot
 
double cosInput = cos(u);
double cosTheta = cos(theta);
double sinTheta = sin(theta);

double a = -(cf*cosInput+cr)/(mass*speed);
double b = (-lf*cf*cosInput+lr*cr)/(mass*speed)-speed;
double c = (-lf*cf*cosInput+lr*cr)/(inertia*speed);
double d = -(lf*lf*cf*cosInput+lr*lr*cr)/(inertia*speed);
double e = cf*cosInput/mass;
double f = lf*cf*cosInput/inertia;

double vyDot = a*vy + c*r + e*u;
double rDot = b*vy + d*r + f*u;
double xDot = speed*cosTheta - vy*sinTheta;
double yDot = speed*sinTheta + vy*cosTheta;
double thetaDot = r;
 
Matrix<double,5,1> x_dot;
x_dot(0,0) = xDot;
x_dot(1,0) = yDot;
x_dot(2,0) = thetaDot;
x_dot(3,0) = vyDot;
x_dot(4,0) = rDot; //theta_dot_dot
 
return x_dot;
}