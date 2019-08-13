#include<Eigen/Dense>
class Dynamics
{

}
void Dynamics::new_state(Nodeq_old, float input, float time){
 //  RK4
 float dt = 0.01;
 for (float i = 0,i<=dt; i+=0.5){
 k1  = dynamics(q_old ,s);
 k2 = dynamics(q_old +k1 /2,s);
 k3 = dynamics(q_old +k2 ./2,s);
 k4 = dynamics(q_old +k3 ,s);
 
 q_old  = q_old  + dt/6*(k1 +2*k2 +2*k3 +k4 );
 }
 point_list(point_indx,:) = q_old ;
 point_indx = 1 + point_indx;
 end
 q_f  = q_old ;
}

Node Dynamics::dynamics(Node x,float s)
{
mass = 760;
lf = 1.025;
lr = 0.787;
inertia = 1490.3;
cf = 5146/2;
cr = 3430/2;
speed = 10.1;
  
%  x =  x.coord(1);
%  y = x.coord(2);
 theta = x(3);
 vy = x(4);
 r = x(5);
 
 cosInput = cos(u);
 cosTheta = cos(theta);
 sinTheta = sin(theta);

 a = -(cf*cosInput+cr)/(mass*speed);
 b = (-lf*cf*cosInput+lr*cr)/(mass*speed)-speed;
 c = (-lf*cf*cosInput+lr*cr)/(inertia*speed);
 d = -(lf*lf*cf*cosInput+lr*lr*cr)/(inertia*speed);
 e = cf*cosInput/mass;
 f = lf*cf*cosInput/inertia;

 vyDot = a*vy + c*r + e*u;
 rDot = b*vy + d*r + f*u;
 xDot = speed*cosTheta - vy*sinTheta;
 yDot = speed*sinTheta + vy*cosTheta;
 thetaDot = r;
 
 
x_dot.coord(1) = xDot;
x_dot.coord(2) = yDot;
x_dot.coord(3) = thetaDot;
x_dot.coord(4) = vyDot;
x_dot.coord(5) = rDot;
 
 end
}