#ifndef __COMMON_H__
#define __COMMON_H__

#include <iostream> 
#include <eigen3/Eigen/Dense>

using namespace std; 

struct Point{
  double x;
  double y;
  Point(const double x_,const double y_){
    x = x_;
    y = y_;
  }
   Point(){
    x = 0;
    y = 0;
  }

  void print() const{
    cout << "X, Y: " << x << ", " << y << endl;
  }
 
  Point operator-(const Point& A) const{
    return Point(x-A.x, y-A.y);
  }
 
  Point operator+(const Point& A) const{
    return Point(x+A.x, y+A.y);
  }
 
  Point operator*(const double a) const{
    return Point(x*a, y*a);
  }
  bool operator==(const Point& A) const{
   return (x == A.x && y == A.y);
  }
   
  void operator=(const Point& A) {
   x = A.x;
   y = A.y;
  }
};

typedef struct Planner_params
{
    Point origin;
    Point goal;
    Eigen::MatrixXd obstacle;
    double iterations;
    int width;
    int height;
    int goalProx = 10;
}planner_params;

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

typedef struct node 
{
    States state;
    double input;
    double cost;
    Point parent;

    Point getcoord(){
        Point A(this->state.x,this->state.y);
        return A;
    }
    bool operator==(const node& A) const{
    return (state==A.state);}
    bool operator!=(const node& A) const{
    return (state!=A.state);}
    void operator=(const node& A) {
    state = A.state;
    input = A.input;
    cost = A.cost;
    parent = A.parent;
    }
    void setcoord(Point& A){
    this->state.setcoord(A);
    }
}Node;

inline void tfXy2Pixel(double& x, double& y, const int& width, const int& height)
{
  x += width/2; 
  y = height/2 - y; 
}

inline double calDistNode(const Node& n1, const Node& n2)
{
  return sqrt(pow(n1.state.x-n2.state.x,2) + pow(n1.state.y-n2.state.y,2));
}


#endif 