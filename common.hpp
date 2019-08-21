#ifndef __COMMON_H__
#define __COMMON_H__

#include <iostream> 
#include <Eigen/Dense>
#include "stdafx.h
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
}planner_params;

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


#endif 