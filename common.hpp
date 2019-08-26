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
    int goalProx;
}planner_params;

typedef struct states
{
    double x; 
    double y;
    double theta;
    double vy;
    double theta_dot;
    bool operator==(const states& A) const{
      return (x==A.x&&y==A.y);
    }
    bool operator!=(const states& A) const{
      return (x!=A.x||y!=A.y);
    }
    void operator=(const states& A) {
      x = A.x;
      y = A.y;
      theta = A.theta;
      vy = A.vy;
      theta_dot = A.theta_dot;
    }
    void setcoord(Point& A){
      x = A.x;
      y = A.y;
      theta = 0;
      vy = 0;
      theta_dot = 0;
    }
    void random_state(const double& Random){
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
      return (state==A.state);
    }
    bool operator!=(const node& A) const{
      return (state!=A.state);
      }
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

inline int orientation(Point p,Point q,Point r){

    float val = (q.y - p.y) * (r.x - q.x) - 
            (q.x - p.x) * (r.y - q.y); 

    if (val == 0) return 0;  // colinear 

    return (val > 0)? 1: 2;
}

inline bool onsegment(Point p, Point q, Point r) { 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
       return true; 
    return false; 
} 
  
inline bool collision_check(Node qa, Node qb, Eigen::MatrixXd obstacle){
    int o1,o2,o3,o4;
    bool safe = true;
    for (int i=0;i<obstacle.rows();i++){
        Point p1(obstacle(i,0),obstacle(i,1));
        Point q1(obstacle(i,2),obstacle(i,3));

        o1 = orientation(qb.getcoord(),qa.getcoord(),p1);
        o2 = orientation(qb.getcoord(),qa.getcoord(),q1);
        o3 = orientation(p1,q1,qb.getcoord());     
        o4 = orientation(p1,q1,qa.getcoord());

        if (o1 != o2 && o3 != o4)
            return !safe ;
        if (o1 == 0 && onsegment(qb.getcoord(),p1,qa.getcoord()))
            return !safe ;

        if (o2 == 0 && onsegment(qb.getcoord(),q1,qa.getcoord()))
            return !safe ;

        if (o3 == 0 && onsegment(p1,qb.getcoord(),q1))
            return !safe ;

        if (o4 == 0 && onsegment(p1,qa.getcoord(),q1))
            return !safe ;
    }
    return safe;
} 

#endif 