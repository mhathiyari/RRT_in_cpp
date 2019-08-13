#include <iostream>
#include <math.h>
#include <limits>
using namespace std;

struct Point{
  double x_;
  double y_;
  Point(const double x,const double y){
    x_ = x;
    y_ = y;
  }

  void print() const{
    cout << "X, Y: " << x_ << ", " << y_ << endl;
  }
 
  Point operator-(const Point& A) const{
    return Point(x_-A.x_, y_-A.y_);
  }
 
  Point operator+(const Point& A) const{
    return Point(x_+A.x_, y_+A.y_);
  }
 
  Point operator*(const double a) const{
    return Point(x_*a, y_*a);
  }
 
};
