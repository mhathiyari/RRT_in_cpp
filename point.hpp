#include <iostream>
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
