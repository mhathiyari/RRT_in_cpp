#include<Eigen/Dense>
#include<vector>
#include<random>
#include <math.h>
#include <limits>
#include <point.hpp>
using namespace std;
// struct Planner_params
// {
//     Point origin;
//     Point goal;
//     Obstacle obstacle;
//     double iterations;
//     int width;
//     int height;
// };
// struct Node 
// {
//     State state;
//     double input;
//     double cost;
//     Point parent;
// };
// class planner
// {
//     Planner_params params;
//     Node q_new;
//     Node q_nearest;
//     vector<Node> node_list;
//     State random_point(void);

//     public:
//     planner(Planner_params params);
// };