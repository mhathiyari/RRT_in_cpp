#include <iostream>
#include <math.h>
#include <limits>
#include <points.hpp>

struct Planner_params
{
    Point origin;
    Point goal;
    Obstacle obstacle;
    double iterations;
    int width;
    int height;
}
struct Node 
{
    State state;
    float input;
    float cost;
    Point parent;
}
class planner
{
    Planner_params params;
    Node q_new;
    Node q_nearest;
    vector<Node> node_list;
    State random_point(void)

    public:
    planner(Planner_params params);
};