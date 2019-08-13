#include "planner.hpp"
#include<random>
#include <iostream>
#include <math.h>
#include <limits>
#include <point.hpp>
#include<vector>

using namespace std;

struct planner_params
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
    float x; //Need to make a state struct do can use it for any dynamics
    float y;
    float theta;
    float vy;
    float theta_dot;
    float input;
    float cost;
    Point parent;
}
class Planner
{
    planner_params params;
    Node q_new;
    Node q_nearest;
    Node q_goal;
    vector<Node> node_list;
    Node random_point();
    Node nearest_pt();
    void rewire(vector<Node> nearby_nodes);

    public:
    Node steer();
    Planner(planner_params params);
    vector<Node> RRTstar();
};
Planner::Planner(planner_params params)
{ 
    params = params;
    Node q_origin;
    q_origin.x = params.origin.x;
    q_origin.y = params.origin.y;
    q_origin.theta = 0;
    q_origin.vy = 0;
    q_origin.theta_dot = 0;
    q_origin.input = 0;
    q_origin.cost = 0;
    q_origin.parent.x = params.origin.x;
    q_origin.parent.y = params.origin.y;
    node_list.push_back(q_origin);
}

vector<Node> Planner::RRTstar()
{
    for (int i = 1;i < params.iterations ; i++)
    {
        q_new = random_point();

        q_nearest = nearest_pt(q_new,node_list);

        q_new = steer(q_nearest,q_new);

        if collision_check(q_new,q_nearest) && distance(q_new,q_nearest) < 1000
        {
            q_new.parent = q_nearest.getcoord();

            nearby_nodes = nearby(q_new);

            q_new = revise_nearest();
            node_list.push_back(q_new);
            rewire();
        }

        return node_list
}

Node Planner::steer()
{

    Node q_f,q_possible;
    float best_angle = 0 ; 
    float steering_max = 1.02;
    float steering_inc = 0.17;
    float dist = numeric_limits<float>::infinity();
    for(float s = -steering_max ; s <= steering_max ; s =+ steering_inc)
    {
        q_f = new_state(q_nearest,s,0.5);     
        new_cost = euc_dist(q_f ,q_nearest);
        if(new_cost < dist){
            dist = new_cost;
            q_possible = q_f;
            q_possible.input = s; //maybe wont end up needing this
        }
    }
    q_new = q_possible;        
}
Node Planner::nearest_pt()
{
    float new_cost,cost_near;
    q_new.cost = euc_dist(q_new ,node_list[1]);
    q_nearest = node_list[1];
    cost_near = node_list[1].cost;

    for(auto i : node_list){
     new_cost = euc_dist(q_new ,node_list[i]);

     if(new_cost < q_new.cost){
         q_new.cost = new_cost;
         cost_near = node_list[i].cost;
         q_nearest = node_list[i];
     }
    }
    q_new.cost += cost_near;
}
Node Planner::random_point()
{   
    default_random_engine generator;
    uniform_int_distribution<int> distribution(0,1);
    float offset[2] = [0;0] - [params.width;params.height]/2;
    q_new.x = width*distribution(generator)+offset(1);
    y_rand = height*distribution(generator)+offset(2);
    p_rand = [x_rand,y_rand];

    return q_new;
}

void Planner::revise_nearest(){
 for (auto i : nearby_nodes){
    new_cost = i.cost + distance_euc(q_new,i);
    if (new_cost < q_new.cost){
        q_ = i;
        q_new.cost = new_cost;
        }
    }
}

void Planner::rewire(vector<Node> nearby_nodes){
    double temp_cost;
     for (auto i : nearby_nodes){
         if (i == q_nearest) continue ;
         temp_cost = (q_new.cost + euc_dist(q_new,i));
         if (i.cost > temp_cost){
            //  if collision_check(q_new,i,obstacle)
                 i.parent.x = q_new.x;
                 i.parent.y = q_new.y;
                 i.cost = temp_cost;
         }
     }
//  % Modifying the new nearby nodes in the actual node list
    for {auto i: nearby_nodes){
        if ((i.parent.x == q_new.x && i.parent.y == q_new.y)  && (i != q_nearest)){
            for (j : node_list){
                if (j == i){ //%& collision_check(q_new.coord,near_nodes(i).coord,obstacle)
                    j.parent = i.parent;
                    i.cost = i.cost;
                }
            }
        }
    }
}

int main ()
{
    

    return 0;
}