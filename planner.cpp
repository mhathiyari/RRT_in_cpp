#include "planner.hpp"
#include<Eigen/Dense>
using namespace std;
double euc_dist(Node q1, Node q2);
typedef struct Planner_params
{
    Point origin;
    Point goal;
    MatrixXd obstacle;
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
}Node;
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

    Node steer();
    public:
    Planner(planner_params params);
    vector<Node> RRTstar();
};
Planner::Planner(planner_params params_in)
{ 
    params = params_in;
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


Node Planner::steer()
{

    Node q_f,q_possible;
    double best_angle = 0 ; 
    double steering_max = 1.02;
    double steering_inc = 0.17;
    double dist = numeric_limits<double>::infinity();
    for(double s = -steering_max ; s <= steering_max ; s =+ steering_inc)
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
    double new_cost,cost_near;
    q_new.cost = euc_dist(q_new ,node_list[1]);
    q_nearest = node_list[1];
    cost_near = node_list[1].cost;

    for(auto i : node_list){
     new_cost = euc_dist(q_new ,i);

     if(new_cost < q_new.cost){
         q_new.cost = new_cost;
         cost_near = i.cost;
         q_nearest = i;
     }
    }
    q_new.cost += cost_near;
}
Node Planner::random_point()
{   
    default_random_engine generator;
    uniform_int_distribution<int> distribution(0,1);
    double offset[2] = [0;0] - [params.width;params.height]/2;
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
    for (auto i: nearby_nodes){
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

double euc_dist(Node q1, Node q2){
    return sqrt((q1.x-q2.x)^2+(q1.y-q2.y)^2);
}

vector<Node> Planner::RRTstar()
{
    vector<Node> nearby_nodes;
    for (int i = 1;i < params.iterations ; i++)
    {
        q_new = random_point();

        q_nearest = nearest_pt();

        q_new = steer();
    
        if (euc_dist(q_new,q_nearest) < 1000) //collision_check(q_new,q_nearest) && 
        {
            q_new.parent = q_nearest.getcoord();

            nearby_nodes = nearby();

            revise_nearest();
            node_list.push_back(q_new);
            rewire();
        }

        return node_list;
}

vector<Node> Planner::nearby(){

    vector<Node> near_nodes;
    r = 1;
    for (i : node_list ){
    // %      if nodes(i).coord == q_new.parent; continue; end
        dist = euc_dist(q_new,i);
        if (dist < r )//&& collision_check(q_new.coord,nodes(i).coord,obstacle)
            near_nodes.push_back(i);
    }

}
int main ()
{
    

    return 0;
}