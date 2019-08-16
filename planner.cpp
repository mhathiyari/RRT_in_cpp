#include "planner.hpp"

using namespace std;
using namespace Eigen;

double euc_dist(Node q1, Node q2);

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
    Dynamics A;
    double best_angle = 0 ; 
    double steering_max = 1.02;
    double steering_inc = 0.17;
    double dist = numeric_limits<double>::infinity();
    double new_cost;
    for(double s = -steering_max ; s <= steering_max ;s += steering_inc)
    {
        q_f = A.new_state(q_nearest,s,5);     
        new_cost = euc_dist(q_f ,q_new);
        if(new_cost < dist){
            dist = new_cost;
            q_possible = q_f;
            q_possible.input = s; //maybe wont end up needing this
        }
    }
    q_new = q_possible;  
    return q_new;      
}
Node Planner::nearest_pt()
{
    double new_cost,cost_near;
    q_new.cost = euc_dist(q_new ,node_list[0]);
    q_nearest = node_list[0];
    cost_near = node_list[0].cost; // for cost of the node you are connecting to 

    for(auto i : node_list){
     new_cost = euc_dist(q_new ,i);

     if(new_cost < q_new.cost){
         q_new.cost = new_cost;
         cost_near = i.cost;
         q_nearest = i;
     }
    }
    q_new.cost += cost_near;
    return q_nearest;
}
Node Planner::random_point()
{   
    q_new.x = params.width*distribution(generator)+(0-params.width/2);
    q_new.y= params.height*distribution(generator)+(0-params.height/2);
    q_new.theta =  2*M_PI*distribution(generator);
    q_new.vy = 0;
    q_new.theta_dot = 0;
    q_new.input = 0;
    q_new.cost = 0;
    q_new.parent.x = q_origin.x;
    q_new.parent.y = q_origin.y;
    return q_new;

}

void Planner::revise_nearest(vector<Node> nearby_nodes){
    double new_cost;
    for (auto i : nearby_nodes){
        new_cost = i.cost + euc_dist(q_new,i);
        if (new_cost < q_new.cost){
            q_new.parent = i.parent;
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
            for (auto j : node_list){
                if (j == i){ //%& collision_check(q_new.coord,near_nodes(i).coord,obstacle)
                    j.parent = i.parent;
                    i.cost = i.cost;
                }
            }
        }
    }
}

double euc_dist(Node q1, Node q2){
    return (sqrt(pow((q1.x-q2.x),2)+pow((q1.y-q2.y),2)));
}

vector<Node> Planner::nearby() {

    vector<Node> near_nodes;
    double dist;
    int r = 500;
    for (auto i : node_list ){
    // %      if nodes(i).coord == q_new.parent; continue; end
        dist = euc_dist(q_new,i);
        if (dist < r )//&& collision_check(q_new.coord,nodes(i).coord,obstacle)
            {near_nodes.push_back(i);}
    }
    return near_nodes;
}

vector<Node> Planner::RRTstar()
{
    vector<Node> nearby_nodes;
    for (int i = 1;i < params.iterations ; i++)
    {
        q_new = random_point();

        cout<<"R "<<q_new.x<<" "<<q_new.y<<endl;
        q_nearest = nearest_pt();
        
        cout<<"N "<<q_nearest.x<<" "<<q_nearest.y<<endl;
        q_new = steer();
        cout<<"S "<<q_new.x<<" "<<q_new.y<<endl;
    
        if (euc_dist(q_new,q_nearest) < 1000) //collision_check(q_new,q_nearest) && 
        {
            q_new.parent = q_nearest.getcoord();

            //nearby_nodes = nearby();

            //revise_nearest(nearby_nodes);

            node_list.push_back(q_new);

            //rewire(nearby_nodes);
        }
    }
    return node_list;
}


int main ()
{
    //Driver Code
    planner_params A;
    A.origin = Point(200,200);
    A.goal = Point(-400,-400);
    MatrixXd obstacle(4,4);
    obstacle.row(0) << 1,1,-1,1;
    obstacle.row(1) << 1,1,-1,1;
    obstacle.row(2) << 1,1,-1,1;
    obstacle.row(3) << 1,1,-1,1;
    // obstacle.block(1,0,1,3) = {-1,1,-1,-1};
    // obstacle.block(2,0,2,3) = {-1,-1,1,-1};
    // obstacle.block(3,0,3,3) = {1,-1,1,1};
    A.obstacle = obstacle;
    A.iterations = 800;
    A.width = 1000;
    A.height = 1000;
    Planner Ab(A);
    vector<Node>node_list = Ab.RRTstar();
    for (auto i : node_list)
    {
        cout<<i.x<<" "<<i.y<<endl;
    }

    return 0;
}