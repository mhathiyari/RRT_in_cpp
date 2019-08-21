#include "planner.hpp"

using namespace std;
using namespace Eigen;

double euc_dist(Node q1, Node q2);

Planner::Planner(planner_params params_in)
{ 
    params = params_in;
    Node q_origin;
    q_origin.setcoord(params.origin);
    q_origin.input = 0;
    q_origin.cost = 0;
    q_origin.parent.x = params.origin.x;
    q_origin.parent.y = params.origin.y;
    q_goal.setcoord(params.goal);
    q_goal.input = 0;
    q_goal.cost = 0;
    q_goal.parent.x = Infinity;
    q_goal.parent.y = Infinity;
    node_list.push_back(q_origin);
    
    #ifdef VISUALIZATION
    visualizer.plannerParamsIn(params); 
    #endif
}


Node Planner::steer()
{

    Node q_f,q_possible;
    Dynamics A;
    double best_angle = 0 ; 
    float steering_max = 1.02;
    float steering_inc = steering_max/21;
    double dist = numeric_limits<double>::infinity();
    double new_cost;
    for(float s = -steering_max ; s <= steering_max ;s += steering_inc)
    {
        if (abs(s)<steering_inc)
            s = 0;
        q_f.state = A.new_state(q_nearest.state,s,0.5);     
        new_cost = euc_dist(q_f ,q_new);
        if(new_cost < dist){
            dist = new_cost;
            q_possible = q_f;
            q_possible.input = s; //maybe wont   up needing this
            q_possible.cost = new_cost; //maybe wont   up needing this
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
    q_new.state.x = params.width*distribution(generator)+(0-params.width/2);
    q_new.state.y= params.height*distribution(generator)+(0-params.height/2);
    q_new.state.random_state(distribution(generator));
    q_new.input = 0;
    q_new.cost = 0;
    q_new.parent.x = q_origin.state.x;
    q_new.parent.y = q_origin.state.y;
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

// void Planner::rewire(vector<Node> nearby_nodes){
//     double temp_cost;
//      for (auto i : nearby_nodes){
//          if (i == q_nearest) continue ;
//          temp_cost = (q_new.cost + euc_dist(q_new,i));
//          if (i.cost > temp_cost){
//             //  if collision_check(q_new,i,obstacle)
//                  i.parent.x = q_new.x;
//                  i.parent.y = q_new.y;
//                  i.cost = temp_cost;
//          }
//      }
// //  % Modifying the new nearby nodes in the actual node list
//     for (auto i: nearby_nodes){
//         if ((i.parent.x == q_new.x && i.parent.y == q_new.y)  && (i != q_nearest)){
//             for (auto j : node_list){
//                 if (j == i){ //%& collision_check(q_new.coord,near_nodes(i).coord,obstacle)
//                     j.parent = i.parent;
//                     i.cost = i.cost;
//                 }
//             }
//         }
//     }
// }

double euc_dist(Node q1, Node q2){
    return (q1.state.cost(q2.state));
}

vector<Node> Planner::nearby() {

    vector<Node> near_nodes;
    double dist;
    int r = 500;
    for (auto i : node_list ){
    // %      if nodes(i).coord == q_new.parent; continue;  
        dist = euc_dist(q_new,i);
        if (dist < r )//&& collision_check(q_new.coord,nodes(i).coord,obstacle)
            {near_nodes.push_back(i);}
    }
    return near_nodes;
}


int orientation(Point p,Point q,Point r){

    float val = (q.y - p.y) * (r.x - q.x) - 
            (q.x - p.x) * (r.y - q.y); 

    if (val == 0) return 0;  // colinear 

    return (val > 0)? 1: 2;
}

bool onsegment(Point p, Point q, Point r) { 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
       return true; 
  
    return false; 
} 
  


bool collision_check(Node qa,Node qb,MatrixXd obstacle){
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
  
bool Planner::goal_prox(Node q_new){
bool prox = false;
double dist = euc_dist(q_new,q_goal);
// cout<<dist<<endl;
if (dist < 10)
    prox = true;
return prox;
}


// need to implement a better way to search the shortest path using a very naive approach right now
vector<Node> Planner::goal_path(){
    auto size_n = node_list.size();
    q_goal.parent = node_list[size_n-1].getcoord();
    vector<Node> path;
    path.push_back(q_goal);
    Node q_next = q_goal;
    while(!(q_next.getcoord() == node_list[0].getcoord())){
        for (auto i : node_list){
            if (i.getcoord() == path.back().parent){
                    q_next = i;
                    break;
            }
        }
        path.push_back(q_next);
    }
    reverse(path.begin(),path.end());
    return path;
}
    

vector<Node> Planner::RRTstar()
{
    vector<Node> nearby_nodes;
    for (int i = 0;i < params.iterations ; i++)
    {
        q_new = random_point();

        // cout<<"R "<<q_new.x<<" "<<q_new.y<<endl;
        q_nearest = nearest_pt();
        
        // cout<<"N "<<q_nearest.x<<" "<<q_nearest.y<<endl;
        q_new = steer();
        // cout<<"S "<<q_new.x<<" "<<q_new.y<<endl;

    
        if (euc_dist(q_new,q_nearest) < 1000 && collision_check(q_new,q_nearest,params.obstacle)) 
        {
            q_new.parent = q_nearest.getcoord();

            // nearby_nodes = nearby();

            //revise_nearest(nearby_nodes);

            node_list.push_back(q_new);

            //rewire(nearby_nodes);
        }
        if(goal_prox(q_new)){
            path_goal = goal_path();
            cout<<"Number of iteration : "<<i<<endl;
            break;
            }

        #ifdef VISUALIZATION
        visualizer.drawMap(params, node_list); 
        #endif 

        cout << "step: " << i << endl; 
    }
    return node_list;
}


int main ()
{   
    clock_t start_time = clock();
    //Driver Code
    planner_params A;
    A.origin = Point(200,200);
    A.goal = Point(-400,-400);
    MatrixXd obstacle(4,4);
    obstacle.row(0) << 1,1,-1,1;
    obstacle.row(1) << -1,1,-1,-1;
    obstacle.row(2) << -1,-1,1,-1;
    obstacle.row(3) << 1,-1,1,1;

    A.obstacle = obstacle*100;
    A.iterations = 1000;
    A.width = 1000;
    A.height = 1000;
    Planner Ab(A);
    vector<Node>node_list = Ab.RRTstar();
    
    //---------inlcude only for code evaluation---------//

    std::ofstream nodelist;
    nodelist.open ("nodelist.csv");
    nodelist<<"x"<<","<<"y"<<","<<"theta"<<",";
    nodelist<<"vy"<<","<<"theta_dot"<<","<<"input"<<",";
    nodelist<<"cost"<<","<<"x"<<","<<".y"<<"\n";

    for (auto i : node_list)
    {
        if(i.state.x<100 && i.state.x>-100 && i.state.y<100 && i.state.y>-100){
            cout<<i.state.x<<" "<<i.state.y<<" obstacle"<<endl;
        }else{
            cout<<i.state.x<<" "<<i.state.y<<endl;
            nodelist<<i.state.x<<","<<i.state.y<<","<<i.state.theta<<",";
            nodelist<<i.state.vy<<","<<i.state.theta_dot<<","<<i.input<<",";
            nodelist<<i.cost<<","<<i.parent.x<<","<<i.parent.y<<"\n";
        }
    }
    nodelist.close();
    nodelist.open ("goal.csv");
    nodelist<<"x"<<","<<"y"<<","<<"theta"<<",";
    nodelist<<"vy"<<","<<"theta_dot"<<","<<"input"<<",";
    nodelist<<"cost"<<","<<"x"<<","<<".state.y"<<"\n";

    for (auto i : Ab.path_goal)
    {
        if(i.state.x<100 && i.state.x>-100 && i.state.y<100 && i.state.y>-100){
            cout<<i.state.x<<" "<<i.state.y<<" obstacle"<<endl;
        }else{
            cout<<i.state.x<<" "<<i.state.y<<endl;
            nodelist<<i.state.x<<","<<i.state.y<<","<<i.state.theta<<",";
            nodelist<<i.state.vy<<","<<i.state.theta_dot<<","<<i.input<<",";
            nodelist<<i.cost<<","<<i.parent.x<<","<<i.parent.y<<"\n";
        }
    }
    nodelist.close();


    cout<<"Time "<<(clock()-start_time)/CLOCKS_PER_SEC<<" s"<<endl;
    char n; 
    cin >> n; 
    //---------inlcude only for code evaluation---------//
    return 0;
}