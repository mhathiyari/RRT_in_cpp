#include "planner.hpp"

using namespace std;
using namespace Eigen;

double euc_dist(Node q1, Node q2);

Planner::Planner(planner_params params_in)
{ 
    params = params_in;
    Node q_origin;
    q_origin.SetCoord(params.origin);
    q_origin.input = 0;
    q_origin.cost = 0;
    q_origin.parent.x = params.origin.x;
    q_origin.parent.y = params.origin.y;
    q_goal.SetCoord(params.goal);
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
    for(float s = -steering_max ; s <= steering_max ;s += steering_inc){
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

bool Planner::SteerForRewire(const Node& q1, Node& q2)
{
    float steering_max = 1.02, steering_inc = steering_max/21;
    double x_eps = 0.3, y_eps = 0.3;
    Dynamics A; 
    for(float s = -steering_max; s <= steering_max; s += steering_inc)
    {
        States st = A.new_state(q1.state, s, 0.5); 
        if(abs(st.x-q2.state.x) < x_eps && abs(st.y-q2.state.y) < y_eps)
        {
            std::cout << "q1 vel: " << q1.state.vy << std::endl;   
            std::cout << "q1: [" << q1.state.x << "," << q1.state.y << "], ";
            std::cout << "q2: [" << q2.state.x << "," << q2.state.y << "], ";
            std::cout << "st: [" << st.x << "," << st.y << "], " << std::endl;

            q2.state = st;
            return true;
        }
    }
    return false;
}

Node Planner::NearestPoint()
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
Node Planner::RandomPoint()
{   
    q_new.state.x = params.width*distribution(generator)+(0-params.width/2);
    q_new.state.y= params.height*distribution(generator)+(0-params.height/2);
    q_new.state.RandomState(distribution(generator));
    q_new.input = 0;
    q_new.cost = 0;
    q_new.parent.x = q_origin.state.x;
    q_new.parent.y = q_origin.state.y;
    return q_new;
}

void Planner::ReviseNearest(const vector<Node>& nearby_nodes){
    double new_cost;
    for (auto i : nearby_nodes){
        new_cost = i.cost + euc_dist(q_new,i);
        if (new_cost < q_new.cost && SteerForRewire(i, q_new)){
            std::cout << "revise nearest" << std::endl;
            q_new.parent = i.GetCoord();
            q_new.cost = new_cost;
            }
        }
}

void Planner::Rewire(vector<Node> nearby_nodes){
    double temp_cost;
     for (auto& i : nearby_nodes){
         if (i == q_nearest) continue ;
         temp_cost = (q_new.cost + euc_dist(q_new,i));
         if (i.cost > temp_cost && SteerForRewire(q_new, i)){
            std::cout << "rewire" << std::endl;
            //  if CollisionCheck(q_new,i,obstacle)
            i.parent.x = q_new.state.x;
            i.parent.y = q_new.state.y;
            i.cost = temp_cost;
         }
     }
//  % Modifying the new nearby nodes in the actual node list
    for (auto i: nearby_nodes){
        if ((i.parent.x == q_new.state.x && i.parent.y == q_new.state.y)  && (i != q_nearest)){
            for (auto& j : node_list){
                if (j == i){ //%& CollisionCheck(q_new.coord,near_nodes(i).coord,obstacle)
                    j.parent = i.parent;
                    j.cost = i.cost;
                }
            }
        }
    }
}

double euc_dist(Node q1, Node q2){
    return (q1.state.Cost(q2.state));
}

vector<Node> Planner::nearby() {

    vector<Node> near_nodes;
    double dist;
    int r = 50;
    for (auto i : node_list ){
    // %      if nodes(i).coord == q_new.parent; continue;  
        dist = euc_dist(q_new,i);
        if (dist < r )//&& CollisionCheck(q_new.coord,nodes(i).coord,obstacle)
            {near_nodes.push_back(i);}
    }
    return near_nodes;
}


int Orientation(Point p,Point q,Point r){

    float val = (q.y - p.y) * (r.x - q.x) - 
            (q.x - p.x) * (r.y - q.y); 

    if (val == 0) return 0;  // colinear 

    return (val > 0)? 1: 2;
}

bool OnSegment(Point p, Point q, Point r) { 
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && 
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y)) 
       return true; 
  
    return false; 
} 
  


bool CollisionCheck(Node qa,Node qb,MatrixXd obstacle){
    int o1,o2,o3,o4;
    bool safe = true;
    for (int i=0;i<obstacle.rows();i++){
        Point p1(obstacle(i,0),obstacle(i,1));
        Point q1(obstacle(i,2),obstacle(i,3));

        o1 = Orientation(qb.GetCoord(),qa.GetCoord(),p1);
        o2 = Orientation(qb.GetCoord(),qa.GetCoord(),q1);
        o3 = Orientation(p1,q1,qb.GetCoord());     
        o4 = Orientation(p1,q1,qa.GetCoord());

        if (o1 != o2 && o3 != o4)
            return !safe ;
        if (o1 == 0 && OnSegment(qb.GetCoord(),p1,qa.GetCoord()))
            return !safe ;

        if (o2 == 0 && OnSegment(qb.GetCoord(),q1,qa.GetCoord()))
            return !safe ;

        if (o3 == 0 && OnSegment(p1,qb.GetCoord(),q1))
            return !safe ;

        if (o4 == 0 && OnSegment(p1,qa.GetCoord(),q1))
            return !safe ;
    }
    return safe;
}    
  
bool Planner::goal_prox(Node q_new){
bool prox = false;
double dist = euc_dist(q_new,q_goal);
// cout<<dist<<endl;
if (dist < params.goalProx)
    prox = true;
return prox;
}


// need to implement a better way to search the shortest path using a very naive approach right now
vector<Node> Planner::goal_path(){
    auto size_n = node_list.size();
    q_goal.parent = node_list[size_n-1].GetCoord();
    vector<Node> path;
    path.push_back(q_goal);
    Node q_next = q_goal;
    while(!(q_next.GetCoord() == node_list[0].GetCoord())){
        for (auto i : node_list){
            if (i.GetCoord() == path.back().parent){
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
        q_new = RandomPoint();

        // cout<<"R "<<q_new.x<<" "<<q_new.y<<endl;
        q_nearest = NearestPoint();
        
        // cout<<"N "<<q_nearest.x<<" "<<q_nearest.y<<endl;
        q_new =  ();
        // cout<<"S "<<q_new.x<<" "<<q_new.y<<endl;

    
        if (euc_dist(q_new,q_nearest) < 1000 && CollisionCheck(q_new,q_nearest,params.obstacle)) 
        {
            q_new.parent = q_nearest.GetCoord();

            nearby_nodes = nearby();

            // ReviseNearest(nearby_nodes);

            node_list.push_back(q_new);

            Rewire(nearby_nodes);
        }
        if(goal_prox(q_new)){

            path_goal = goal_path();

            #ifdef VISUALIZATION
            visualizer.drawMapwGoalPath(params, node_list, path_goal);
            #endif

            cout<<"Number of iteration : "<<i<<endl;
            break;
            }

        #ifdef VISUALIZATION
        visualizer.drawMap(params, node_list, q_goal); 
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
    A.goal = Point(-150, -150);
    MatrixXd obstacle(4,4);
    obstacle.row(0) << 1,1,-1,1;
    obstacle.row(1) << -1,1,-1,-1;
    obstacle.row(2) << -1,-1,1,-1;
    obstacle.row(3) << 1,-1,1,1;

    A.obstacle = obstacle*100;
    A.iterations = 8000;
    A.width = 1000;
    A.height = 1000;
    A.goalProx = 30;
    Planner Ab(A);
    vector<Node>node_list = Ab.RRTstar();
    
    //---------inlcude only for code evaluation---------//
    /*
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
    */

    cout<<"Time "<<(clock()-start_time)/CLOCKS_PER_SEC<<" s"<<endl;
    char n; 
    cin >> n; 
    //---------inlcude only for code evaluation---------//
    return 0;
}