#include "plannerTmp.hpp"

/*
* static
*/
double Planner::distCoeff = 300.0; 


/*
* public
*/

double EucDist(Node& q1, Node& q2)
{
    return q1.state.Cost(q2.state);
}

Planner::Planner(const planner_params& params_in)
{
    params       = params_in; 
    steering_max = 1.02; 
    steering_inc = steering_max/21;

    q_origin.SetCoord(params.origin);
    q_origin.input = 0; 
    q_origin.cost  = 0; 
    q_origin.parent.x = params.origin.x;//seet if parent gets used remove 
    q_origin.parent.y = params.origin.y; 

    q_goal.SetCoord(params.goal); 
    q_goal.input = 0; 
    q_goal.cost  = 0; 
    q_goal.parent.x = Infinity;// set to something meaningful or remove
    q_goal.parent.y = Infinity; 

    optimal_cost = q_origin.state.Cost(q_goal.state);
    maximum_cost = optimal_cost + 400; // avoiding devided by 0 10*opt +1
    maxDist      = 0; 
    tree.treeInit(q_origin);  

    #ifdef VISUALIZATION
    visualizer.plannerParamsIn(params);
    #endif
}

void Planner::Steer()
{
    Node q_f, q_possible; 
    double best_angle   = 0; 
    double dist         = numeric_limits<double>::infinity();
    double new_cost; 

    for(double s = -steering_max; s <= steering_max; s += steering_inc)
    {
        if(abs(s) < steering_inc)
        {
            s = 0; 
        }
        q_f.state = dynamic.new_state(qNearestPtr->node.state, s, 0.5); 
        new_cost  = EucDist(q_f, q_new);
        if(new_cost < dist)
        {
            dist       = new_cost; 
            q_possible = q_f; 
            q_possible.input = s; 
            q_possible.cost  = qNearestPtr->node.cost + EucDist(q_f, qNearestPtr->node); 
        } 
    }
    q_new = q_possible; 
}

int Planner::DubinsCurve(DubinsPath* path)
{
    Node q_nearest = qNearestPtr->node ; 

    double q0[]          = {q_nearest.state.x, q_nearest.state.y, q_nearest.state.theta}; 
    double q1[]          = {q_new.state.x, q_new.state.y, q_new.state.theta}; 
    double turningRadius = 10.0; 
    
    int err = dubins_shortest_path(path, q0, q1, turningRadius); 
    
    return (err)? 0 : 1; 
}

void Planner::RRTstar()
{
    clock_t st = clock();
    vector<kdNodePtr> nearby_nodes; 
    for(int i = 0; i < params.iterations; i++)
    {  
        q_new = RandomPoint(0);//pass zero to turn off goal bias

        qNearestPtr =  tree.findNearestPtr(q_new);
        q_new.cost  += EucDist(q_new, qNearestPtr->node);  
        double distNewNear = EucDist(q_new, qNearestPtr->node); 

        #ifdef DUBINSCURVE
        // q_new.state.theta = qNearestPtr->node.state.theta + distribution(generator)*2*M_PI/12 - 2*M_PI/12;
        // q_new.state.theta = q_new.state.theta - 2*M_PI * floor(q_new.state.theta / 2*M_PI); 
        
        DubinsPath path;
        if(DubinsCurve(&path) == 0) continue; 
        q_new.cost = path.param[0] + path.param[1] + path.param[2]; 

        if(distNewNear < 1000 && collisionCheckDubins(&path))
        #endif
        
        #ifdef DYNAMICS
        Steer();

        if(distNewNear < 1000 && CollisionCheck(q_new, qNearestPtr->node, params.obstacle))
        #endif        

        {
            maximum_cost = max(maximum_cost, q_new.cost + EucDist(q_new, q_goal)); 
            maxDist      = max(maxDist, EucDist(q_new, q_origin)); 
         
            qNewPtr = tree.insert(q_new);
            qNewPtr->parent = qNearestPtr; 
            
            #ifdef DUBINSCURVE
            qNewPtr->path      = path; 
            qNewPtr->node.cost += qNearestPtr->node.cost; 
            #endif

            nearby_nodes = tree.Nearby(q_new, 50.0);
            Rewire(nearby_nodes); 

            
            #ifdef DUBINSCURVE
            if(GoalProxDubins())
            #elif defined(DYNAMICS)
            if(GoalProx())
            #endif
            {
                #ifdef VISUALIZATION
                qGoalPtr = tree.insert(q_goal); 
                qGoalPtr->parent = qNewPtr;
                
                #ifdef DYNAMICS
                visualizer.drawMapwGoalPath(tree.getRootPtr(), qGoalPtr); 
                #endif
                #ifdef DUBINSCURVE
                visualizer.drawDubinsCurve(tree.getRootPtr(), qGoalPtr); 
                #endif
                
                #endif
                
                cout << "Number of iteration: " << i << endl;
                return; 
            }
        }
        #ifdef VISUALIZATION
        visualizer.drawMap(tree.getRootPtr(), q_goal); 
        #endif

        cout << i << endl;
    }
    cout << "Time is: " << (clock()-st)/CLOCKS_PER_SEC << endl;
    return;
}

void Planner::print()
{
    tree.printTree();
}

/**
* private
*/

Node Planner::RandomPoint()
{
    Node q_new;

    q_new.state.RandomState(distribution(generator));
    q_new.state.x  = params.width*distribution(generator)+(0-params.width/2);
    q_new.state.y  = params.height*distribution(generator)+(0-params.height/2);
    q_new.input    = 0;
    q_new.cost     = 0;
    q_new.parent.x = q_origin.state.x;
    q_new.parent.y = q_origin.state.y;

    return q_new; 
}

Node Planner::RandomPoint(int k) //k nearest planner
{       
    if (k == 0){
        double tmp = maxDist + distCoeff;
        
        q_new.state.x  = params.width*distribution(generator)+(0-params.width/2);
        q_new.state.y  = params.height*distribution(generator)+(0-params.height/2);
        q_new.state.RandomState(distribution(generator));
        q_new.input    = 0;
        q_new.cost     = 0;
        q_new.parent.x = q_origin.state.x; // remove maybe
        q_new.parent.y = q_origin.state.y;

        return q_new; 
    }else{
        while(1){
            q_new.state.RandomState(distribution(generator));
            q_new.state.x  = params.width*distribution(generator)+(0-params.width/2);
            q_new.state.y  = params.height*distribution(generator)+(0-params.height/2);
            q_new.input    = 0;
            q_new.cost     = 0;
            q_new.parent.x = q_origin.state.x;
            q_new.parent.y = q_origin.state.y;

            qNearestPtr = tree.findNearestPtr(q_new);
            q_new.cost  = EucDist(q_new, qNearestPtr->node) + qNearestPtr->node.cost;  


            double mquality = 1 - ((q_new.cost + EucDist(q_new, q_goal) - optimal_cost)/(maximum_cost-optimal_cost));
            mquality        = min(mquality,0.4);

            double r        = distribution(generator);
            if (r < mquality)
            {
                q_new.cost = 0;
                return q_new;
            }
        }
    }
}

bool Planner::SteerForRewire(const kdNodePtr& p1, const kdNodePtr& p2)
{
    double x_eps = 0.3, y_eps = 0.3; // static make ratio of the arear of the map
    for(double s = -steering_max; s <= steering_inc; s += steering_inc)
    {
        States st = dynamic.new_state(p1->node.state, s, 0.5); 
        if(abs(st.x-p2->node.state.x) < x_eps && abs(st.y-p2->node.state.y) < y_eps)
        {
            p2->node.state = st; 
            return true;
        } 
    }    
    return false;
}

bool Planner::dubinForRewire(const kdNodePtr& p1, const kdNodePtr& p2, DubinsPath* path)
{
    Node n1 = p1->node, n2 = p2->node;
    double q0[]          = {n1.state.x, n1.state.y, n1.state.theta}; 
    double q1[]          = {n2.state.x, n2.state.y, n2.state.theta}; 
    double turningRadius = 10.0; 
    
    int err = dubins_shortest_path(path, q0, q1, turningRadius);
    return (err)? false : true;
}

bool Planner::collisionCheckDubins(DubinsPath* path){
    double qInit[3];
    for(int i = 0; i < 3; i++){
        qInit[i] = path->qi[i]; 
    }
    double len = dubins_path_length(path), x = 0;
    bool safe = true;  
    for(;x <= len; x += len/5){         
        Point p(qInit[0], qInit[1]); 
        dubins_path_sample(path, x, qInit); 
        Point q(qInit[0], qInit[1]);         
        if (abs(q.x) > params.width/2 || abs(q.y) > params.height/2 || !collisionCheckPoint(p, q, params.obstacle)){
            return !safe; 
        };
    }
    return safe; 
}


void Planner::Rewire(vector<kdNodePtr>& nearby_nodes)
{
    double temp_cost; 
    for(auto& q : nearby_nodes)
    {
        if(q->node == qNearestPtr->node) continue; 

        #ifdef DUBINSCURVE
        DubinsPath path; 
        if(!dubinForRewire(qNewPtr, q, &path)) continue;
        temp_cost = qNewPtr->node.cost + path.param[0] + path.param[1] + path.param[2]; 
        if(q->node.cost > temp_cost && collisionCheckDubins(&path))
        {
            q->path = path; 

        #elif defined(DYNAMICS)
        temp_cost = qNewPtr->node.cost + EucDist(qNewPtr->node, q->node); 
        if(q->node.cost > temp_cost && SteerForRewire(qNewPtr, q))
        {
        #endif

            // std::cout << "rewire" << std::endl; 
            q->parent    = qNewPtr; 
            q->node.cost = temp_cost;  
        }
    }
}

void Planner::ReviseNearest(const vector<kdNodePtr>& nearby_nodes)
{
    double new_cost; 
    for(auto i : nearby_nodes)
    {
        new_cost = i->node.cost + EucDist(qNewPtr->node, i->node); 
        if(new_cost < qNewPtr->node.cost && SteerForRewire(i, qNewPtr))
        {
            cout <<  "revise nearest" << endl; 
            qNewPtr->parent    = i;
            qNewPtr->node.cost = new_cost; 
        }
    }
}

bool Planner::GoalProx()
{
    double dist = EucDist(q_new, q_goal);
    return (dist < params.goalProx)? true : false; 
}

bool Planner::GoalProxDubins(){
    DubinsPath path = qNewPtr->path; 
    double len = dubins_path_length(&path), x = 0, dist; 
    double q[3];
    for(int i = 0; i < 3; i++) q[i] = path.qi[i];
    bool GOAL = true; 
    for(;x <= len; x += len/8){
        dubins_path_sample(&path, x, q); 
        dist = sqrt(pow(q_goal.state.x - q[0], 2) + pow(q_goal.state.y-q[1],2)); 
        if(dist < params.goalProx){ 
            return GOAL;
        }
    }
    return !GOAL;
}

int main()
{
    planner_params A;
    A.origin = Point(400,- 400);
    A.goal   = Point(-400, 400);
    MatrixXd obstacle(5,4);

    // Simple Rectangle obstacle
    // obstacle.row(0) << 1,1,-1,1;
    // obstacle.row(1) << -1,1,-1,-1;
    // obstacle.row(2) << -1,-1,1,-1;
    // obstacle.row(3) << 1,-1,1,1;
    // A.obstacle = 100*obstacle;

    // Maze Map
    obstacle.row(0) << 50,0,50,150;
    obstacle.row(1) << 150,200,150,100;
    obstacle.row(2) << 150,100,100,100;
    obstacle.row(3) << 125,0,125,50;
    obstacle.row(4) << 200,75,150,75;
    A.obstacle = obstacle-(100*MatrixXd::Ones(5,4));
    // cout<< A.obstacle<< endl;
    A.obstacle.col(0) = -1*A.obstacle.col(0);
    A.obstacle.col(2) = -1*A.obstacle.col(2);
    A.obstacle *= 5; 

    A.iterations = 8000;
    A.width      = 1000; 
    A.height     = 1000;
    A.goalProx   = 15;

    Planner p(A);
    
    p.RRTstar();
    // p.print();
    
    return 0;
}