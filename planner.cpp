#include "planner.hpp"

/*
* static
*/
double Planner::distCoeff = 300.0; 

default_random_engine generator(time(0));
uniform_real_distribution<double> distribution(0,1);

/*
* public
*/

double EucDist(Node& q1, Node& q2)
{
    return q1.state.Cost(q2.state);
}

//Constructor 
Planner::Planner(const PlannerParams& params_in)
{
    params       = params_in; 
    steering_max = 1.02; 
    steering_inc = steering_max/21;

    q_origin.SetCoord(params.origin);
    q_origin.input = 0; 
    q_origin.cost  = 0; 
    q_origin.parent.x = params.origin.x;//see if parent gets used remove 
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

// iterates throught all possible steering angle and selects one with minmum cost
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
    clock_t st = clock();                                       //Time at entry
    vector<kdNodePtr> nearby_nodes; 
    for(int i = 0; i < params.iterations; i++)
    {  
        q_new = RandomPoint(0);                                  //pass zero to turn off goal bias

        nearby_nodes = tree.Nearby(q_new, 500.0);
        qNearestPtr =  tree.findNearestPtr(q_new);
        double distNewNear = EucDist(q_new, qNearestPtr->node); 
        q_new.cost  += distNewNear;
        
        Steer();

        if(distNewNear < 1000 && CollisionCheck(q_new, qNearestPtr->node, params.obstacle))
        {
            maximum_cost = max(maximum_cost, q_new.cost + EucDist(q_new, q_goal)); 
            maxDist      = max(maxDist, EucDist(q_new, q_origin)); 
         
            qNewPtr = tree.insert(q_new);
            qNewPtr->parent = qNearestPtr; 
        
            Rewire(nearby_nodes); 

            if(GoalProx())
            {
                #ifdef VISUALIZATION
                qGoalPtr = tree.insert(q_goal); 
                qGoalPtr->parent = qNewPtr;
                
                visualizer.drawMapwGoalPath(tree.getRootPtr(), qGoalPtr); 
                #endif
                
                cout << "Number of iteration: " << i << endl;
                return; 
            }
        }
        #ifdef VISUALIZATION
        //visualizer.drawMap(tree.getRootPtr(), q_goal); 
        #endif

        cout << i << endl;
    }
    cout << "Time is: " << (clock()-st)/CLOCKS_PER_SEC << endl;
    return;
}

int helper(double q[3], double x, void* user_data)
{   
    return 0; 
}

void Planner::ExtractPath(Path& path){
    kdNodePtr p = qGoalPtr;
    while(p && p->parent){
        std::vector<std::vector<double>> samplePoints; 
        std::vector<double> xTmp, yTmp; 
        dubins_path_sample_many(&p->path, 5.0, helper, samplePoints); 
        double x1, y1; 
        for(int i = 0; i < samplePoints.size(); i++){
            x1 = samplePoints[i][0];
            y1 = samplePoints[i][1]; 
            xTmp.push_back(x1); 
            yTmp.push_back(y1); 
        }
        x1 = p->node.state.x; 
        y1 = p->node.state.y; 
        xTmp.push_back(x1); 
        yTmp.push_back(y1);
        reverse(xTmp.begin(), xTmp.end()); 
        reverse(yTmp.begin(), yTmp.end()); 
        for(int i = 0; i < xTmp.size(); i++){
            path.cx.push_back(xTmp[i]); 
            path.cy.push_back(yTmp[i]);
        }
        p = p->parent; 
    }
}

void Planner::print()
{
    tree.printTree();
}

/**
* private
*/

kdNodePtr Planner::findNearestLeastCost(std::vector<kdNodePtr> nearbyNodes){
    double minCost = pow((params.width + params.height), 2), tempCost; 
    kdNodePtr ret; 
    for(kdNodePtr ptr : nearbyNodes){
        tempCost = calDistNode(q_new, ptr->node) + ptr->node.cost; 
        if(tempCost < minCost){
            tempCost = minCost; 
            ret = ptr; 
        }
    }
    return ret;
}

Node Planner::RandomPoint()
{
    q_new.state.x  = params.width*distribution(generator)+(0-params.width/2);
    q_new.state.y  = params.height*distribution(generator)+(0-params.height/2);
    q_new.state.RandomState(distribution(generator));
    q_new.input    = 0;
    q_new.cost     = 0;  //incase we ever wabt to add a heatmap of cost(State cost)
    q_new.parent.x = q_origin.state.x; // remove maybe
    q_new.parent.y = q_origin.state.y;
    return q_new; 
    
}

Node Planner::RandomPoint(int k) //k nearest planner based on https://www.ri.cmu.edu/pub_files/pub4/urmson_christopher_2003_1/urmson_christopher_2003_1.pdf
{       
    if (k == 0)
        return RandomPoint(); //Truly Random
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
        if (abs(q.x) > params.width/2 || abs(q.y) > params.height/2 || !CollisionCheckPoint(p, q, params.obstacle)){
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