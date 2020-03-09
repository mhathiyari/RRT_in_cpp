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
        if(abs(s) < steering_inc) // to prevent it from missing 0 incase of inc not being a mulitple of the limits
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

void Planner::RRTstar()
{
    clock_t st = clock();                                       //Time at entry
    vector<kdNodePtr> nearby_nodes; 
    for(int i = 0; i < params.iterations; i++)
    {  
        q_new = RandomPoint(0);                                  //pass zero to turn off goal bias

        qNearestPtr =  tree.findNearestPtr(q_new);
        double distNewNear = EucDist(q_new, qNearestPtr->node); 
        q_new.cost  += distNewNear;
        
        Steer();

        if(distNewNear < 100 && CollisionCheck(q_new, qNearestPtr->node, params.obstacle))
        {
            maximum_cost = max(maximum_cost, q_new.cost + EucDist(q_new, q_goal)); 

            qNewPtr = tree.insert(q_new);
            qNewPtr->parent = qNearestPtr; 
            
            // nearby_nodes = tree.Nearby(q_new, 500.0);
            // Rewire(nearby_nodes); 

            if(GoalProx())
            {
                #ifdef VISUALIZATION
                qGoalPtr = tree.insert(q_goal); 
                qGoalPtr->parent = qNewPtr;
                // visualizer.drawMap(tree.getRootPtr(), q_goal); 
                int q;
                // cin>>q;
                visualizer.drawMapGoalPath(tree.getRootPtr(), qGoalPtr); 
                cin>>q;

                #endif
                
                cout << "Number of iteration: " << i << endl;
                return; 
            }
        }
        #ifdef VISUALIZATION //incase you want step by step viz(SLOW!!)
        // if(i>19000)
        //visualizer.drawMap(tree.getRootPtr(), q_goal); 
        #endif

        cout << i << endl;
    }
    cout << "Time is: " << (clock()-st)/CLOCKS_PER_SEC << endl;
    return;
}


// TODO write implementation to trace the path normally
void Planner::ExtractPath(Path& path){ 
    kdNodePtr p = qGoalPtr;
    while(p && p->parent){
        std::vector<std::vector<double>> samplePoints; 
        std::vector<double> xTmp, yTmp; 
        //dubins_path_sample_many(&p->path, 5.0, helper, samplePoints); 
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
    else{
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


// Rewire is part RRT* it revises cost of nearby nodes of the q_new if possible
void Planner::Rewire(vector<kdNodePtr>& nearby_nodes)
{
    double temp_cost; 
    for(auto& q : nearby_nodes)
    {
        if(q->node == qNearestPtr->node) continue; 
        temp_cost = qNewPtr->node.cost + EucDist(qNewPtr->node, q->node); 
        if(q->node.cost > temp_cost && SteerForRewire(qNewPtr, q))
        {
           // std::cout << "rewire" << std::endl; 
            q->parent    = qNewPtr; 
            q->node.cost = temp_cost;  
        }
    }
}

bool Planner::GoalProx()
{
    double dist = EucDist(q_new, q_goal);
    return (dist < params.goalProx)? true : false; 
}

void Planner::DrawsmoothPath(Path smooth){
    //visualizer.drawMapGoalPath(tree.getRootPtr(), qGoalPtr); 
    plt::clf();
    plt::plot(smooth.cx,smooth.cy,"-g");
    plt::show();
}
