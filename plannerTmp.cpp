#include "plannerTmp.hpp"

/*
* public
*/

double euc_dist(Node& q1, Node& q2)
{
    return q1.state.cost(q2.state);
}

Planner::Planner(const planner_params& params_in)
{
    params       = params_in; 
    steering_max = 1.02; 
    steering_inc = steering_max/21;

    q_origin.setcoord(params.origin);
    q_origin.input = 0; 
    q_origin.cost  = 0; 
    q_origin.parent.x = params.origin.x;//seet if parent gets used remove 
    q_origin.parent.y = params.origin.y; 

    q_goal.setcoord(params.goal); 
    q_goal.input = 0; 
    q_goal.cost  = 0; 
    q_goal.parent.x = Infinity;// set to something meaningful or remove
    q_goal.parent.y = Infinity; 

    optimal_cost = q_origin.state.cost(q_goal.state);
    maximum_cost = optimal_cost + 400; // avoiding devided by 0 10*opt +1

    tree.treeInit(q_origin);  

    #ifdef VISUALIZATION
    visualizer.plannerParamsIn(params);
    #endif
}

void Planner::steer()
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
        new_cost = euc_dist(q_f, q_new);
        if(new_cost < dist)
        {
            dist = new_cost; 
            q_possible = q_f; 
            q_possible.input = s; 
            q_possible.cost  = qNearestPtr->node.cost + euc_dist(q_f, qNearestPtr->node); 
        } 
    }
    q_new = q_possible; 
}

void Planner::RRTstar()
{
    clock_t st = clock();
    vector<kdNodePtr> nearby_nodes; 
    for(int i = 0; i < params.iterations; i++)
    {  

        // qNewPtr     = tree.insert(random_point()); 

        q_new = random_point(0);//pass zero to turn off goal bias

        qNearestPtr = tree.findNearestPtr(q_new);
        q_new.cost += euc_dist(q_new, qNearestPtr->node);  

        steer();
        if(euc_dist(q_new, qNearestPtr->node) < 1000 && collision_check(q_new, qNearestPtr->node, params.obstacle)) //should think of an alternative  1000 dist compare
        {

            maximum_cost = max(maximum_cost, q_new.cost + euc_dist(q_new, q_goal)); 

            qNewPtr = tree.insert(q_new);
            qNewPtr->parent = qNearestPtr; 
            nearby_nodes = tree.nearby(q_new, 50.0);
            // rewire(nearby_nodes); 

            if(goal_prox())
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

Node Planner::random_point()
{
    Node q_new;

    q_new.state.random_state(distribution(generator));
    q_new.state.x  = params.width*distribution(generator)+(0-params.width/2);
    q_new.state.y  = params.height*distribution(generator)+(0-params.height/2);
    q_new.input    = 0;
    q_new.cost     = 0;
    q_new.parent.x = q_origin.state.x;
    q_new.parent.y = q_origin.state.y;

    return q_new; 
}

Node Planner::random_point(int k) //k nearest planner
{   
    if (k == 0){
        Node q_new;

        q_new.state.random_state(distribution(generator));
        q_new.state.x  = params.width*distribution(generator)+(0-params.width/2);
        q_new.state.y  = params.height*distribution(generator)+(0-params.height/2);
        q_new.input    = 0;
        q_new.cost     = 0;
        q_new.parent.x = q_origin.state.x; // remove maybe
        q_new.parent.y = q_origin.state.y;

        return q_new; 
    }else{
        while(1){
            q_new.state.random_state(distribution(generator));
            q_new.state.x  = params.width*distribution(generator)+(0-params.width/2);
            q_new.state.y  = params.height*distribution(generator)+(0-params.height/2);
            q_new.input    = 0;
            q_new.cost     = 0;
            q_new.parent.x = q_origin.state.x;
            q_new.parent.y = q_origin.state.y;

            qNearestPtr = tree.findNearestPtr(q_new);
            q_new.cost  = euc_dist(q_new, qNearestPtr->node) + qNearestPtr->node.cost;  


            double mquality = 1 - ((q_new.cost + euc_dist(q_new, q_goal) - optimal_cost)/(maximum_cost-optimal_cost));

            mquality = min(mquality,0.4);
            double r = distribution(generator);

            

            if (r < mquality)
            {
                q_new.cost = 0;
                return q_new;
            }
        }
    }
}

bool Planner::steerForRewire(const kdNodePtr& p1, const kdNodePtr& p2)
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

void Planner::rewire(vector<kdNodePtr>& nearby_nodes)
{
    double temp_cost; 
    for(auto& q : nearby_nodes)
    {
        if(q->node == qNearestPtr->node) continue; 
        
        temp_cost = (qNewPtr->node.cost + euc_dist(qNewPtr->node, q->node)); 
        if(q->node.cost > temp_cost && steerForRewire(qNewPtr, q))
        {
            std::cout << "rewire" << std::endl; 
            q->parent    = qNewPtr; 
            q->node.cost = temp_cost;  
        }
    }
}

void Planner::revise_nearest(const vector<kdNodePtr>& nearby_nodes)
{
    double new_cost; 
    for(auto i : nearby_nodes)
    {
        new_cost = i->node.cost + euc_dist(qNewPtr->node, i->node); 
        if(new_cost < qNewPtr->node.cost && steerForRewire(i, qNewPtr))
        {
            cout <<  "revise nearest" << endl; 
            qNewPtr->parent    = i;
            qNewPtr->node.cost = new_cost; 
        }
    }
}

bool Planner::goal_prox()
{
    double dist = euc_dist(q_new, q_goal);
    return (dist < params.goalProx)? true : false; 
}

int main()
{
    planner_params A;
    A.origin = Point(90,0);
    A.goal = Point(-90, 90);
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
    cout<< A.obstacle<< endl;
    A.obstacle.col(0) = -1*A.obstacle.col(0);
    A.obstacle.col(2) = -1*A.obstacle.col(2);

    A.iterations = 8000;
    A.width = 200; 
    A.height = 200;
    A.goalProx = 5;

    Planner p(A);
    
    p.RRTstar();
    // p.print();
    
    return 0;
}