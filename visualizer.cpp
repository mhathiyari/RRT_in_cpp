#include "visualizer.hpp"

/*
* private
*/ 
void plotLine(double x1, double y1, double x2, double y2, string lineType){
    vector<double> x = {x1, x2}, y = {y1, y2}; 
    plt::plot(x, y, lineType); 
}

void plotCircle(double x, double y, double radius){
    double inc = M_PI / 20;
    std::vector<double> xPos, yPos; 
    for(double theta = 0; theta <= 2*M_PI; theta += inc){
        double x1 = x + radius * cos(theta);
        double y1 = y + radius * sin(theta); 
        xPos.push_back(x1); 
        yPos.push_back(y1); 
    }
    plt::plot(xPos, yPos, "ro");   
}

void plotPoint(double x1, double y1, string pointType){
    vector<double> x = {x1}, y = {y1};
    plt::plot(x, y, pointType);
}

void Visualizer::drawObstacle(){
    for(int i = 0; i < obstacle.rows(); i++){
        double x1 = obstacle(i,0), y1 = obstacle(i,1), x2 = obstacle(i,2), y2 = obstacle(i,3); 
        string lineType = "r-";
        plotLine(x1, y1, x2, y2, lineType);
    }
}

void Visualizer::drawNodes(const kdNodePtr& root){
    if(root == nullptr) return; 

    bool isOrigin = true; 
    string nodeType, lineType = "k"; 
    
    std::queue<kdNodePtr> q; 
    q.push(root); 
    while(!q.empty()){
        kdNodePtr p = q.front(); 
        Node      n = p->node, n2; 
        q.pop(); 
        double x1 = n.state.x, y1 = n.state.y, x2, y2; 
        nodeType = "k*"; 
        if(isOrigin){
            isOrigin = false; 
            nodeType = "ro"; 
            plotPoint(x1, y1, nodeType);
            if(p->left)  q.push(p->left); 
            if(p->right) q.push(p->right);  
            continue; 
        }

        if(p->parent == nullptr){
            std::cout << "PARENT ERROR" << std::endl;
        }else{
            n2 = p->parent->node; 
            x2 = n2.state.x;
            y2 = n2.state.y; 
        }
        // std::cout << x1 << " " << y1 << " " << x2 << " " << y2 << std::endl;
        plotPoint(x1, y1, nodeType); 
        plotLine(x1, y1, x2, y2, lineType);

        if(p->left)  q.push(p->left); 
        if(p->right) q.push(p->right); 
    }
}

void Visualizer::drawGoal(const Node& goal){
    double x = goal.state.x, y = goal.state.y; 
    plotCircle(x, y, goalProx); 
}

void Visualizer::wire(const kdNodePtr& root, string color){

}

void Visualizer::wireGoalPath(const kdNodePtr& goalPtr){
    kdNodePtr p = goalPtr; 
    Node n1, n2; 
    while(p && p->parent){
        n1 = p->node; 
        n2 = p->parent->node; 
        double x1 = n1.state.x, y1 = n1.state.y, x2 = n2.state.x, y2 = n2.state.y; 
        plotLine(x1, y1, x2, y2, "r-"); 
        
        p = p->parent; 
    }
}


/*
* public
*/
Visualizer::Visualizer(){
    mName  = "RRT w/ PureOursuit";
    mFrame = 0; 
}

void Visualizer::plannerParamsIn(const PlannerParams& A){
    xUpperLim =  A.height / 2;
    xLowerLim = -A.height / 2; 
    yUpperLim =  A.width  / 2; 
    yLowerLim = -A.width  / 2;
    goalProx  =  A.goalProx; 
    obstacle  =  A.obstacle;
}

void Visualizer::drawMap(const kdNodePtr& root, const Node& goal){
    plt::clf(); 
    // plt::xlim(xLowerLim, xUpperLim);
    // plt::ylim(yLowerLim, yUpperLim);
    string lineType = "r-";
    plotLine(xLowerLim, yUpperLim, xUpperLim, yUpperLim, lineType);
    plotLine(xLowerLim, yLowerLim, xUpperLim, yLowerLim, lineType);
    plotLine(xLowerLim, yLowerLim, xLowerLim, yUpperLim, lineType);
    plotLine(xUpperLim, yLowerLim, xUpperLim, yUpperLim, lineType);

    drawObstacle(); 
    drawNodes(root); 
    drawGoal(goal); 

    plt::pause(0.0001);
}

void Visualizer::drawMapGoalPath(const kdNodePtr& root, const kdNodePtr& goalPtr){
    plt::clf(); 
    // plt::xlim(xLowerLim, xUpperLim);
    // plt::ylim(yLowerLim, yUpperLim);

    drawObstacle(); 
    wireGoalPath(goalPtr); 
    drawGoal(goalPtr->node); 

    // plt::show(); 
    plt::pause(0.0001);
}

