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

int wireDubins(double q[3], double x, void* user_data)
{   

}

void Visualizer::wireGoalDubin(const kdNodePtr& goalPtr){
    kdNodePtr p = goalPtr; 
    while(p && p->parent){
        std::vector<std::vector<double>> samplePoints; 
        dubins_path_sample_many(&p->path, 5.0, wireDubins, samplePoints);
        std::cout << "size is: " << samplePoints.size() << std::endl;
        for(int i = 0; i < samplePoints.size(); i++){
            double x1 = samplePoints[i][0], y1 = samplePoints[i][1]; 
            // double x2 = samplePoints[i+1][0], y2 = samplePoints[i+1][1]; 
            plotPoint(x1, y1, "bo");
            // plotPoint(x2, y2, "ro");
            // plotLine(x1, y1, x2, y2, "r-");
        }
        double x = p->node.state.x, y = p->node.state.y; 
        plotPoint(x, y, "ro"); 
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

void Visualizer::plannerParamsIn(const planner_params& A){
    xUpperLim =  A.height / 2 + 100;
    xLowerLim = -A.height / 2 - 100; 
    yUpperLim =  A.width  / 2 + 100; 
    yLowerLim = -A.width  / 2 - 100;
    goalProx  =  A.goalProx; 
    obstacle  =  A.obstacle;
}

void Visualizer::drawMap(const kdNodePtr& root, const Node& goal){
    plt::clf(); 
    // plt::xlim(xLowerLim, xUpperLim);
    // plt::ylim(yLowerLim, yUpperLim);

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

    plt::show(); 
}

void Visualizer::drawDubinsCurve(const kdNodePtr& root, const kdNodePtr& goalPtr){
    plt::clf(); 
    plt::xlim(xLowerLim, xUpperLim);
    plt::ylim(yLowerLim, yUpperLim);

    drawObstacle(); 
    wireGoalDubin(goalPtr); 
    drawGoal(goalPtr->node);

    plt::show(); 
}
