#pragma once

#include "kdTreeNode.hpp"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class Visualizer{

private:
    int xUpperLim; 
    int yUpperLim;
    int xLowerLim;
    int yLowerLim;
    int goalProx; 
    Eigen::MatrixXd obstacle;
    string mName; 
    int mFrame;

    void line(double x1, double y1, double x2, double y2, string lineType); 
    void drawObstacle(); 
    void drawNodes(const kdNodePtr& root); 
    void drawGoal(const Node& goal);
    void wire(const kdNodePtr& root, string color);  
    void wireGoalPath(const kdNodePtr& goalPtr); 
    void wireGoalDubin(const kdNodePtr& goalPtr); 

public:
    Visualizer(); 

    void plannerParamsIn(const planner_params& A); 
    void drawMap(const kdNodePtr& root, const Node& goal); 
    void drawMapGoalPath(const kdNodePtr& root, const kdNodePtr& goalPtr); 
    void drawDubinsCurve(const kdNodePtr& root, const kdNodePtr& goalPtr); 
};