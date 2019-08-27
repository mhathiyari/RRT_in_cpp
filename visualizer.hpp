#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__

#include "kdTreeNode.hpp"

#include <cv.h>
#include <highgui.h>


typedef cv::Mat Image; 

class Visualizer
{
private:
    Image map; 

    int mFrame; 
    int rows; 
    int cols; 
    int goalProx;
    Eigen::MatrixXd obstacle; 

    string mName; 

    void drawObstacle();
    void drawNodes(const kdNodePtr& root); 
    void drawGoal(const Node& goal); 
    void wire(const kdNodePtr& root, const cv::Scalar color);
    void wireGoalPath(const kdNodePtr& goalPtr);
    void show(); 

public:
    Visualizer(); 
    void plannerParamsIn(const planner_params& A);

    int getRows();
    int getCols(); 

    void drawMap(const kdNodePtr& root, const Node& goal);
    void drawMapwGoalPath(const kdNodePtr& root, const kdNodePtr& goalPtr);

};

#endif