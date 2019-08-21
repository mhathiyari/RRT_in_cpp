#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__

#include "common.hpp"

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

    string mName; 

    void drawObstacle(const Eigen::MatrixXd& obstacle);
    void drawNodes(const vector<Node>& node_list); 
    void drawGoal(const Node& goal); 
    void wire(const vector<Node>& node_list, const cv::Scalar color);
    void show(); 

public:
    Visualizer(); 
    void plannerParamsIn(const planner_params& A);

    int getRows();
    int getCols(); 

    void drawMap(const planner_params& A, const vector<Node>& nodeList, const Node& goal);
    void drawMapwGoalPath(const planner_params& A, const vector<Node>& nodeList, const vector<Node>& goalPath);

};

#endif