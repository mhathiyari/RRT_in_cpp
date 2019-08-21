#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__

#include "planner.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

inline void tfXy2Pixel(double& x, double& y, const int& width, const int& height)
{
  x += width/2; 
  y = height/2 - y; 
}

typedef cv::Mat Image; 

class Visualizer
{
private:
    Image map; 

    int mFrame; 
    int rows; 
    int cols; 

    string mName; 

    void drawObstacle(const Eigen::MatrixXd& obstacle);
    void drawNodes(const vector<Node>& node_list); 
    void wire(const vector<Node>& node_list);
    void show(); 

public:
    Visualizer(); 
    void plannerParamsIn(const planner_params& A);

    int getRows();
    int getCols(); 

    void drawMap(const planner_params& A, const vector<Node>& nodeList);

};

#endif