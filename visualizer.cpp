#include "visualizer.hpp"

int calDist(double x1, double y1, double x2, double y2)
{
    double ret = sqrt((pow(x1-x2,2) + pow(y1-y2,2))); 
    return (int)ret;
}

Visualizer::Visualizer()
{
    mFrame = 0; 
    mName  = "Visualizer"; 
}

void Visualizer::plannerParamsIn(const planner_params& A)
{    
    cols   = A.width;  
    rows   = A.height;  
}

int Visualizer::getRows()
{
    return rows; 
}

int Visualizer::getCols()
{
    return cols; 
}

void Visualizer::drawObstacle(const Eigen::MatrixXd& obstacle)
{
    /*
    try{
        if(map.empty()) throw 1;
    }catch(...){
        std::cout << "Mat Initialization Failed." << std::endl; 
    }
    */
    // top left corner 
    int width  = calDist(obstacle(0,0), obstacle(0,1), obstacle(0,2), obstacle(0,3)); 
    int height = calDist(obstacle(1,0), obstacle(1,1), obstacle(1,2), obstacle(1,3)); 
    cv::rectangle(map, 
                  cv::Rect((int)obstacle(1,0) + cols/2,
                           rows/2 - (int)obstacle(1,1),
                           width,
                           height),cv::Scalar(0,100,0), -1); 
}

void Visualizer::drawNodes(const vector<Node>& nodeList)
{
    int n = nodeList.size(); 
    for(Node n: nodeList)
    {
        double x = n.x, y = n.y; 
        tfXy2Pixel(x, y, cols, rows); 
        
        cv::circle(map, 
                   cv::Point2d(x, y), 
                   1, 
                   cv::Scalar(0,0,255), 
                   -1, 
                   CV_AA);
    }
}

void Visualizer::wire(const vector<Node>& nodeList)
{
    for(Node n : nodeList)
    {
        Point p = n.parent; 
        double x1 = n.x, y1 = n.y, x2 = p.x, y2 = p.y; 
        tfXy2Pixel(x1, y1, cols, rows); 
        tfXy2Pixel(x2, y2, cols, rows); 

        cv::line(map, 
                 cv::Point2d(x1,y1), 
                 cv::Point2d(x2,y2), 
                 cv::Scalar(255,255,255),
                 1,
                 CV_AA); 
    }
}

void Visualizer::show()
{
    cv::imshow(mName, map); 
}

void Visualizer::drawMap(const planner_params& A, const vector<Node>& nodeList)
{
    if(map.empty())
    {
        map = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC3); 
    }
    mFrame++; 
    
    drawObstacle(A.obstacle); 
    drawNodes(nodeList); 
    wire(nodeList); 

    show(); 
    char key = cv::waitKey(1); 
}