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
    cols     = A.width;  
    rows     = A.height; 
    goalProx = A.goalProx;
    obstacle = A.obstacle; 
}

int Visualizer::getRows()
{
    return rows; 
}

int Visualizer::getCols()
{
    return cols; 
}

void Visualizer::drawObstacle()
{
    /*
    try{
        if(map.empty()) throw 1;
    }catch(...){
        std::cout << "Mat Initialization Failed." << std::endl; 
    }
    */
    for (int i = 0; i<obstacle.rows(); i++){
        double x1 = obstacle(i,0), y1 = obstacle(i,1), x2 = obstacle(i,2), y2 = obstacle(i,3); 
        tfXy2Pixel(x1, y1, cols, rows); 
        tfXy2Pixel(x2, y2, cols, rows); 

        cv::line(map, 
                 cv::Point2d(x1,y1), 
                 cv::Point2d(x2,y2), 
                 cv::Scalar(0,100,0),
                 3,
                 CV_AA);
    }
    // // top left corner 
    // int width  = calDist(obstacle(0,0), obstacle(0,1), obstacle(0,2), obstacle(0,3)); 
    // int height = calDist(obstacle(1,0), obstacle(1,1), obstacle(1,2), obstacle(1,3)); 
    // cv::rectangle(map, 
    //               cv::Rect((int)obstacle(1,0) + cols/2,
    //                        rows/2 - (int)obstacle(1,1),
    //                        width,
    //                        height),cv::Scalar(0,100,0), -1); 
}

void Visualizer::drawGoal(const Node& goal)
{
    double x = goal.state.x, y = goal.state.y; 
    tfXy2Pixel(x, y, cols, rows); 
    
    cv::circle(map, 
               cv::Point2d(x, y), 
               goalProx, 
               cv::Scalar(255,0,0), 
               -1, 
               CV_AA);
}

void Visualizer::drawNodes(const kdNodePtr& root)
{
    int count = 0, size; 
    cv::Scalar color;

    std::queue<kdNodePtr> q;
    q.push(root);

    while(!q.empty())
    {
        kdNodePtr p = q.front();
        Node n      = p->node;  
        q.pop();

        size = 1;
        color = cv::Scalar(0,0,255); 
        
        if(count == 0)
        {
            size = 5, color = cv::Scalar(255,0,0); 
        }
        count++;

        double x = n.state.x, y = n.state.y; 
        tfXy2Pixel(x, y, cols, rows); 
        
        cv::circle(map, 
                   cv::Point2d(x, y), 
                   size, 
                   color, 
                   -1, 
                   CV_AA);
        
        if(p->left)  q.push(p->left); 
        if(p->right) q.push(p->right); 
    }
}

void Visualizer::wire(const kdNodePtr& root, const cv::Scalar color)
{
    if(root == nullptr) return; 
    std::queue<kdNodePtr> q; 
    if(root->left)  q.push(root->left); 
    if(root->right) q.push(root->right); 
    while(!q.empty())
    {
        kdNodePtr p = q.front(); 
        q.pop();
        Node n1 = p->node, n2;  
        if(p->parent == nullptr)
        {
            std::cout << "Parent error" << std::endl;
        }else
        {
            n2 = p->parent->node; 
        }
         

        double x1 = n1.state.x, y1 = n1.state.y, x2 = n2.state.x, y2 = n2.state.y; 
        tfXy2Pixel(x1, y1, cols, rows); 
        tfXy2Pixel(x2, y2, cols, rows); 

        cv::line(map, 
                 cv::Point2d(x1,y1), 
                 cv::Point2d(x2,y2), 
                 color,
                 1,
                 CV_AA);
        
        if(p->left)  q.push(p->left); 
        if(p->right) q.push(p->right);

    }
}

void Visualizer::wireGoalPath(const kdNodePtr& goalPtr)
{
    kdNodePtr p = goalPtr; 
    Node n1, n2; 
    while(p && p->parent)
    {
        n1 = p->node;
        n2 = p->parent->node; 
        double x1 = n1.state.x, y1 = n1.state.y, x2 = n2.state.x, y2 = n2.state.y; 
        tfXy2Pixel(x1, y1, cols, rows); 
        tfXy2Pixel(x2, y2, cols, rows); 

        cv::line(map, 
                 cv::Point2d(x1,y1), 
                 cv::Point2d(x2,y2), 
                 cv::Scalar(255,0,0),
                 1,
                 CV_AA);

        p = p->parent;
    }
}

int wireDubins(double q[3], double x, void* user_data)
{   

}

void Visualizer::wireGoalDubin(const kdNodePtr& goalPtr)
{
    kdNodePtr p = goalPtr; 
    while(p && p->parent){
        std::vector<std::vector<double>> samplePoints; 
        dubins_path_sample_many(&p->path, 5, wireDubins, samplePoints); 

        for(int i = 0; i < samplePoints.size(); i++)
        {
            double x = samplePoints[i][0], y = samplePoints[i][1]; 
            tfXy2Pixel(x, y, cols, rows); 
            cv::circle(map, 
                       cv::Point2d(x, y), 
                       2, 
                       cv::Scalar(255,255,255), 
                       -1, 
                       CV_AA);
        }

        double x = p->node.state.x, y = p->node.state.y;
        tfXy2Pixel(x, y, cols, rows); 
        cv::circle(map, cv::Point2d(x, y), 2, cv::Scalar(0,0,255), -1, CV_AA); 

        p = p->parent;
    }
}

void Visualizer::show()
{
    cv::imshow(mName, map); 
}
/*
void Visualizer::drawMap(const vector<Node>& nodeList, const Node& goal)
{
    if(map.empty())
    {
        map = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC3); 
    }
    mFrame++; 
    
    drawObstacle(A.obstacle); 
    wire(nodeList, cv::Scalar(255,255,255)); 
    drawNodes(nodeList); 
    drawGoal(goal); 
    

    show(); 
    char key = cv::waitKey(1); 
}
*/

void Visualizer::drawMap(const kdNodePtr& root, const Node& goal)
{
    map = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC3); 
    mFrame++; 
    
    drawObstacle(); 
    wire(root, cv::Scalar(255,255,255)); 
    drawNodes(root); 
    drawGoal(goal); 
    

    show(); 
    char key = cv::waitKey(1); 
}


void Visualizer::drawMapwGoalPath(const kdNodePtr& root, const kdNodePtr& goalPtr)
{
    map = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC3); 

    mFrame++; 

    drawObstacle(); 
    wire(root, cv::Scalar(255,255,255));
    wireGoalPath(goalPtr); 
    drawNodes(root); 
    drawGoal(goalPtr->node);
    
    show(); 
    char key = cv::waitKey(100000);
}

void Visualizer::drawDubinsCurve(const kdNodePtr& root, const kdNodePtr& goalPtr)
{
    
    map = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC3); 
    
    mFrame++; 

    drawObstacle(); 
    wireGoalDubin(goalPtr); 
    drawGoal(goalPtr->node); 
    show(); 
    cv::waitKey(100000);
}