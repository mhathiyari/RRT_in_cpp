#pragma once

#include "common.hpp"
#include "dubins.h"
// #include <random>
// #include <memory>
// #include <vector> 
// #include <functional>
// #include <algorithm>
// #include <queue> 
// #include <cmath>
// #include <ctime>




struct kdNode
{
    Node                    node; 
    std::shared_ptr<kdNode> left; 
    std::shared_ptr<kdNode> right; 
    std::weak_ptr<kdNode>   parent; 
    DubinsPath              path; 
    
    kdNode(){}; //look into whether its needed 
    kdNode(const Node& n)
    :node(n), left(nullptr), right(nullptr){}; 
};
typedef std::shared_ptr<kdNode> kdNodePtr;

class kdTreeNode
{
public:
   
    kdTreeNode(){}; 

    void treeInit(const Node& n); 
    void printTree(); 
    kdNodePtr getRootPtr(); // should not do smth like this, only for debugging used only by visualizer
    kdNodePtr insert(const Node& n);
    kdNodePtr findNearestPtr(const Node& n); 
    std::vector<kdNodePtr> Nearby(const Node& n, const double& thres);

private:
    
    kdNodePtr root; 
    int   dim; 

    kdNodePtr insert(const Node& n, kdNodePtr& r, int level); 
    kdNodePtr findNearest(const kdNodePtr& r, // start node of the tree
                          const Node& n, // position we are querying
                          const int level, // x or y
                          const kdNodePtr& best, // store the best option so far
                          const double& bestDist); // store the min dist
    void Nearby(const kdNodePtr& r,
                const Node& n, 
                const int level, 
                const double& thres,
                std::vector<kdNodePtr>& nearbyPtrVec);
    /*
    kdNodePtr getParentNode(const kdNodePtr& r,
                            const Node& n,
                            const int level);
    */
}; 