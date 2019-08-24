#pragma once

#include "common.hpp"
#include <random>
#include <memory>
#include <vector> 
#include <functional>
#include <algorithm>
#include <queue> 
#include <cmath>
#include <ctime>




struct kdNode
{
    Node                    node; 
    std::shared_ptr<kdNode> left; 
    std::shared_ptr<kdNode> right; 
    std::shared_ptr<kdNode> parent; 

    kdNode(){}; 
    kdNode(const Node& n)
    :node(n), left(nullptr), right(nullptr), parent(nullptr){}; 
};
typedef std::shared_ptr<kdNode> kdNodePtr;

class kdTreeNode
{
public:
   
    kdTreeNode(const Node& n); 

    void printTree(); 
    kdNodePtr getRootPtr(); // should not do smth like this, only for debugging
    // void setParent(kdNodePtr& np, const kdNodePtr& p);
    kdNodePtr insert(const Node& n);
    Node findNearestPoint(const Node& n); 
    // Node getParent(const Node& n);    

private:
    
    kdNodePtr root; 
    int   dim; 

    // double calDistNode(const Node& n1, const Node& n2);
    kdNodePtr insert(const Node& n, kdNodePtr& r, int level); 
    // Node findNearest(const Node& n); 
    kdNodePtr findNearest(const kdNodePtr& r, // start node of the tree
                          const Node& n, // position we are querying
                          const int level, // x or y
                          const kdNodePtr& best, // store the best option so far
                          const double& bestDist); // store the min dist
    /*
    kdNodePtr getParentNode(const kdNodePtr& r,
                            const Node& n,
                            const int level);
    */
}; 