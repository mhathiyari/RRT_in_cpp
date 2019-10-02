#pragma once

#include <iostream>
#include <memory>
#include <vector> 
#include <functional>
#include <algorithm>
#include <queue> 
#include <cmath>
#include <ctime>
#include <random>

template <class T> 
class kdTree
{
public:
    typedef std::vector<T> tVec; 
    struct kdNode
    {
        tVec                      pos; 
        std::shared_ptr<kdNode>   left; 
        std::shared_ptr<kdNode>   right; 
        std::shared_ptr<kdNode>   parent; 

        kdNode(){};
        kdNode(const tVec& position)
        :pos(position), left(nullptr), right(nullptr){}
    };
    typedef std::shared_ptr<kdNode> kdNodePtr;

    kdTree(const tVec& position);

    void insert(const tVec& x);             
    tVec findNearestPoint(const tVec& x); 
    tVec getParent(const tVec& x);     
    void printTree();

private:
    
    // typedef std::shared_ptr<kdNode> kdNodePtr; 

    kdNodePtr node; 
    int dim; // dimension

    T         calDisttVec(const tVec& a, const tVec& b);
    bool      insert(const tVec& x, kdNodePtr& r, int level); 
    tVec      findNearest(const tVec& x); 
    kdNodePtr findNearest(const kdNodePtr& branch,   // start node of the tree
                          const tVec&      position, // position we are querying
                          const int&       level,    // x or y
                          const kdNodePtr& best,     // store the best option so far
                          const T&    bestDist);     // store the min dist
    kdNodePtr getParentNode(const kdNodePtr& branch, 
                            const tVec&      position,
                            const int&       level);
    
}; 