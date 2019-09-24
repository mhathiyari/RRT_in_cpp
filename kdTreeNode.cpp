#include "kdTreeNode.hpp"


void kdTreeNode::treeInit(const Node& n)// add dim
{
    root = std::make_shared<kdNode>(n);
    dim  = 2; 
    if(!root)
    {
        std::cout << "CONSTRUCTOR ERROR" << std::endl;
    } // remove maybe
}

void kdTreeNode::printTree()
{
    int total = 0;
    std::queue<kdNodePtr> q;
    q.push(root); 
    while(!q.empty())
    {
        size_t n = q.size(); 
        total += n; 
        std::cout << "Size of this layer: " << n << std::endl;
        for(size_t i = 0; i < n; i++)
        {
            std::shared_ptr<kdNode> r = q.front();
            q.pop(); 

            std::cout << "[" << r->node.state.x << "," << r->node.state.y << "] ";

            if(r->left  != nullptr) q.push(r->left );  
            if(r->right != nullptr) q.push(r->right);
        }
        std::cout << std::endl;
    }
    std::cout << "total is: " << total << std::endl;
}

kdNodePtr kdTreeNode::getRootPtr()
{
    return root; 
}

kdNodePtr kdTreeNode::insert(const Node& n)
{
    return insert(n, root, 0); 
}

kdNodePtr kdTreeNode::findNearestPtr(const Node& n)
{
    double dist = calDistNode(n, root->node); 
    return findNearest(root, n, 0, root, dist);
}

std::vector<kdNodePtr> kdTreeNode::Nearby(const Node& n, const double& thres)
{
    std::vector<kdNodePtr> nearbyPtrVec; 
    Nearby(root, n, 0, thres, nearbyPtrVec); 
    return nearbyPtrVec;
}

/***************************************************/

kdNodePtr kdTreeNode::insert(const Node& n,
                             kdNodePtr& r, 
                             int level)
{
    if(r == nullptr)
    {
        r = std::make_shared<kdNode>(n);
        return r;
    }else if(level == 0)
    {
        if(n.state.x < r->node.state.x)
        {
            return insert(n, r->left,  (1+level)%dim);
        }else
        {
            return insert(n, r->right, (1+level)%dim);
        }
        
    }else 
    {
        if(n.state.y < r->node.state.y)
        {
            return insert(n, r->left,  (1+level)%dim);
        }else
        {
            return insert(n, r->right, (1+level)%dim);
        }
        
    }
} 

kdNodePtr kdTreeNode::findNearest(const kdNodePtr& r,
                                  const Node& n, 
                                  const int level, 
                                  const kdNodePtr& best,
                                  const double& bestDist)
{
    if(r == nullptr) return r;

    double dist = calDistNode(r->node, n); 

    kdNodePtr bestNew     = best; 
    double    newBestDist = bestDist; 
    if(dist < bestDist)
    {
        newBestDist = dist; 
        bestNew     = r; 
    }

    kdNodePtr section = r->right, other = r->left; 
    if((level == 0 && n.state.x < r->node.state.x) || (level == 1 && n.state.y < r->node.state.y))
    {
        section = r->left; 
        other   = r->right; 
    }
    
    kdNodePtr bestNext = findNearest(section, n, (1+level)%dim, bestNew, newBestDist);
    if(bestNext != nullptr) // check this part later
    {
        double dist2 = calDistNode(bestNext->node, n); 
        if(dist2 < newBestDist)
        {
            newBestDist = dist2; 
            bestNew     = bestNext; 
        }
    }

    if((level == 0 && abs(n.state.x - r->node.state.x) <= bestDist) || (level == 1 && abs(n.state.y - r->node.state.y) <= bestDist))
    {
        bestNext = findNearest(other, n, (1+level)%dim, bestNew, newBestDist);
        if(bestNext != nullptr)
        {
            double dist2 = calDistNode(bestNext->node, n);
            if(dist2 < newBestDist)
            {
                newBestDist = dist2; 
                bestNew     = bestNext;
            }
        }
    }
    
    return bestNew;
}

void kdTreeNode::Nearby(const kdNodePtr& r,
                        const Node& n, 
                        const int level, 
                        const double& thres,
                        std::vector<kdNodePtr>& nearbyPtrVec)
{
    if(r == nullptr) return; 

    double dist = calDistNode(r->node, n); 

    if(dist < thres)
    {
        nearbyPtrVec.push_back(r);
    }

    kdNodePtr section = r->right, other = r->left; 
    if((level == 0 && n.state.x < r->node.state.x) || (level == 1 && n.state.y < r->node.state.y))
    {
        section = r->left; 
        other   = r->right; 
    }
    
    Nearby(section, n, (1+level)%dim, thres, nearbyPtrVec);

    if((level == 0 && abs(n.state.x - r->node.state.x) <= thres) || (level == 1 && abs(n.state.y - r->node.state.y) <= thres))
    {
        Nearby(other, n, (1+level)%dim, thres, nearbyPtrVec);
    }

    return;
}

/*
kdNodePtr kdTreeNode::getParentNode(const kdNodePtr& r, 
                                    const Node& n, 
                                    const int level)
{
    return r;
}
*/
/*********************************************************************************/

/**
* Belows are the code for debugging
*/

/*
std::default_random_engine generator(time(0));
std::uniform_real_distribution<double> distribution(0,1);

Node RandomPoint()
{   
    Node q_new; 
    q_new.state.x = 100*distribution(generator)+(0-100/2);
    q_new.state.y = 100*distribution(generator)+(0-100/2);
    q_new.state.RandomState(distribution(generator));
    q_new.input = 0;
    q_new.cost = 0;
    return q_new;
}

int main()
{
    Node q_origin; 
    q_origin.state.x = 25.0, q_origin.state.y = 23.6, q_origin.state.RandomState(0.2);
    q_origin.input = 0, q_origin.cost = 0; 

    kdTreeNode tree(q_origin);
    kdNodePtr p1 = tree.getRootPtr(), p2; 

    for(int i = 0; i < 8000; i++)
    {
        Node q = RandomPoint();
        // std::cout << "[" << q.state.x << "," << q.state.y << "]" << std::endl;
        p2 = tree.insert(q);
        // p2->parent = p1; 
        // p1 = p2;         
        // q = p2->parent->node; 
        // std::cout << "[" << q.state.x << "," << q.state.y << "]" << std::endl;
        // std::cout << std::endl;
    }

    Node tmp = RandomPoint();
    queue<kdNodePtr> q; 
    q.push(tree.getRootPtr()); 
    kdNodePtr a = q.front();
    double dist = calDistNode(a->node, tmp);

    clock_t st = clock();

    while(!q.empty())
    {
        size_t n = q.size(); 
        for(size_t i = 0; i < n; i++)
        {
            kdNodePtr b = q.front(); 
            q.pop(); 
            double tmpDist = calDistNode(tmp, b->node);
            if(tmpDist < dist){
                a = b; 
                dist = tmpDist; 
            }
            if(b->left) q.push(b->left); 
            if(b->right) q.push(b->right);
        }
    }
    std::cout << clock()-st << std::endl;
    
    clock_t st2 = clock();
    Node aa = tree.findNearestPoint(tmp); 
    std::cout << clock()-st2 << std::endl;
    
    std::cout << std::endl << "[" << tmp.state.x << "," << tmp.state.y << "]" << std::endl;
    std::cout << "[" << a->node.state.x << "," << a->node.state.y << "]" << std::endl;
    std::cout << "[" << aa.state.x << "," << aa.state.y << "]" << std::endl;

    std::cout << calDistNode(tmp, a->node) << std::endl;
    std::cout << calDistNode(tmp, aa) << std::endl;


    // tree.printTree();
    return 0;
}
*/