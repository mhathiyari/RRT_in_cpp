#include "kdTree.hpp" 

template <class T> 
kdTree<T>::kdTree(const tVec& position)
{
    node = std::make_shared<kdNode>(position); 
    dim  = 2;
    if(!node){
        std::cout << "CONSTRUCTOR ERROR" << std::endl;
    }
}

template <class T> 
void kdTree<T>::insert(const tVec& x)
{
    insert(x, node, 0); 
}

template <class T> 
typename kdTree<T>::tVec kdTree<T>::findNearestPoint(const tVec& x)
{
    return findNearest(x); 
}

template <class T> 
typename kdTree<T>::tVec kdTree<T>::getParent(const tVec& x)
{
    std::shared_ptr<kdNode> p = getParentNode(node, x, 0);
    if(p != nullptr)
    {
        return p->pos; 
    }else
    {
        tVec n; 
        return n;
    }
    
}

template <class T> 
void kdTree<T>::printTree()
{
    std::queue<std::shared_ptr<kdNode>> q; 
    q.push(node); 
    while(!q.empty())
    {
        int n = q.size(); 
        std::cout << "n: " << n << std::endl;
        for(int i = 0; i < n; i++)
        {
            std::shared_ptr<kdNode> r = q.front(); 
            q.pop(); 

            std::cout << "[" << r->pos[0] << "," << r->pos[1] << "] "; 
            
            if(r->left  != nullptr) q.push(r->left ); 
            if(r->right != nullptr) q.push(r->right); 
        }
        std::cout << std::endl;
    }
}

/*****************************************************************/

template <class T> 
T kdTree<T>::calDisttVec(const tVec& a, const tVec& b)
{
    return sqrt(pow(a[0]-b[0],2) + pow(a[1]-b[1],2)); 
}

template <class T> 
bool kdTree<T>::insert(const tVec& x, kdNodePtr& r, int level)
{
    bool ind = false; 
    if(r == nullptr)
    {
        r = std::make_shared<kdNode>(x);
        return true;
    }else if(x[level] < r->pos[level])
    {
        ind = insert(x, r->left,  (1+level)%dim); 
        if(ind)
        {
            r->left->parent = r; 
        }
        return false;
    }else
    {
        ind = insert(x, r->right, (1+level)%dim); 
        if(ind)
        {
            r->right->parent = r;
        }
        return false; 
    }
}

template <class T> 
typename kdTree<T>::tVec kdTree<T>::findNearest(const tVec& x)
{
    T dist = calDisttVec(node->pos, x);
    return (findNearest(node, x, 0, node, dist))->pos;
}


template <class T>
typename kdTree<T>::kdNodePtr kdTree<T>::findNearest(const kdNodePtr& branch,
                                                     const tVec&      position, 
                                                     const int&       level, 
                                                     const kdNodePtr& best,
                                                     const T&    bestDist)  
{
    if(branch == nullptr) return branch;    

    T dist = calDisttVec(branch->pos, position); 

    kdNodePtr bestNew     = best; 
    T         newBestDist = bestDist; 
    if(dist < bestDist)
    {
        newBestDist = dist; 
        bestNew     = branch;
    }

    kdNodePtr section, other; 
    if(position[level] < branch->pos[level])
    {
        section = branch->left; 
        other   = branch->right; 
    }else{
        section = branch->right;
        other   = branch->left;
    }

    kdNodePtr bestNext = findNearest(section, position, (level+1)%dim, bestNew, newBestDist);
    if(bestNext && !(bestNext->pos.empty()))
    {
        T dist2 = calDisttVec(bestNext->pos, position);
        if(dist2 < newBestDist)
        {
            newBestDist = dist2; 
            bestNew     = bestNext; 
        }
    }

    if(abs(position[level] - branch->pos[level]) <= bestDist)
    {
        bestNext = findNearest(other, position, (level+1)%dim, bestNew, newBestDist); 
        if(bestNext && !bestNext->pos.empty())
        {
            T dist2 = calDisttVec(bestNext->pos, position);
            if(dist2 < newBestDist)
            {
                newBestDist = dist2; 
                bestNew     = bestNext; 
            }
        }
    }

    return bestNew; 
} 

template <class T>
typename kdTree<T>::kdNodePtr kdTree<T>::getParentNode(const kdNodePtr& branch, 
                                                       const tVec&      position, 
                                                       const int&       level)
{
    if(branch == nullptr)
    {
        return branch; 
    }
    if(branch->pos[0] == position[0] && branch->pos[1] == position[1])
    {
        return branch->parent; 
    }else if(position[level] < branch->pos[level])
    {
        return getParentNode(branch->left, position, (1+level)%dim);
    }else
    {
        return getParentNode(branch->right, position, (1+level)%dim);
    }
}

/*****************************************************************/
/* Below is for testing kd tree  */

/*
std::vector<std::vector<double>> list = {
                                        {27.88,-9.54},
                                        {-7.65,-5.16},
                                        {-40.92,-13.42},
                                        {-23.35,26.35},
                                        {-34.63,12.79},
                                        {-21.90,27.20},
                                        {-5.99,43.29},
                                        {2.71,47.27},
                                        {-4.26,-30.80},
                                        {37.54,-36.11},
                                        {1.81,19.63},
                                        {44.36,-40.62},
                                        {13.77,2.54},
                                        {45.77,3.03},
                                        {-25.93,36.11},
                                        {17.61,-1.51},
                                        {-21.09,-10.65},
                                        {17.18,17.14},
                                        {19.51,24.13},
                                        {-43.20,2.01},
                                        {-24.52,-15.23},
                                        {-27.60,-35.00},
                                        {16.78,8.61},
                                        {34.44,-23.79},
                                        {-15.55,-45.55},
                                        {28.05,25.49},
                                        {17.53,-25.72},
                                        {-49.33,-5.76},
                                        {10.22,18.78},
                                        {-11.32,-14.08},
                                        {41.60,23.63},
                                        {-49.88,-10.53},
                                        {-3.76,18.34},
                                        {-7.57,20.40},
                                        {-3.91,-5.77},
                                        {27.02,-48.04},
                                        {-17.75,-16.91},
                                        {28.47,-7.57},
                                        {-2.86,-22.97},
                                        {-46.42,-30.29},
                                        {-32.41,32.17},
                                        {22.18,-7.01},
                                        {-2.65,38.78},
                                        {-34.73,-10.88},
                                        {-15.89,26.91},
                                        {10.74,-10.32},
                                        {-30.83,30.85},
                                        {23.84,25.51},
                                        {-25.72,-12.26},
                                        {41.74,-28.40},
                                        {-23.09,29.04},
                                        {26.55,44.93},
                                        {-31.13,-17.24},
                                        {-21.25,17.13},
                                        {-40.89,-6.14},
                                        {7.62,33.35},
                                        {18.34,26.89},
                                        {4.66,-33.27},
                                        {-7.43,36.20},
                                        {14.44,48.99},
                                        {14.76,1.44},
                                        {17.90,38.43},
                                        {13.58,8.80},
                                        {44.52,-34.52},
                                        {-29.11,-30.01},
                                        {20.93,-9.30},
                                        {-26.38,24.87},
                                        {-38.06,32.56},
                                        {10.73,29.00},
                                        {-4.99,-18.15},
                                        {-4.13,3.41},
                                        {16.19,-41.00},
                                        {27.03,-38.83},
                                        {-14.98,-36.37},
                                        {16.20,17.87},
                                        {-8.38,-0.48},
                                        {34.19,-31.03},
                                        {33.29,-0.50},
                                        {-24.36,-35.24},
                                        {11.35,-44.50},
                                        {8.22,35.07},
                                        {4.07,6.06},
                                        {36.99,42.96},
                                        {-23.52,19.67},
                                        {-18.19,8.28},
                                        {-38.08,31.54},
                                        {43.98,37.90},
                                        {14.56,48.89},
                                        {-2.05,-49.95},
                                        {13.93,36.54},
                                        {4.47,11.26},
                                        {14.73,49.00},
                                        {4.39,2.77},
                                        {22.10,-2.05},
                                        {2.25,30.13},
                                        {49.37,-27.22},
                                        {-28.13,-0.19},
                                        {-39.42,40.09},
                                        {-39.03,7.47},
                                        {-43.64,34.52}
                                        };
*/
/*
std::vector<std::vector<double>> list = {{23.86,26.90},
{8.60,8.14},
{-25.33,42.83},
{16.64,8.01},
{-41.65,-48.30},
{12.60,-37.91},
{16.09,36.27},
{22.98,-1.57},
{39.08,34.49},
{48.23,-29.06}};
*/
/*
double di(std::vector<double> a, std::vector<double> b)
{
    return sqrt(pow(a[0]-b[0],2) + pow(a[1]-b[1],2));
}


int main()
{
    kdTree<double> k(list[0]); 

    for(int i = 1; i < list.size(); i++)
    {
        k.insert(list[i]);
    }
    // k.printTree();

    std::vector<double> tmp = {20.0, 20.0}, tmp1, tmp2 = list[0];  
    
    // clock_t st = clock();
    tmp1 = k.findNearestPoint(tmp);
    // std::cout << "Time of kd tree: " << clock()-st << std::endl;

    double dmin = di(tmp, list[0]);
    clock_t st1 = clock();
    for(auto i : list)
    {   
        if(dmin > di(tmp, i))
        {
            dmin = di(tmp, i); 
            tmp2 = i;
        }
    }
    // std::cout << "Time of looping through vector: " << clock()-st1 << std::endl;

    std::cout << "[" << tmp1[0] << "," << tmp1[1] << "]"<< std::endl;
    std::cout << "[" << tmp2[0] << "," << tmp2[1] << "]"<< std::endl;

    tmp2 = k.getParent(list[0]);
    if(!tmp2.empty())
    {
        std::cout << "------------------------------------" << std::endl;
        std::cout << "[" << tmp2[0] << "," << tmp2[1] << "]"<< std::endl;
    }else
    {
        std::cout << "node is the root of the tree!" <<std::endl;
    }
    

    return 0; 
}
*/