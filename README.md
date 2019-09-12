# RRT_in_cpp
This is the sister repository to RRT_Experimentation. RRTs implemented in C++  
# Command line in terminal
g++ -std=c++11 -o a.o dynamics.cpp planner.cpp kdTree.cpp visualizer.cpp `pkg-config opencv --cflags --libs`
g++ -std=c++11 -I /usr/local/include/eigen3 -o a.o dynamics.cpp plannerTmp.cpp kdTreeNode.cpp visualizer.cpp `pkg-config opencv --cflags --libs`

# g++ version : 
g++ (Ubuntu 4.8.5-4ubuntu2) 4.8.5, opencv 2.4.8

So for clutered environments espeically the ones where there is a large obstacle between goal and the source RRT performs better than hRRT(hueristic RRT) Example a run for maze with goal bias took ~7000 while normal RRT took ~4000
