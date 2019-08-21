# RRT_in_cpp
This is the sister repository to RRT_Experimentation. RRTs implemented in C++  
# Command line in terminal
g++ -std=c++11 -o a.o dynamics.cpp planner.cpp kdTree.cpp visualizer.cpp `pkg-config opencv --cflags --libs`
# g++ version : 
g++ (Ubuntu 4.8.5-4ubuntu2) 4.8.5, opencv 2.4.8