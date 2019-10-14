#include "common.hpp"

int main(){
    double l = 5.1, w = 3.2; 
    double theta = M_PI / 10; 

    double x = 100.0, y = 100.0; 
    double x1 = x + l/2, x2 = x1 - l; 
    double y1 = y + w/2, y2 = y1 - w; 
    // for(int i = 0; i < 4; i++){
        // plotPoint(x1, y1, ".r");
        // plotPoint(x1, y2, ".b");
        // plotPoint(x2, y1, ".y");
        // plotPoint(x2, y2, ".g");

        // plotPoint(x + l/2*cos(theta)-w/2*sin(theta), y + l/2*sin(theta)+w/2*cos(theta), ".r");
        // plotPoint(x + l/2*cos(theta)+w/2*sin(theta), y + l/2*sin(theta)-w/2*cos(theta), ".b");
        // plotPoint(x - l/2*cos(theta)-w/2*sin(theta), y - l/2*sin(theta)+w/2*cos(theta), ".y");
        // plotPoint(x - l/2*cos(theta)+w/2*sin(theta), y - l/2*sin(theta)-w/2*cos(theta), ".g");

        
    // }
    plotCar(x, y, theta);
    plotCar(x, y, 0);
    plt::axis("equal");
    plt::show();
}