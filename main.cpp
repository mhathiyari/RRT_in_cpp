#include "common.hpp"
#include "planner.hpp"
#include "ppc.hpp"

#define VIZ

int main()
{
    planner_params A;
    A.origin = Point(400,- 400);
    A.goal   = Point(-400, 400);
    MatrixXd obstacle(5,4);

    // Simple Rectangle obstacle
    // obstacle.row(0) << 1,1,-1,1;
    // obstacle.row(1) << -1,1,-1,-1;
    // obstacle.row(2) << -1,-1,1,-1;
    // obstacle.row(3) << 1,-1,1,1;
    // A.obstacle = 100*obstacle;

    // Maze Map
    obstacle.row(0) << 50,0,50,150;
    obstacle.row(1) << 150,200,150,100;
    obstacle.row(2) << 150,100,100,100;
    obstacle.row(3) << 125,0,125,50;
    obstacle.row(4) << 200,75,150,75;
    A.obstacle = obstacle-(100*MatrixXd::Ones(5,4));

    A.obstacle.col(0) = -1*A.obstacle.col(0);
    A.obstacle.col(2) = -1*A.obstacle.col(2);
    A.obstacle *= 5; 

    A.iterations = 8000;
    A.width      = 1000; 
    A.height     = 1000;
    A.goalProx   = 15;

    Planner p(A);
    
    p.RRTstar();
    
    Path path; 
    p.ExtractPath(path);
    reverse(path.cx.begin(), path.cx.end()); 
    reverse(path.cy.begin(), path.cy.end());

    double targetSpeed;
    std::cout << "Enter target speed between 5 and 30: " << std::endl;
    std::cin >> targetSpeed; 
    while(true){
        if(std::cin.fail() || targetSpeed < 5.0 || targetSpeed > 30.0){
            std::cin.clear(); 
            std::cin.ignore(std::numeric_limits<streamsize>::max(), '\n');
            std::cout << "Enter target speed between 5 and 30: " << std::endl;
            std::cin >> targetSpeed;
        }else if(!std::cin.fail()){
            break;
        }
    }

     
    // double T = 100.0

    ppc car(A.origin.x, A.origin.y, -M_PI, 0.0); 
    int lastIndex = path.cx.size()-1, currentIndex = 0; 
    // double mTime = 0.0; 
    std::vector<double> x = {car.st.x};
    std::vector<double> y = {car.st.y};
    std::vector<double> theta = {car.st.theta};
    std::vector<double> v = {car.st.v};
    // std::vector<double> t = {mTime};

    #ifdef VIZ
    Visualizer viz; 
    viz.plannerParamsIn(A);
    #endif

    while(lastIndex > currentIndex){
        std::vector<double> ret = car.implementPPC(path, targetSpeed, currentIndex);
        currentIndex = ret[0];
        // mTime += car.dt; 
        // std::cout << currentIndex << ": " << car.st.x << " " << car.st.y; 
        // std::cout << " " << car.st.theta << " " << car.st.v << std::endl;
        // std::cout << "path: " << path.cx[currentIndex] << " " << path.cy[currentIndex]; 
        // std::cout << " distance: " << calDist(car.st.x, car.st.y, path.cx[currentIndex], path.cy[currentIndex]); 
        // std::cout << std::endl << std::endl;
        x.push_back(car.st.x); 
        y.push_back(car.st.y); 
        v.push_back(car.st.theta); 
        theta.push_back(car.st.theta);

        #ifdef VIZ
        plt::clf(); 
        // plt::figure_size(1200, 780); 
        // plt::xlim(-10, 60); 
        // plt::ylim(-25, 25); 
        plt::named_plot("path", path.cx, path.cy, "k-");
        plt::named_plot("Tracking", x,  y,  "go");
        plotCar(car.st.x, car.st.y, car.st.theta);
        viz.drawObstacle();
        // plt::named_plot("Traj_woPF", xWoLoc, yWoLoc, "c*");

        // plt::named_plot("pfTraj", pfX, pfY, "co");
        plt::title("PurePursuitControl"); 
        plt::legend(); 
        plt::pause(0.001);
        plt::xlim(-A.width/2-50, A.width/2+50);
        plt::ylim(-A.height/2-50, A.height/2+50);
        // plt::axis("equal");
        #endif
    }
    plt::show();
    return 0;
}