#include "common.hpp"
#include "planner.hpp"

#define VIZ
double g02(double t)
{
	return 1.0/2.0*(t-1.0)*(t-1.0);
}

double g12(double t)
{
	return 1.0/2.0*(-2.0*t*t+2*t+1.0);
}

double g22(double t)
{
	return 1.0/2.0*t*t;
}

Point pkn(double x0,double y0,double x1,double y1,double x2,double y2,double t)
{
	Point interp_poi;
	interp_poi.setvalue(x0*g02(t)+x1*g12(t)+x2*g22(t),y0*g02(t)+y1*g12(t)+y2*g22(t));
	return interp_poi;
}


Path B_Spline(std::vector<double> x,std::vector<double> y)
{
	std::vector<double> newx,newy;
	double t=0;
	newx.push_back(x[0]);
	newy.push_back(y[0]);

	Point body,tail;
	double cos_body_tail;

	for(int i=0;i<=x.size()-3;i++)
	{
		body.setvalue(x[i+1]-x[i],y[i+1]-y[i]);
		tail.setvalue(x[i+2]-x[i+1],y[i+2]-y[i+1]);
		cos_body_tail=(tail.x*body.x+tail.y*body.y)/(pow((tail.x*tail.x+tail.y*tail.y)*(body.x*body.x+body.y*body.y),0.5));
		if(cos_body_tail<0)
		{
			newx.push_back(x[i+1]);
            newy.push_back(y[i+1]);
			continue;
		}
		for(int j=-1;j<POINT_NUM_IN_ONE_CURVE;j++)
		{
			t=1.0/double(POINT_NUM_IN_ONE_CURVE)*double(j+1);
			Point temp(pkn(x[i],y[i],x[i+1],y[i+1],x[i+2],y[i+2],t));
            newx.push_back(temp.x);
            newy.push_back(temp.y);
		}	
	}
    newx.push_back(x.back());
    newy.push_back(y.back());
    Path result;
    result.cx = newx;
    result.cy = newy;
	return result;
}

int main()
{
    PlannerParams A;
    A.origin = Point(-50,-50);
    A.goal   = Point(50, -50);

    //Simple Rectangle obstacle
    // MatrixXd obstacle(4,4);
    // obstacle.row(0) << 1,1,-1,1;
    // obstacle.row(1) << -1,1,-1,-1;
    // obstacle.row(2) << -1,-1,1,-1;
    // obstacle.row(3) << 1,-1,1,1;
    // A.obstacle = 100*obstacle;

    // // Maze Map
    // MatrixXd obstacle(5,4);
    // obstacle.row(0) << 50,0,50,150;
    // obstacle.row(1) << 150,200,150,100;
    // obstacle.row(2) << 150,100,100,100;
    // obstacle.row(3) << 125,0,125,50;
    // obstacle.row(4) << 200,75,150,75;
    // A.obstacle = obstacle-(100*MatrixXd::Ones(5,4));

    // A.obstacle.col(0) = -1*A.obstacle.col(0);
    // A.obstacle.col(2) = -1*A.obstacle.col(2);
    // A.obstacle *= 5; 

    //Simple parking lot 
    MatrixXd obstacle(1,4);
    obstacle.row(0)<< 0,0,0,-100;

    A.obstacle = obstacle;

    A.iterations = 50000;
    A.width      = 200; 
    A.height     = 200;
    A.goalProx   = 5;

    Planner p(A);
    
    p.RRTstar();
    
    Path path; 
    p.ExtractPath(path);
    reverse(path.cx.begin(), path.cx.end()); 
    reverse(path.cy.begin(), path.cy.end());

    // vector<double> distance(path.cx.size());
    // distance[0] = 0;
    // for(int i = 1;i<distance.size();i++){
    //     distance[i] = calDist(path.cx[i],path.cy[i],path.cx[i-1],path.cy[i-1]);
    // }

    Path smooth;
    smooth = B_Spline(path.cx,path.cy);
    std::cout<<"Path cx cy"<<path.cx.size()<<std::endl;
    std::cout<<"smooth cx cy"<<smooth.cx.size()<<std::endl;
    p.DrawsmoothPath(smooth);


    // double targetSpeed = 10.0/3.6; 
    // // double T = 100.0

    // ppc car(A.origin.x, A.origin.y, -M_PI, 0); 
    // int lastIndex = path.cx.size()-1, currentIndex = 0; 
    // // double mTime = 0.0; 
    // std::vector<double> x = {car.st.x};
    // std::vector<double> y = {car.st.y};
    // std::vector<double> theta = {car.st.theta};
    // std::vector<double> v = {car.st.v};
    // // std::vector<double> t = {mTime};

    // std::cout << lastIndex << std::endl;

    // while(lastIndex > currentIndex){
    //     std::vector<double> ret = car.implementPPC(path, targetSpeed, currentIndex);
    //     currentIndex = ret[0];
    //     // mTime += car.dt; 
    //     std::cout << currentIndex << std::endl;
    //     x.push_back(car.st.x); 
    //     y.push_back(car.st.y); 
    //     v.push_back(car.st.theta); 
    //     theta.push_back(car.st.theta);

    //     #ifdef VIZ
    //     plt::clf(); 
    //     // plt::figure_size(1200, 780); 
    //     // plt::xlim(-10, 60); 
    //     // plt::ylim(-25, 25); 
    //     plt::named_plot("path", path.cx, path.cy, "r-");
    //     plt::named_plot("Traj_wPF", x,  y,  "b*");
    //     // plt::named_plot("Traj_woPF", xWoLoc, yWoLoc, "c*");

    //     // plt::named_plot("pfTraj", pfX, pfY, "co");
    //     plt::title("PurePursuitControl"); 
    //     plt::legend(); 
    //     plt::pause(0.001);
    //     #endif
    // }

    return 0;
}