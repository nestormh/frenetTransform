#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>

#include "frenetplotter.h"
#include "frenettransform.h"

using namespace std;

int main(int argC, char * argV[]) {
    
//     vector <frenet_transform::Point2d> sinFunc, cosFunc;    
    std::vector<geometry_msgs::PoseStamped> global_plan(1000);
//     {
//         const unsigned int n = 100;
//         sinFunc.resize(n);
//         cosFunc.resize(n);
//         global_plan.resize(n);
//         
//         const double fpi = 3.1415926 / 180.0;
//         const double step = 360. / (n - 1);
//         
//     
//         for (int i = 0; i < n; i++) {   
//             
//             const double xRay = i * step;
//             const double x = xRay * fpi;
//             
//             sinFunc[i] = frenet_transform::Point2d(xRay, sin(x));
//             cosFunc[i] = frenet_transform::Point2d(xRay, cos(x));
//             
//             global_plan[i].pose.position.x = xRay;
//             global_plan[i].pose.position.y = sin(x);
//         }
//     }
    
    double factor = 0.1;
    for (uint32_t i = 0; i < 1000; i++) {
        global_plan[i].pose.position.x = i * factor;
        global_plan[i].pose.position.y = (i * factor / 5.0) * (i * factor / 5.0);
    }
  
    frenet_transform::FrenetTransform frenet;
    frenet.setGlobalPlan(global_plan);
    frenet.generatePaths();
    
//     if (fork() == 0) { 
//         frenet_transform::FrenetPlotter plotter("window1");
//         plotter.setAxis(-360., 600., 0.0, 45., -2.0, 2.0, 0.0, 0.25);
//         plotter.plot(sinFunc, "red");
//         plotter.waitForButton();
//     } else {
//         frenet_transform::FrenetPlotter plotter2("window2");
//         plotter2.setAxis(0.0, 360.0, 0.0, 90.0, -1.0, 1.0, -1.0, 0.5);
//         plotter2.plot(cosFunc, "green");
//         plotter2.waitForButton();
//     }
    
    return 0;
}