#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>

#include "frenetplotter.h"

using namespace std;

int main(int argC, char * argV[]) {
    
    vector <frenet_transform::Point2d> sinFunc, cosFunc;    
    {
        const unsigned int n = 100;
        sinFunc.resize(n);
        cosFunc.resize(n);
        
        const double fpi = 3.1415926 / 180.0;
        const double step = 360. / (n - 1);
    
        for (int i = 0; i < n; i++) {   
            
            const double xRay = i * step;
            const double x = xRay * fpi;
            
            sinFunc[i] = frenet_transform::Point2d(xRay, sin(x));
            cosFunc[i] = frenet_transform::Point2d(xRay, cos(x));
        }
    }
  
    if (fork() == 0) { 
        frenet_transform::FrenetPlotter plotter("window1");
        plotter.plot(sinFunc, "red");
        plotter.waitForButton();
    } else {
        frenet_transform::FrenetPlotter plotter2("window2");
        plotter2.plot(cosFunc, "green");
        plotter2.waitForButton();
    }
    
    return 0;
}