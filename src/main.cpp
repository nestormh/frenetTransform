#include <iostream>
#include <vector>
#include <stdio.h>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>

#include "frenetplotter.h"
#include "frenettransform.h"

using namespace std;

int main(int argC, char * argV[]) {
    
    std::vector<geometry_msgs::PoseStamped> global_plan(1000);
    
    double factor = 0.1;
    for (uint32_t i = 0; i < 1000; i++) {
        global_plan[i].pose.position.x = i * factor;
        global_plan[i].pose.position.y = (i * factor / 5.0) * (i * factor / 5.0);
    }
  
    frenet_transform::FrenetTransform frenet(20.0, 10.0, 5.0, (uint32_t)10, (uint32_t)200);
    frenet.setGlobalPlan(global_plan);
    frenet.generatePaths(0.0, 10.0, 3.14 / 4, 0);
    
    return 0;
}