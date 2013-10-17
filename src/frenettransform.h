/*
 *  Copyright 2013 Néstor Morales Hernández <nestor@isaatc.ull.es>,
 *                 Rafael Arnay del Arco <rafa@isaatc.ull.es>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 * 
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */


#ifndef FRENETTRANSFORM_H
#define FRENETTRANSFORM_H

#include "geometry_msgs/PoseStamped.h"

#include <grull_ackermann_base_local_planner/trajectory.h>

#include "eigen3/Eigen/Core"

using namespace std;
using namespace grull_ackermann_base_local_planner;

namespace frenet_transform {
    
class FrenetTransform
{

public:
    FrenetTransform();
    
    void setGlobalPlan(const std::vector<geometry_msgs::PoseStamped> & global_plan);
    
    void generatePaths();
    
    void generatePath(const double& qf, const double & s_total, const double & si, const double & sf, 
                      const double & qi, const double & theta, std::vector <double> & q);
    
    void transformPaths(const uint32_t & idxGlobal, const double & s_total);
    
    virtual ~FrenetTransform();
private:
    void plotPaths(const vector < vector< double > >& path);
    void plotTransformedPaths();
    
    vector<geometry_msgs::PoseStamped> m_global_plan;
    
    double m_pathResolution;
    double m_distBetweenPaths;
    double m_qMax;
    
    uint32_t m_idxForTangent;
    
    vector < vector < double > > m_pathList;
    vector<Trajectory> m_transformedPaths;
    
};

}

#endif // FRENETTRANSFORM_H
