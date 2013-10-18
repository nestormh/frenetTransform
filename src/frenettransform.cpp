/*
    Copyright 2013 Néstor Morales Hernández <nestor@isaatc.ull.es>,
                   Rafael Arnay del Arco <rafa@isaatc.ull.es>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#include "frenettransform.h"
#include "frenetplotter.h"

#include <vector>

#include <sys/time.h>
#include <stdio.h>

#include <eigen3/Eigen/Dense>

using namespace std;

namespace frenet_transform {
    
    
FrenetTransform::FrenetTransform(const double & pathResolution, const double & distBetweenPaths, 
                                 const double & qMax, const double & sf, const double & sTotal, 
                                 const double idxForTangent)
{
    initialize(pathResolution, distBetweenPaths, qMax, sf, sTotal, idxForTangent);
}

FrenetTransform::FrenetTransform(const double & maxDist, const double & maxWide, const double & evolutionDist,
                                 const uint32_t & numPaths, const uint32_t & numSteps,
                                 const uint32_t idxForTangent)
{
    initialize(maxDist, maxWide, evolutionDist, numPaths, numSteps, idxForTangent);
}
    

void FrenetTransform::initialize(const double & pathResolution, const double & distBetweenPaths, 
                                 const double & qMax, const double & sf, const double & sTotal, 
                                 const uint32_t & idxForTangent)
{
    m_pathResolution = pathResolution;
    m_distBetweenPaths = distBetweenPaths;
    m_qMax = qMax;
    m_idxForTangent = idxForTangent;
    m_sTotal = sTotal;
    m_sf = sf;
}

void FrenetTransform::initialize(const double & maxDist, const double & maxWide, const double & evolutionDist,
                                 const uint32_t & numPaths, const uint32_t & numSteps,
                                 const uint32_t & idxForTangent)
{
    m_sTotal = maxDist;
    m_pathResolution = maxDist / numSteps;
    
    m_qMax = maxWide / 2.0;
    m_distBetweenPaths = maxWide / numPaths;

    
    m_idxForTangent = idxForTangent;
    m_sf = evolutionDist;
    
    cout << "m_sTotal " << m_sTotal << endl;
    cout << "m_pathResolution " << m_pathResolution << endl;
    cout << "m_qMax " << m_qMax << endl;
    cout << "m_distBetweenPaths " << m_distBetweenPaths << endl;
    cout << "m_idxForTangent " << m_idxForTangent << endl;
    cout << "m_sf " << m_sf << endl;
    
}

FrenetTransform::~FrenetTransform()
{

}

void FrenetTransform::generatePaths(const double & si, const double & qi, 
                                    const double & theta, const uint32_t & idxGlobal)
{
    struct timeval start, end;
    
    long mtime, seconds, useconds;    
    
    gettimeofday(&start, NULL);     
    
    m_pathList.resize(2 * m_qMax / m_distBetweenPaths + 1);
    
    uint32_t i = 0;
    for (double qf = -m_qMax; qf <= m_qMax /*i < m_pathList.size()*/; qf += m_distBetweenPaths, i++) {
        cout << i << endl;
        generatePath(qf, si, qi, theta, m_pathList[i]);
    }
    
    transformPaths(idxGlobal);
    
    gettimeofday(&end, NULL);
    
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;
    
    mtime = ((seconds) * 1000 + useconds / 1000.0) + 0.5;
    
    printf("Elapsed time: %ld milliseconds\n", mtime);
    
    plotPaths(m_pathList);
    plotTransformedPaths();
}

inline
void FrenetTransform::generatePath(const double& qf, const double& si, 
                                   const double& qi, const double& theta,
                                   std::vector <double> & q)
{
    double x = m_sf - si;
    double x2 = x * x;
    double x3 = x2 * x;
    
    Eigen::MatrixXf A(3, 3);
    A << x3, x2, x,
        3 * x2, 2 * x, 1,
        0, 0, 1;
    
    Eigen::MatrixXf B(3, 1);
    B << qf - qi,
         0,
         tan(theta);
    
    Eigen::MatrixXf X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
    
    q.resize(ceil(m_sTotal / m_pathResolution), (float)qf);
    
    std::vector <double> dq(ceil(m_sTotal / m_pathResolution), 0.0f);
    std::vector <double> d2q(ceil(m_sTotal / m_pathResolution), 0.0f);
    
    {
        double s = 0;
        for (uint32_t i = 0; i < q.size(); i++, s += m_pathResolution) {
            
            if (s <= m_sf) {
                x = s - si;
                x2 = x * x;
                x3 = x2 * x;
                q[i] = X(0) * x3 + X(1) * x2 + X(2) * x + qi;
//                 dq[i] = 3 * X(0) * x + 2 * X(1) * x + X(2); 
//                 d2q[i] = 6 * X(0) * x + 2 * X(1);
            }
        }
    }
}

void FrenetTransform::transformPaths(const uint32_t & idxGlobal)
{
    m_transformedPaths.resize(m_pathList.size());
    
    double dist = 0;
    for (uint32_t i = idxGlobal; m_global_plan.size(); i++) {
        if (i != 0) {
            dist += sqrt((m_global_plan[i].pose.position.x - m_global_plan[i - 1].pose.position.x) * 
            (m_global_plan[i].pose.position.x - m_global_plan[i - 1].pose.position.x) +
            (m_global_plan[i].pose.position.y - m_global_plan[i - 1].pose.position.y) * 
            (m_global_plan[i].pose.position.y - m_global_plan[i - 1].pose.position.y));
        }
        
        
        if (dist > m_sTotal)
            break;
        
        Point2d dir(m_global_plan[i + m_idxForTangent].pose.position.x - m_global_plan[i].pose.position.x,
                    m_global_plan[i + m_idxForTangent].pose.position.y - m_global_plan[i].pose.position.y);
        
        Point2d normal(-dir.y, dir.x);
        normal.normalize();
        uint32_t index = round(dist / m_pathResolution);
        
        for (uint32_t j = 0; j < m_transformedPaths.size(); j++) {
            //             if (i == 0) {
            //                 TODO: Reservar tamaño para Trajectory
            //             }
            
            if (index > m_pathList[j].size() - 1)
                break;
            
            m_transformedPaths[j].addPoint(m_global_plan[i].pose.position.x  + m_pathList[j][index] * normal.x,
                                           m_global_plan[i].pose.position.y  + m_pathList[j][index] * normal.y,
                                           0.0f);
            
        }
    }
}


void FrenetTransform::setGlobalPlan(const std::vector< geometry_msgs::PoseStamped >& global_plan)
{
    m_global_plan = global_plan;
}

void FrenetTransform::plotPaths(const vector < vector< double > >& paths)
{
    if (fork() == 0) {
        frenet_transform::FrenetPlotter plotter("path");
        plotter.setAxis(-10., 20., 0.0, 10., -10.0, 10.0, 0.0, 10.);
        
        for (vector < vector< double > >::const_iterator it = paths.begin();
                it != paths.end(); it++) {
                
            const vector< double > & path = *it;
                
            vector <frenet_transform::Point2d> path2D(path.size());
            {
                double x = 0.0f;
                for (int i = 0; i < path.size(); i++, x += m_pathResolution) {   
                    
                    path2D[i] = frenet_transform::Point2d(x, path[i]);
                }
            }
            plotter.plot(path2D, "red");
        }
        
        plotter.waitForButton();
    }
}

void FrenetTransform::plotTransformedPaths()
{
//     if (fork() == 0) {
        frenet_transform::FrenetPlotter plotter("TransformedPath");
        plotter.setAxis(-10., 50., 0.0, 10., -10.0, 50.0, 0.0, 10.);
        
        vector <Point2d> global2D(m_global_plan.size());
        for (uint32_t i = 0; i < m_global_plan.size(); i++) {
                        
            global2D[i] = Point2d(m_global_plan[i].pose.position.x, m_global_plan[i].pose.position.y);
        }
        plotter.plot(global2D, "blue");
        
        for ( vector < Trajectory >::iterator it = m_transformedPaths.begin(); 
                    it != m_transformedPaths.end(); it++) {
            
            vector <Point2d> path2D(it->getPointsSize());
            {
                for (int i = 0; i < path2D.size(); i++) { 
                    double x, y, th;
                    it->getPoint(i, x, y, th);
                    path2D[i] = Point2d(x, y);
                }
            }
        
            plotter.plot(path2D, "green");
        }
        
        plotter.waitForButton();
//     }
}


}