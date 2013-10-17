/*
 *  Copyright 2013 Néstor Morales Hernández <nestor@isaatc.ull.es>,
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

#include "frenetplotter.h"

#include <iostream>
#include <cmath>

#include <iostream>
#include <stdio.h>

using namespace std;

namespace frenet_transform {

FrenetPlotter::FrenetPlotter(const string & title)
{
    g.metafl ("cons");
    g.scrmod ("revers");
    g.disini ();
    g.pagera ();
    g.complx ();
    g.axspos (450, 1800);
//     g.axslen (2200, 1200);
    g.axslen (1200, 1200);

    g.name   ("X-axis", "x");
    g.name   ("Y-axis", "y");

    g.labdig (-1, "x");
    g.ticks  (9, "x");
    g.ticks  (10, "y");

    g.titlin (title.c_str(), 1);

    int ic=g.intrgb (0.95,0.95,0.95);
    g.axsbgd (ic);
}

void FrenetPlotter::plot(const vector< Point2d >& points, const string & color)
{
    float xray[points.size()];
    float yray[points.size()];
    
    {
        int i = 0;
        for (vector <Point2d>::const_iterator it = points.begin(); it != points.end(); it++, i++) {
            xray[i] = it->x;
            yray[i] = it->y;
        }
    }
    
    g.color  (color.c_str());
    g.curve  (xray, yray, points.size());
}

void FrenetPlotter::setAxis(const double & xStart, const double & xEnd, const double & xOrigin, const double & xStep,
                            const double & yStart, const double & yEnd, const double & yOrigin, const double & yStep) {
    
    g.graf(xStart, xEnd, xOrigin, xStep, yStart, yEnd, yOrigin, yStep);
    
    g.setrgb (0.7, 0.7, 0.7);
    g.grid   (1, 1);
    
    g.color  ("fore");
    g.height (50);
    g.title  ();
}


void FrenetPlotter::waitForButton()
{
    g.disfin ();
}
    
}