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


#ifndef FRENETPLOTTER_H
#define FRENETPLOTTER_H

#include <string>

#include <opencv2/opencv.hpp>

#include "discpp.h"

using namespace std;

namespace frenet_transform {

class FrenetPlotter
{
public:
    FrenetPlotter(const string & title);
    void plot(const vector <cv::Point2d> & points, const string & color);
    void waitForButton();
    
private:
    Dislin g;
};

}

#endif // FRENETPLOTTER_H
