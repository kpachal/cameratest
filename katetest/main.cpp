//
//  main.cpp
//  katetest
//
//  Created by Katherine Pachal on 27.10.17.
//  Copyright © 2017 Katherine Pachal. All rights reserved.
//

#include <iostream>
#include "opencv2/opencv.hpp"

#include "FocusFinder.hpp"

using namespace cv;
int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    // Make autofocus device
    FocusFinder myFocus(&cap);

    myFocus.Focus();

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
