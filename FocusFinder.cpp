//
//  FocusFinder.cpp
//  katetest
//
//  Created by Katherine Pachal on 27.10.17.
//  Copyright © 2017 Katherine Pachal. All rights reserved.
//

#include "FocusFinder.hpp"

#include <thread>
#include <chrono>
#include <algorithm>
#include <math.h>

FocusFinder::FocusFinder(VideoCapture * cap)
{
	m_cap = cap;

	// MOVE THESE VALUES TO A CONFIG FILE
	m_dTable = -120.0; // mm relative to home at which gantry table is in focus
	m_stepLarge = 2.0; //mm to move in large scale search for focus
    m_stepSmall = 0.05; // mm to move when near the focus region

	// Default values for focusing algorithms
	m_focusAlg = LAPV;
}


FocusFinder::~FocusFinder()
{
}

void FocusFinder::Focus() {

    // Going to iteratively move around until we are in focus.
    // Roughly follow algorithm in [this paper]

    // For displaying frames as we go
    namedWindow("status",1);

    // Store what focus regime we are in
    bool foundFocus = false;
    enum focusRegimes {IF, OOF};
    focusRegimes regime = OOF;
  
    // Sign of z movement. For our algorithm we
    // start close to the table and move up, since
    // we know the lowest we need to be to focus.
    int dir = +1;
  
    // Use edge counting in OOF routine.
    SetAlg(CANNY);
  
    // Keep previous number to compare to
    double prevMeasure = -1;
    while (!foundFocus) {
    
    
        // First: retrieve focus measure for what we are
        // looking at.
        // And, show us what it looks like!
        Mat frame;
        m_cap->read(frame);
        imshow("status", frame);
        double focus_measure = ComputeFocus(frame);
      
        // If this is the first movement, just store itself as previous
        // measure to compare to.
        if (prevMeasure < 0) prevMeasure = focus_measure;
      
        // If we have actually decreased in focus quality,
        // it means we are stepping out of interesting area.
        // Stop and go carefully backwards in IF regime.
        if (focus_measure < prevMeasure) {
          std::cout << "prev, now: " << prevMeasure << " " << focus_measure << std::endl;

          if (regime == OOF) {

            // TODO:
            // add some buffer epsilon to make sure
            // this is meaningful change
            std::cout << "Reversing direction and switching to IF regime algorithm" << std::endl;
            regime = IF;
            dir = -1 * dir;
            SetAlg(LAPV);
          
            // Also reset focus measure to new quantity
            // because comparing number
            // of edges to Laplacian isn't useful.
            focus_measure = ComputeFocus(frame);
            
          } else {
            
            // If we are in the in-focus region,
            // this means we are there!
            
            // If performance not good enough, allow one additional
            // change of direction.
            std::cout << "Focus found!" << std::endl;
            foundFocus = true;
            continue;
          }
        }
      
        // Set step size. For in focus region we just move by
        // a small amount.
        double stepSize = m_stepSmall;
        // In out of focus region we use an adaptive step size.
        // Compute it from relative differences.
        if (regime == OOF) {

            // Calculate how far off we think we are from finding
            // our in-focus region.
            double relativeDiff = fabs(focus_measure-prevMeasure)/std::max(focus_measure,prevMeasure);
            // How do we use this to scale?
            switch (int(relativeDiff*8.0)) {
              // 0 <= relativeDiff < 1/8
              // Not much difference: move far
              case 0 :
                  stepSize = m_stepLarge * 2.0;
                  break;
              // 1/8 <= relativeDiff < 1/4
              // A bit different. Keep to standard step size.
              case 1 :
                  stepSize = m_stepLarge;
                  break;
              // 1/4 <= relativeDiff < 1/2
              // These are different enough to slow down.
              case 2 :
              case 3 :
                  stepSize = m_stepLarge/2.0;
              default :
                  stepSize = m_stepLarge/3.0;
            } // end of switch
        } else {
          stepSize = m_stepSmall;
        }
    
        // Move by step size in desired direction and rest a
        // little to let camera refocus.

        // MOVE
        std::cout << "Moving by " << stepSize*dir << std::endl;
      
        // REST
        std::cout << "Resting" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        if(waitKey(30) >= 0) break;
      
        // Store this measure for reference next time
        prevMeasure = focus_measure;

    }


}

double FocusFinder::ComputeFocus(Mat img) {

	// Step 1: convert to greyscale
	img = EnsureGrayscale(img);

	// Step 2: blur image
	img = Blur(img);

	// Step 3: compute measure of focus.
	double focusMeasure;
	switch (m_focusAlg) {
		case LAPV:	focusMeasure = varLaplacian(img);
					break;
		case LAPM:	focusMeasure = modLaplacian(img);
					break;
        case CANNY: focusMeasure = edgeCount(img);
                    break;
	}

	std::cout << "Got focus measure " << focusMeasure << " using algorithm " << m_focusAlg << std::endl;

	return focusMeasure;

}

Mat FocusFinder::EnsureGrayscale(Mat img) {

	Mat dest;
	Mat bgr[3];
	Mat img_grey;
	split(img, bgr);
	absdiff(bgr[0],bgr[1], dest);

	if (countNonZero(dest)) {
		cvtColor(img, img_grey, CV_BGR2GRAY);
	}
	else {
		img_grey = img;
	}

	return img_grey;
}

Mat FocusFinder::Blur(Mat img) {

	GaussianBlur(img,img,Size(7,7),1.5,1.5,BORDER_DEFAULT);

	return img;
}

double FocusFinder::modLaplacian(Mat img) {

	int ddepth = img.depth();

	Mat M = (Mat_<double>(3, 1) << -1, 2, -1);
	Mat G = cv::getGaussianKernel(3, -1, ddepth);

	Mat Lx;
	sepFilter2D(img, Lx, ddepth, M, G);

	Mat Ly;
	sepFilter2D(img, Ly, ddepth, G, M);

	Mat FM = abs(Lx) + abs(Ly);

	double focusMeasure = mean(FM).val[0];
	return focusMeasure;
}

double FocusFinder::varLaplacian(Mat img) {

	Mat temp;
	Scalar  median, sigma;

	// Apply Laplacian operator
	int kernel_size = 3;
	//int scale = 1;
	//int delta = 0;
	int ddepth = img.depth();
	cv::Laplacian(img, temp, ddepth, kernel_size);// , scale, delta, cv::BORDER_DEFAULT);
	cv::meanStdDev(temp, median, sigma); //mean,output_value);

	//return variance of the laplacian
	double focusMeasure = sigma.val[0]*sigma.val[0];
	return focusMeasure;
}

double FocusFinder::edgeCount(Mat img) {

    // Apply Canny edge detection
    Canny(img, img, 0, 30, 3);

    //Count the number of pixel representing an edge
    int nCountCanny = countNonZero(img);

    return nCountCanny;
  
}
