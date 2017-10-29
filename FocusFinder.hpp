//
//  FocusFinder.hpp
//  katetest
//
//  Created by Katherine Pachal on 27.10.17.
//  Copyright © 2017 Katherine Pachal. All rights reserved.
//

#ifndef FocusFinder_hpp
#define FocusFinder_hpp

#include <stdio.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

enum FOCUS_ALG
{
	LAPV = 0,
	LAPM = 1,
    CANNY = 2
};

class FocusFinder
{
public:
	FocusFinder(VideoCapture * cap);
	~FocusFinder();

	void Focus();

	double ComputeFocus(Mat img);
  
    void SetAlg(FOCUS_ALG newAlg)
      { m_focusAlg = newAlg; };

protected :

	Mat EnsureGrayscale(Mat img);

	Mat Blur(Mat img);

	double modLaplacian(Mat img);

	double varLaplacian(Mat img);
  
    double edgeCount(Mat img);

	VideoCapture * m_cap;

	// For setting the algorithm to be
	// used to define the degree of focus
	FOCUS_ALG m_focusAlg;

	// MOVE THESE VALUES TO A CONFIG FILE
	double m_dTable;
	double m_stepLarge;
    double m_stepSmall;
  
};
#endif /* FocusFinder_hpp */
