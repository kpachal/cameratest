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

FocusFinder::FocusFinder(VideoCapture * cap)
{
	m_cap = cap;

	// MOVE THESE VALUES TO A CONFIG FILE
	m_dTable = -120.0; // mm relative to home at which gantry table is in focus
	m_stepLarge = 2.0; //mm to move in large scale search for focus

	// Default values for focusing algorithms
	m_focusAlg = LAPV;
}


FocusFinder::~FocusFinder()
{
}

void FocusFinder::Focus() {


    Mat edges;
    namedWindow("image",1);
    for(int i=0; i<5; i++)
    {
        Mat frame;
        m_cap->read(frame); // get a new frame from camera
        double focus_measure = ComputeFocus(frame);
//        cvtColor(frame, edges, COLOR_BGR2GRAY);
//        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
//        Canny(edges, edges, 0, 30, 3);
        imshow("image", frame);

        std::cout << "resting" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        if(waitKey(30) >= 0) break;
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
	}

	std::cout << "Got focus measure " << focusMeasure << std::endl;

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
	int scale = 1;
	int delta = 0;
	int ddepth = img.depth();
	cv::Laplacian(img, temp, ddepth, kernel_size);// , scale, delta, cv::BORDER_DEFAULT);
	cv::meanStdDev(temp, median, sigma); //mean,output_value);

	//return variance of the laplacian
	double focusMeasure = sigma.val[0]*sigma.val[0];
	return focusMeasure;
}
