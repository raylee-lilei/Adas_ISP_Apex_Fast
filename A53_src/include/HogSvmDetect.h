/*
 * HogSvmDetect.h
 *
 *  Created on: 2019年2月18日
 *      Author: wangxiaobao
 */

#ifndef NXPADAPTER_HOGSVMDETECT_H_
#define NXPADAPTER_HOGSVMDETECT_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/ml.hpp"

class HogSvmDetect {
private:
	void init();
	cv::HOGDescriptor mHog;
	bool mIsInited;
public:
	HogSvmDetect();
	virtual ~HogSvmDetect();

	void foundCar(cv::Mat&img,std::vector<cv::Rect> &found);
};

#endif /* NXPADAPTER_HOGSVMDETECT_H_ */
