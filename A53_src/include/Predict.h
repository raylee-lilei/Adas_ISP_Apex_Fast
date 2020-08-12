//
// Created by wangxiaobao on 18-8-30.
//

#ifndef ADASPROJECT_PREDICT_H
#define ADASPROJECT_PREDICT_H


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

class Predict
{
private:
    static Predict *ins;
    std::vector<cv::Ptr<cv::ml::SVM> > svms;
    std::map<int,std::string> pics;
    Predict();
    void train();
public:
    static Predict* instance();
    void init();
    int predict(cv::Mat& mat);
    static int realSpeed(int speed);
    static int offset();
};

#endif //ADASPROJECT_PREDICT_H
