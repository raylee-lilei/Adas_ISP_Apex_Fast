//
// Created by wangxiaobao on 18-8-29.
//

#ifndef ADASPROJECT_TSR_H
#define ADASPROJECT_TSR_H
#define PARAM_PARAM_2 50
#include "define.h"


class MediaCapture;
class TSR {
    cv::Mat oriImage;
    cv::Mat removeNoise(cv::Mat mat);
    void adjustTopBtm(cv::Mat& mat);
    void adjustWhiteBlack(cv::Mat& mat);
    vector<int> getDigitRect(cv::Mat& mat);
    int  getLimitSpeedPredict(vector<int> &xPoint, cv::Mat &mat);
    void calcOkRate();
    static int showTsrCount;
    static cv::Mat showLastSpeed;
    static pthread_mutex_t mutex;
    int getResultPredict(int* data,int count);
    MediaCapture *mMediaCapture;

    int process(cv::Rect& roi);

public:
    TSR(MediaCapture *mediaCapture,cv::Mat& mat);
    void process();
    int speedLimitPredict;
    static void drawResult(cv::Mat& mat);
};


#endif //ADASPROJECT_TSR_H
