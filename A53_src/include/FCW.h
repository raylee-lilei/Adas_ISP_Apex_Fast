//
// Created by wangxiaobao on 18-9-5.
//

#ifndef ADASPROJECT_FCW_H
#define ADASPROJECT_FCW_H

#include "define.h"
#include <opencv2/core/mat.hpp>

class MediaCapture;

class FCW {
private:
    cv::Mat oriImage;
    MediaCapture *mediaCapture;
    static cv::CascadeClassifier detectorClassifier;
    static cv::Mat result;
    static pthread_mutex_t mutex;

    static int rowOffset;
    static int colOffset;
    static int roiWidth;
    static int roiHeight;
    vector<cv::Rect> merge(vector<cv::Rect>& first,vector<cv::Rect>& second);
    float getDistance(int yPos);
    float getTTC(float dis);
    cv::Mat getROI();
    void kickOverlap(vector<cv::Rect> &first);

    int checkAlarm(int x, int width, float ttc);

    int detectPeople(cv::Mat& mat);
public:
    FCW(MediaCapture *mediaCapture,cv::Mat& mat);
    void process();
    static void drawResult(cv::Mat&mat);
};


#endif //ADASPROJECT_FCW_H
