//
// Created by wangxiaobao on 18-10-24.
//

#ifndef ADASPROJECT_ADAPTER_H
#define ADASPROJECT_ADAPTER_H

#include "../include/ErrorCode.h"
#include "../include/define.h"

class Adapter {
protected:
    Adapter(){}
public:
    static Adapter* instance();
    virtual ErrorCode initialize() = 0;
    virtual ErrorCode canny(cv::Mat& src, cv::Mat& dst,double min,double max) = 0;
    virtual void showImage(const char *title, cv::Mat &mat,int waitTime) = 0;
    virtual void gaussianFilter(cv::Mat& src, cv::Mat&dst,int windowSize) = 0;
    virtual void equalizeHist(cv::Mat& src, cv::Mat& dst) = 0;
    virtual void lbpTrain(cv::Mat& src,int count,int width,int height,cv::Mat& model) = 0;
    virtual void lbpPredict(cv::Mat& model,int modelCount,cv::Mat& src,cv::Mat& result) = 0;
    virtual void Rgb2Gray(cv::Mat&src,cv::Mat&dst) = 0;
    virtual void capature(int dev,cv::Mat& mat) = 0;
    virtual void saveVideo(std::string path,cv::Mat& mat) = 0;
    virtual void foundCar(cv::Mat&img,std::vector<cv::Rect> &found) = 0;
};


#endif //ADASPROJECT_ADAPTER_H
