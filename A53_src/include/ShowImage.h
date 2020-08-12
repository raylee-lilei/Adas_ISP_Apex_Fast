//
// Created by wangxiaobao on 18-8-31.
//

#ifndef ADASPROJECT_SHOWIMAGE_H
#define ADASPROJECT_SHOWIMAGE_H

#include "define.h"

class ShowImage {
public:
    static void showImage(const char* title,cv::Mat& mat);
    static void showImage(const char* title,cv::Mat& mat,int waitTime);
};


#endif //ADASPROJECT_SHOWIMAGE_H
