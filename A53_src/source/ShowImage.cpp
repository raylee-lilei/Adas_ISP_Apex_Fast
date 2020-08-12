//
// Created by wangxiaobao on 18-8-31.
//

#include "../include/ShowImage.h"
#include "../include/Adapter.h"

void ShowImage::showImage(const char *title, cv::Mat &mat) {
    showImage(title,mat,1);
}

void ShowImage::showImage(const char *title, cv::Mat &mat,int waitTime) {
#ifdef _ENV_IMAGE_NO_SHOW
    usleep(waitTime*1000);
    return;
#endif

    Adapter::instance()->showImage(title,mat,waitTime);
}
