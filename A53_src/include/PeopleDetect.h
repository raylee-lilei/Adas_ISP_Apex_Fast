//
// Created by wangxiaobao on 19-1-24.
//

#ifndef ADASPROJECT_PEOPLEDETECT_H
#define ADASPROJECT_PEOPLEDETECT_H


#include "define.h"

class PeopleDetect {
private:
    cv::HOGDescriptor hog;
    static PeopleDetect *inst;
public:
    static PeopleDetect * instance();
    void init();
    void detect(cv::Mat& mat,std::vector<cv::Rect>& regions);
};


#endif //ADASPROJECT_PEOPLEDETECT_H
