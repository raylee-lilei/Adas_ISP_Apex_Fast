//
// Created by wangxiaobao on 18-9-7.
//

#ifndef ADASPROJECT_UTIL_H
#define ADASPROJECT_UTIL_H

#include "define.h"

#define INVALID_DISTANCE 200.0

class Util {
public:
    static long sysTimeMs();
    static long sysTimeUs();
    static int  abs(int p1,int p2);
    static float getDistance(int yPos);
    static cv::Mat YUV2RGB(cv::Mat& mat);
    static cv::Mat RGB2YUV(cv::Mat& mat);
    static float getTanByAngle(int angle);
};

class UtilTimeDiff{
private:
    long time;
    string title;
public:
    UtilTimeDiff(string title);
    ~UtilTimeDiff();
};


class UtilMutexAutoLock {
    pthread_mutex_t *pLock;
public:
    UtilMutexAutoLock(pthread_mutex_t &mutex) {
        pLock = &mutex;
        pthread_mutex_lock(pLock);
    }

    ~UtilMutexAutoLock() {
        pthread_mutex_unlock(pLock);
    }
};
#endif //ADASPROJECT_UTIL_H
