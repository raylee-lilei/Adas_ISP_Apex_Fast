//
// Created by wangxiaobao on 18-8-23.
//

#ifndef ADASPROJECT_MEDIACAPTURE_H
#define ADASPROJECT_MEDIACAPTURE_H

#include "define.h"
#include "LDWS.h"
#include "DSM.h"
#include "TSR.h"
#include "FCW.h"

class MediaCapture {
private:
    cv::VideoCapture videoCapture;
    int devId;
    DSM mDsm;
    static cv::Mat processMatLdws;
    static cv::Mat processMatTsr;
    static cv::Mat processMatFcw;

    LDWS *pmLdws;

    bool isARMLocalCamera;

    static MediaCapture * inst;
public:
    LDWS* getLdwsInstance();
    void syncShow(const char* title,cv::Mat& mat);
    static MediaCapture * instance();
    MediaCapture();
    ~MediaCapture();
    bool open(int devId);
    bool open(const string& fileName);
    void loop();

    void ldwsProcessed(LDWS& ldws);
    void tsrProcessed(TSR& tsr);
    void fcwProcessed(FCW& tsr);

    bool isLdwsThreadIsRun;
    bool isTsrThreadIsRun;
    bool isFcwThreadIsRun;
    pthread_t ldwsThreadId;
    pthread_t tsrThreadId;
    pthread_t fcwThreadId;
    static void * LdwsThread(void* args);
    static void * TsrThread(void* args);
    static void * FcwThread(void* args);

    void startLdwsThread(cv::Mat&mat);
    void startTsrThread(cv::Mat&mat);
    void startFcwThread(cv::Mat&mat);
    void ApexFast(cv::Mat&mat);
};


#endif //ADASPROJECT_MEDIACAPTURE_H
