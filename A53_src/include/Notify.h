//
// Created by wangxiaobao on 18-8-28.
//

#ifndef ADASPROJECT_NOTIFY_H
#define ADASPROJECT_NOTIFY_H

#include <set>
#include "define.h"
#include "Message.h"

typedef enum NOTIFY_TYPE_E_ {
    SHIFT_LEFT = 0,
    SHIFT_RIGHT,
    NOTIFY_DSM,
    NOTIFY_TYPE_END
}NOTIFY_TYPE_E;

class Notify {
private:
    static set<HHSQ_ADAS_MESSAGE_S> notifyList;
    static pthread_mutex_t mutex;
public:
    static int notifyCnt;
    static cv::Mat showMat;
    static void notify(HHSQ_ADAS_MESSAGE_S& msg);
    static void * SoundThread(void *args);
    static void drawResult(cv::Mat &mat);
};

#endif //ADASPROJECT_NOTIFY_H
