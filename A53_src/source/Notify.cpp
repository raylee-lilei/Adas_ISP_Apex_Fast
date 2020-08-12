//
// Created by wangxiaobao on 18-8-28.
//

#include "../include/Notify.h"
#include "../include/path.h"
#include "../include/Util.h"
#include "../include/MessageType.h"
#include "../include/Message.h"
#include <unistd.h>
#include <sys/types.h>

#ifdef _ENV_DESKTOP
#define NOTIFY_COUNT 35
#else
#define NOTIFY_COUNT 20
#endif
#define INVALID_MSG_ID -1

set<HHSQ_ADAS_MESSAGE_S> Notify::notifyList;

int  Notify::notifyCnt = 0;
cv::Mat  Notify::showMat;
pthread_mutex_t Notify::mutex;


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
void * Notify::SoundThread(void *args) {

    int beforeShowImage = -123;
    for(;;){
        if (notifyList.empty()) {
            sleep(1);
            continue;
        }

        HHSQ_ADAS_MESSAGE_S message;
        if (true) {
            memset(&message,0,sizeof(message));
            UtilMutexAutoLock lock(mutex);
            message = *notifyList.begin();
            notifyList.erase(notifyList.begin());
        }

        Message::instance()->send(message);

        cv::Mat notifyMat;
        long type = message.type;
        int command = message.command;
        unsigned char data = message.content[0];

        //如果已经取到了，直接用上次读到的图
        int showFlag = type * 1000 + command * 100 + data;
        if (showFlag == beforeShowImage) {
            notifyCnt = NOTIFY_COUNT;
            continue;
        }

        switch(type){
            case HHSQ_ADAS_TYPE_LDW:
                if (command == HHSQ_COMMAND_LDWS_ALARM) {
                    if (data == HHSQ_LDWS_SHIFT_RIGHT) {
                        notifyMat = cv::imread(PATH_IMAGE_LANE_RIGHT);
                    } else {
                        notifyMat  = cv::imread(PATH_IMAGE_LANE_LEFT);
                    }
                }
                break;
            case HHSQ_ADAS_TYPE_FCW:
                if (command == HHSQ_COMMAND_FCW_ALARM){
                    if (data == HHSQ_FCW_LEVEL_ONE) {
                        notifyMat = cv::imread(PATH_IMAGE_FCW_ONE);
                    } else {
                        notifyMat = cv::imread(PATH_IMAGE_FCW_TWO);
                    }
                }
                break;
            default:;
        }

        if (notifyMat.empty()){
            continue;
        }

        beforeShowImage = showFlag;

        showMat = notifyMat;
        notifyCnt = NOTIFY_COUNT;
    }
}
#pragma clang diagnostic pop

void Notify::notify(HHSQ_ADAS_MESSAGE_S&msg) {

    static bool isFirst = true;

    if (isFirst) {
        pthread_mutex_init(&mutex, NULL);
        isFirst = false;
        pthread_t thread;
        pthread_create(&thread, NULL, SoundThread, NULL);
    }

    if (true) {
        UtilMutexAutoLock lock(mutex);
        notifyList.insert(msg);
    }
}

void  Notify::drawResult(cv::Mat &mat){
    if (Notify::notifyCnt > 0) {
        Notify::notifyCnt--;
        cv::Mat roi = mat(cv::Rect(mat.cols /2 - Notify::showMat.cols / 2, mat.rows - Notify::showMat.rows  - 10,Notify::showMat.cols,Notify::showMat.rows));
        Notify::showMat = Util::RGB2YUV(Notify::showMat);
        Notify::showMat.convertTo(roi,roi.type(),1,0);
    }

}
