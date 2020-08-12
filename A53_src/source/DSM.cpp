//
// Created by wangxiaobao on 18-8-28.
//

#include "../include/DSM.h"
#include "../include/Notify.h"


DSM::DSM() {

}

DSM::~DSM() {

}

void DSM::setShiftState(bool isLeftShift,bool isRightShift)
{
    time_t now;
    if (isLeftShift) {
        time(&now);
        mState[now] = DSM_LEFT_SHIFT;
    } else if (isRightShift) {
        time(&now);
        mState[now] = DSM_RIGHT_SHIFT;
    } else {
        return;
    }

    for (map<time_t,unsigned char>::iterator it = mState.lower_bound(0); it != mState.end() && it != mState.upper_bound(now - 3600);) {
        mState.erase(it++);
    }

    int checkCount = 10;
    if (mState.size() >= checkCount) {
        HHSQ_ADAS_MESSAGE_S message = {0};
        message.type = HHSQ_ADAS_TYPE_DSM;
        message.command = HHSQ_COMMAND_DSM_ALARM;
        message.content[0] = HHSQ_DSM_LEVEL_ONE;
        Notify::notify(message);
        for (int i = 0; i < checkCount; ++i) {
            mState.erase(mState.begin());
        }
    }
}