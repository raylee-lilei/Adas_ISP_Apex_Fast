//
// Created by wangxiaobao on 18-10-19.
//

#ifndef ADASPROJECT_MESSAGE_H
#define ADASPROJECT_MESSAGE_H

#include <pthread.h>
#include "sys/ipc.h"
#include "sys/msg.h"
#include "MessageType.h"
#include "ErrorCode.h"

class Message {

private:
    static Message* inst;
    static int      messageId;

    pthread_t threadId;

    static void getMessageId();

public:
    Message();
    static Message* instance();

    ErrorCode send(HHSQ_ADAS_MESSAGE_S &message);

    static void * receive(void* args);
};


#endif //ADASPROJECT_MESSAGE_H
