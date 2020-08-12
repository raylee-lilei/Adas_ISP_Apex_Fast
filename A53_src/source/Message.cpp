//
// Created by wangxiaobao on 18-10-19.
//

#include "../include/Message.h"
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "../include/define.h"
#include "../include/Util.h"

#define INVALID_MESSAGE_ID -1

Message* Message::inst = 0;
int      Message::messageId = INVALID_MESSAGE_ID;

Message::Message() {

    pthread_create(&threadId, NULL, receive, (void *) 0);
}

/**
 * 获取消息ID
 */
void Message::getMessageId() {

    if (messageId == INVALID_MESSAGE_ID ) {
        messageId=msgget((key_t)1234,0666 | IPC_CREAT);
    }

    return;
}

/**
 * 消息通信模块实例
 * @return
 */
Message* Message::instance(){

    getMessageId();

    if (0 == inst){
        inst = new Message();
    }

    return inst;
}

/**
 * 发送消息
 * @param message
 * @return
 */
ErrorCode Message::send(HHSQ_ADAS_MESSAGE_S &message){
    if (message.type <= 0){
        return ERROR_SUCCESS;
    }

    printf("receive message-------1\n");
    if (messageId == INVALID_MESSAGE_ID) {
        return ERROR_FAILURE;
    }

    int code = msgsnd(messageId, &message, sizeof(message), 0);
    if (-1 == code) {
        cout << "send message error!" << endl;
    }

#ifdef _ENV_DESKTOP
    int fd = open("/tmp/notify",O_WRONLY|O_NONBLOCK);
    if (fd < 0) {
        //cout << "fd Error" << endl;
    } else {
        write(fd,&message,sizeof(message));
        close(fd);
    }

#endif

//#define DebugCI
#ifdef DebugCI
#pragma message("CI版本编译")
    printf("receive message-------2\n");
    extern long g_timeSysStartSecond;
    long diffTime = Util::sysTimeMs() / 1000 - g_timeSysStartSecond;

    FILE* fw = fopen("./message.log","ab+");
    message.content[6] = diffTime / 256;
    message.content[7] = diffTime % 256;
    fwrite(&message,sizeof(message),1,fw);
    fclose(fw);
#endif
    return code;
}

/**
 * 接收消息
 * @param args
 * @return
 */
void * Message::receive(void* args){
    cout << "Message::receive run ------\n";
    while(messageId == INVALID_MESSAGE_ID){
        sleep(1);
    }

    HHSQ_ADAS_MESSAGE_S message;
    long int receiveId = 0;
    while(messageId != INVALID_MESSAGE_ID){
        if (msgrcv(messageId,(void*)&message,sizeof(message),receiveId,0) != 1) {
            cout << "receive Message:" ;
            cout << "type:" << message.type;
            cout << " command:" << message.command << "->";
            for (int i = 0; i < sizeof(message.content); ++i){
                cout << (int)message.content[i] << " ";
            }
            cout << endl;
        }
    }
}


