//
// Created by wangxiaobao on 18-10-19.
//

#include <string.h>

#ifndef ADASPROJECT_MESSAGETYPE_H
#define ADASPROJECT_MESSAGETYPE_H

#define HHSQ_ADAS_TYPE_LDW 1
#define HHSQ_ADAS_TYPE_DSM 2
#define HHSQ_ADAS_TYPE_FCW 3
#define HHSQ_ADAS_TYPE_TSR 4

#define HHSQ_COMMAND_LDWS_SWITCH 0
#define HHSQ_COMMAND_LDWS_RESULT 1
#define HHSQ_COMMAND_LDWS_ALARM  2
#define HHSQ_LDWS_OFF 0
#define HHSQ_LDWS_ON  1
#define HHSQ_LDWS_SHIFT_LEFT  0
#define HHSQ_LDWS_SHIFT_RIGHT 1

#define HHSQ_COMMAND_DSM_SWITCH 0
#define HHSQ_COMMAND_DSM_ALARM  1
#define HHSQ_DSM_LEVEL_ONE 0
#define HHSQ_DSM_LEVEL_TWO 1


#define HHSQ_COMMAND_FCW_SWITCH 0
#define HHSQ_COMMAND_FCW_ALARM  1
#define HHSQ_FCW_OFF 0
#define HHSQ_FCW_ON  1
#define HHSQ_FCW_LEVEL_ONE 0
#define HHSQ_FCW_LEVEL_TWO 1

#define HHSQ_COMMAND_TSR_SWITCH 0
#define HHSQ_COMMAND_TSR_LIMIT_BEGIN 1
#define HHSQ_COMMAND_TSR_LIMIT_END 2
#define HHSQ_COMMAND_TSR_LIMIT_OVER 3
#define HHSQ_TSR_OFF 0
#define HHSQ_TSR_ON  1
#define HHSQ_TSR_SPEED_OK 0
#define HHSQ_TSR_SPEED_OVER 1

typedef struct _HHSQ_ADAS_MESSAGE_S
{
    long type;//功能模块，对应上方的宏定义
    int command;//指令类别，见下方表格定义
    unsigned char content[8];//指令所附带的数据，按具体功能划分,最大8个字节,多余的用于扩展预留
}HHSQ_ADAS_MESSAGE_S;

inline bool operator < (const HHSQ_ADAS_MESSAGE_S &first, const HHSQ_ADAS_MESSAGE_S &second) {
    if (first.type < second.type) return true;
    if (first.command < second.command) return true;
    if (memcmp(first.content,second.content,sizeof(first.content)) < 0) return true;

    return false;
}
#endif //ADASPROJECT_MESSAGETYPE_H
