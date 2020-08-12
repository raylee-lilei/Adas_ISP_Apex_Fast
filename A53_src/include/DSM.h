//
// Created by wangxiaobao on 18-8-28.
//

#ifndef ADASPROJECT_DSM_H
#define ADASPROJECT_DSM_H

#include <map>
#include "define.h"
#include<time.h>

#define DSM_LEFT_SHIFT (unsigned char)1
#define DSM_RIGHT_SHIFT (unsigned char)2

class DSM {
private:
    map<time_t,unsigned char> mState;
public:
    DSM();
    ~DSM();

    void setShiftState(bool isLeftShift,bool isRightShift);
};


#endif //ADASPROJECT_DSM_H
