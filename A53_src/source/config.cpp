//
// Created by wangxiaobao on 18-11-1.
//

#include "../include/config.h"

Config* Config::inst = 0;

Config::Config() {
    this->isDsmEnable = false;
    this->isFcwEnable = false;
    this->isLdwEnable = false;
    this->isTsrEnable = false;
    this->isRound = false;
    this->isLdwCalibration = false;
    this->ldwsTop.x = 0;
    this->ldwsTop.y = 0;
    this->saveVideoPath = "";
    this->isNoDrawDetectResult = false;
    this->isSyncRun = false;
    this->jumpToSeconds = 0;
    this->jumpEndSeconds = 0;
    this->videoFps = 30;
}

Config* Config::instance() {
    if (0 == inst)
    {
        inst = new Config();
    }

    return inst;
}
