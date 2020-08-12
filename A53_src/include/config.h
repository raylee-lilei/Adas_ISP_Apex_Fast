//
// Created by wangxiaobao on 18-11-1.
//

#ifndef ADASPROJECT_CONFIG_H
#define ADASPROJECT_CONFIG_H

#include <opencv2/core/types.hpp>
#include "config.h"

class Config
{
private:
    static Config* inst;

public:
    bool isLdwCalibration;
    bool isNoDrawDetectResult;

    bool isLdwEnable;
    bool isFcwEnable;
    bool isTsrEnable;
    bool isDsmEnable;
    bool isSyncRun;
    bool isRound;

    cv::Point ldwsLeft;
    cv::Point ldwsTop;
    cv::Point ldwsRight;
    int ldwsTopWrapY;
    int jumpToSeconds;
    int jumpEndSeconds;
    int videoFps;
    std::string saveVideoPath;

    Config();
    static Config* instance();
};

#endif //ADASPROJECT_CONFIG_H
