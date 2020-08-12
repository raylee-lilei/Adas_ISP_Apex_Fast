//
// Created by wangxiaobao on 19-1-24.
//

#include "../include/PeopleDetect.h"


PeopleDetect *PeopleDetect ::inst = NULL;

PeopleDetect *PeopleDetect::instance(){
    if (PeopleDetect::inst == NULL){
        PeopleDetect::inst = new PeopleDetect();
        PeopleDetect::inst->init();
    }

    return PeopleDetect::inst;
}

void PeopleDetect::init() {
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

    //hog.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());

    //hog= cv::HOGDescriptor(cv::Size(48, 96), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9, 1,-1, cv::HOGDescriptor::L2Hys, 0.2, true, cv::HOGDescriptor::DEFAULT_NLEVELS);


    // 1. 定义HOG对象
    //hog = cv::HOGDescriptor(cv::Size(48, 96), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9, 1,-1, cv::HOGDescriptor::L2Hys, 0.2, true, cv::HOGDescriptor::DEFAULT_NLEVELS);


    // 2. 设置SVM分类器
    //hog.setSVMDetector(cv::HOGDescriptor::getDaimlerPeopleDetector());   // 采用已经训练好的行人检测分类器

}

void PeopleDetect::detect(cv::Mat& mat,std::vector<cv::Rect>& regions){
	return;
    //hog.detectMultiScale(mat, regions, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 1);
    //hog.detectMultiScale(mat, regions, 0, cv::Size(8,8), cv::Size(0,0), 1.03, 2);
    //hog.detectMultiScale(mat, regions, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 1);
    hog.detectMultiScale(mat, regions);

    //对输入的图片img进行多尺度行人检测
    //img为输入待检测的图片；found为检测到目标区域列表；参数3为程序内部计算为行人目标的阈值，也就是检测到的特征到SVM分类超平面的距离;
    //参数4为滑动窗口每次移动的距离。它必须是块移动的整数倍；参数5为图像扩充的大小；参数6为比例系数，即测试图片每次尺寸缩放增加的比例；
    //参数7为组阈值，即校正系数，当一个目标被多个窗口检测出来时，该参数此时就起了调节作用，为0时表示不起调节作用。
}
