/*
 * HogSvmDetect.cpp
 *
 *  Created on: 2019年2月18日
 *      Author: wangxiaobao
 */

#include "../include/HogSvmDetect.h"

#include "../include/define.h"
#include "../include/path.h"
#include <fstream>

using namespace std;
using namespace cv;
using namespace cv::ml;

typedef struct SVM_INFO_S_{
    int dw;
    int dh;
    std::string path;
    float radio;
}SVM_INFO_S;

static SVM_INFO_S s_svmInfos[]={
        {32,32,"/opt/project/OwnCollection/hog_svm_x32.bin",0.125},
        {64,64,"/opt/project/OwnCollection/hog_svm_x64.bin",0.065},
        {48,48,"/opt/project/OwnCollection/hog_svm_x48.bin",0.038}
};

static int s_svmIndex = 1;

HogSvmDetect::HogSvmDetect() {
	// TODO Auto-generated constructor stub
    mIsInited = false;
    init();
}

HogSvmDetect::~HogSvmDetect() {
	// TODO Auto-generated destructor stub
}

void HogSvmDetect::init() {

    int dw  = s_svmInfos[s_svmIndex].dw;//64;
    int dh = s_svmInfos[s_svmIndex].dh;//64;
    mHog.winSize = cv::Size(dw,dh);
    FILE *fp = fopen(s_svmInfos[s_svmIndex].path.c_str(),"rb");
    vector<float> svmData;
    float value;
    while(fscanf(fp,"%f\n",&value) != EOF){
        svmData.push_back(value);
    }
    mHog.setSVMDetector(svmData);

    mIsInited = true;
}

void HogSvmDetect::foundCar(cv::Mat&frame,std::vector<cv::Rect> &found){
    if (!mIsInited){
        found.clear();
        return;
    }

    vector<double> foundWeights;

    cv::Mat img;
    if (frame.type() == CV_8UC3) {
        cv::cvtColor(frame, img, COLOR_BGR2GRAY);
    } else {
        img = frame;
    }

    //cv::GaussianBlur(img,img,cv::Size(3,3),0);

    static int count = 0;

    mHog.detectMultiScale(img, found, foundWeights,s_svmInfos[s_svmIndex].radio);

    if (found.size() == 0) return;

    cout << endl << "found weights:" << (int)found.size() << endl;
#if 0
    for (int i = 0; i < found.size(); ++i){
        printf("%.03f  ",foundWeights[i]);
        char path[255] = {0};
        sprintf(path,"/opt/project/OwnCollection/save2/cute_%d.jpg",count++);
        cv::imwrite(path,img(found[i]));
    }
#endif

    cout  << endl;
}
