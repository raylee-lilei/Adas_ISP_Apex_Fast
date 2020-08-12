//
// Created by wangxiaobao on 18-9-5.
//

#include "../include/define.h"
#include "../include/FCW.h"
#include "../include/path.h"
#include "../include/Util.h"
#include "../include/Adapter.h"
#include "../include/config.h"
#include "../include/MessageType.h"
#include "../include/Notify.h"

#include "../include/PeopleDetect.h"

pthread_mutex_t FCW::mutex = PTHREAD_MUTEX_INITIALIZER;

cv::CascadeClassifier FCW::detectorClassifier;
cv::Mat FCW::result;

#ifdef _ENV_DESKTOP
#define FDW_DETECT_FRAME_COUNT 2
#else
#define FDW_DETECT_FRAME_COUNT 2
#endif

#define OFFSET_SET \
		FCW::colOffset = 80; \
		FCW::rowOffset = 280; \
		FCW::roiWidth = 1088; \
		FCW::roiHeight = 400;

vector<cv::Rect> g_found[FDW_DETECT_FRAME_COUNT]; //用于防抖
unsigned long g_detectCount = 0;
int FCW::rowOffset = 0;
int FCW::colOffset = 0;
int FCW::roiWidth = 0;
int FCW::roiHeight = 0;

FCW::FCW(MediaCapture *mediaCapture,cv::Mat &mat) {
    this->oriImage = mat.clone();
    this->mediaCapture = mediaCapture;
#if 0
    //rowOffset = oriImage.rows /2;
    //colOffset = oriImage.cols /4;
    colOffset = 50;//Config::instance()->ldwsLeft.x - 50;//30;
    rowOffset = 250;//210;//Config::instance()->ldwsTop.y - 100;
    roiWidth = 1180;//Config::instance()->ldwsRight.x - Config::instance()->ldwsLeft.x + 100;//60;
    roiHeight = 460;//500;//Config::instance()->ldwsLeft.y - Config::instance()->ldwsTop.y + 50;//100;
    roiWidth = roiWidth;///4 * 4;
    roiHeight = roiHeight;///4 * 4;
#endif
    OFFSET_SET;

#if 0
    colOffset = 0;//Config::instance()->ldwsLeft.x - 50;//30;
    rowOffset = 0;//Config::instance()->ldwsTop.y - 100;
    roiWidth = 1280;//Config::instance()->ldwsRight.x - Config::instance()->ldwsLeft.x + 100;//60;
    roiHeight = 720;//Config::instance()->ldwsLeft.y - Config::instance()->ldwsTop.y + 50;//100;
#endif
}

/**
 * 取矩形中心点
 * @param rect
 * @param x
 * @param y
 */
void getCenter(cv::Rect& rect,int& x,int& y){
    x = rect.x + rect.width / 2;
    y = rect.y + rect.height /2;
}


/**
 * @brief 计算两个矩形的相交面积及组合面积，同时计算相交面积占组合面积的比例
 * @param 第一个矩形的位置
 * @param 第二个矩形的位置
 * @return 两个矩阵相交面积
 */
float computRectJoinUnion(const cv::Rect &rc1, const cv::Rect &rc2)
{
    CvPoint p1, p2;                 //p1为相交位置的左上角坐标，p2为相交位置的右下角坐标
    p1.x = std::max(rc1.x, rc2.x);
    p1.y = std::max(rc1.y, rc2.y);

    p2.x = std::min(rc1.x +rc1.width, rc2.x +rc2.width);
    p2.y = std::min(rc1.y +rc1.height, rc2.y +rc2.height);

    float AJoin = 0;
    if( p2.x > p1.x && p2.y > p1.y )            //判断是否相交
    {
        AJoin = (p2.x - p1.x)*(p2.y - p1.y);    //如果先交，求出相交面积
    }

    return AJoin;
}

void FCW::kickOverlap(vector<cv::Rect> &found){
    float joinMax = 0;
    int joinX = 0,joinY = 0;
    for (int i = 0; i < (int)found.size(); ++i) {
        for (int j = i + 1; j < (int)found.size(); ++j) {
            float join = computRectJoinUnion(found[i], found[j]);
            if (join > joinMax) {
                joinMax = join;
                joinX = i;
                joinY = j;
            }
        }
    }

    if (joinMax > 1000) {
        cout << "join:" << (float)joinMax << " i:" << joinX << " j:" << joinY << endl;
        if (found[joinX].width > found[joinY].width) {
            found.erase(found.begin()+joinX);
        } else {
            found.erase(found.begin()+joinY);
        }
    }
}

/**
 * 以first为基础，看second里有没有这个区域，主要用于防抖
 * @param first
 * @param second
 * @return
 */
vector<cv::Rect> FCW::merge(vector<cv::Rect> &first,vector<cv::Rect> &second){
    vector<cv::Rect> result;
    int buff = 20;
    for (int i = 0; i < (int)first.size(); ++i) {
        int x,y;
        getCenter(first[i],x,y);

        float xOffset = abs(this->oriImage.cols / 2 - x);
        float yOffset = abs(this->oriImage.rows - y);

        if (xOffset/yOffset > 1.2){
            cout << "continue---------------------\n";
            //continue;
        }

        if (first[i].y + first[i].height > oriImage.rows/2 && first[i].height > 32) {
            //cout << "continue2--------------------\n";
            //continue;
        }

        int j = 0;
        for (j = 0; j < (int)second.size(); ++j) {
            int x2,y2;
            getCenter(second[j],x2,y2);

            if (abs(x-x2) < buff && abs(y - y2) < buff && abs(first[i].width - second[j].width) < buff && abs(first[i].height - second[j].height) < buff) {
                break;
            }
        }

        if (j == (int)second.size()){
            continue;
        }

        if (abs(first[i].width - second[j].width) < 5 && abs(first[i].height - second[j].height) < 5) {
            first[i] = second[j];
        } else {
            first[i].width = (first[i].width + second[j].width)/2;
            first[i].height = (first[i].height + second[j].height)/2;
            first[i].x = (first[i].x + second[j].x) / 2;
            first[i].y = (first[i].y + second[j].y) / 2;

            if (first[i].x < (second[j].x-5)) first[i].x += 10;//趋势补偿
            if (first[i].x > (second[j].x+5)) first[i].x -= 10;
        }

        result.push_back(first[i]);
    }

    return result;
}

/**
 * 根据Y坐标计算距离
 * @param yPos
 * @return
 */
float FCW::getDistance(int yPos){
#if 0
    float distance = 3.5/(2*(Config::instance()->ldwsTop.x-Config::instance()->ldwsLeft.x)*1.0 / (Config::instance()->ldwsLeft.y - Config::instance()->ldwsTop.y) * 1.0);
    //float logRx = log(1*(yPos-Config::instance()->ldwsTop.y)/ (Config::instance()->ldwsLeft.y -Config::instance()->ldwsTop.y));
    float up  = (yPos-Config::instance()->ldwsTop.y) * 1.0;
    float down =(Config::instance()->ldwsLeft.y -Config::instance()->ldwsTop.y) * 1.0;

    cout << "disantce:" << (float)distance << endl;

    float logRx = log(up/ down);
    if (yPos-Config::instance()->ldwsTop.y < 0)
    {
        //return 0;
    }

    printf("pring:%.6f\n",logRx);
    //float logRx = log((Config::instance()->ldwsLeft.y -Config::instance()->ldwsTop.y)/(yPos-Config::instance()->ldwsTop.y));

    distance = distance * logRx * -1;

    cout << "up:" <<(int)(yPos-Config::instance()->ldwsTop.y) << "  down:" << (int)(Config::instance()->ldwsLeft.y -Config::instance()->ldwsTop.y) << endl;

    cout << "distance::"<< (float)distance << " pos:" << (int)yPos <<  " logrX:" << (float)logRx << endl;

    //return distance;

    //TODO: 先简单计算,需要按实际修改
    //return ((oriImage.rows - yPos) * 1.0 / 10);
#endif
    return Util::getDistance(yPos);
}

float FCW::getTTC(float dis) {
    //return dis;
    float mps = 60*1000/3600;
    return dis / mps;
}

cv::Mat FCW::getROI() {
#if 0
    colOffset = 50;//Config::instance()->ldwsLeft.x - 50;//30;
    rowOffset = 250;//210;//Config::instance()->ldwsTop.y - 100;
    roiWidth = 1140;//1180;//Config::instance()->ldwsRight.x - Config::instance()->ldwsLeft.x + 100;//60;
    roiHeight = 460;//500;//Config::instance()->ldwsLeft.y - Config::instance()->ldwsTop.y + 50;//100;
#endif
    OFFSET_SET;
#if 0
    colOffset = 0;//Config::instance()->ldwsLeft.x - 50;//30;
    rowOffset = 0;//Config::instance()->ldwsTop.y - 100;
    roiWidth = 1280;//Config::instance()->ldwsRight.x - Config::instance()->ldwsLeft.x + 100;//60;
    roiHeight = 720;//Config::instance()->ldwsLeft.y - Config::instance()->ldwsTop.y + 50;//100;
#endif
    //cv::Rect rect(colOffset,rowOffset,oriImage.cols - colOffset*2,oriImage.rows -  rowOffset - 100);
    cv::Rect rect(colOffset,rowOffset,roiWidth ,roiHeight);
    /*
    rowOffset=319;
    colOffset=223;
    cv::Rect rect(colOffset,rowOffset,800,720-rowOffset);
     */

    cv::Mat grayImage = oriImage(rect);


    return grayImage;
}


int FCW::checkAlarm(int x, int width, float ttc){

    if (ttc > 1) {
        return 0;
    }

    x += colOffset;

    int leftCheck = Config::instance()->ldwsTop.x - 20;
    int rightCheck = Config::instance()->ldwsTop.x + 20;

    if (x > rightCheck){
        return 0;
    }

    if (x + width < leftCheck) {
        return 0;
    }

    HHSQ_ADAS_MESSAGE_S message;
    message.type = HHSQ_ADAS_TYPE_FCW ;
    message.command = HHSQ_COMMAND_FCW_ALARM;
    if (ttc > 0.5){
        message.content[0] = HHSQ_FCW_LEVEL_ONE;
    } else {
        message.content[0] = HHSQ_FCW_LEVEL_TWO;
    }

    Notify::notify(message);
    return ttc > 0.5 ? 1 : 2;
}

/**
 * 主处理函数
 */
void FCW::process() {
    //UtilTimeDiff diff("FCW::process");

    if (detectorClassifier.empty()) {
        detectorClassifier.load(PATH_CLASSIFIER_CAR);
    }

    if (Config::instance()->ldwsTop.y == 0){
        return;
    }

    cv::Mat grayImage = getROI();

    //cv::cvtColor(grayImage,grayImage,cv::COLOR_RGB2GRAY);
    Adapter::instance()->Rgb2Gray(grayImage,grayImage);

    //cv::threshold(grayImage,grayImage,0,100,cv::THRESH_OTSU);

    //cv::imshow("ddsds",grayImage);
    //cv::waitKey(1);

    //cv::Rect rect(colOffset,rowOffset,oriImage.cols - colOffset*2,oriImage.rows -  rowOffset - 100);

    //grayImage = oriImage(rect);
    //cv::cvtColor(grayImage,grayImage,cv::COLOR_RGB2GRAY);

    //cv::equalizeHist(grayImage,grayImage);

    Adapter::instance()->equalizeHist(grayImage,grayImage);

    //cv::medianBlur(grayImage,grayImage,5);
    Adapter::instance()->gaussianFilter(grayImage,grayImage,5);

    vector<cv::Rect> found;

    //detectorClassifier.detectMultiScale(grayImage,found,1.05, 5, CV_HAAR_DO_CANNY_PRUNING|0|CV_HAAR_SCALE_IMAGE, cv::Size(60, 60), cv::Size(1200, 1200));

    //detectorClassifier.detectMultiScale(grayImage,found,1.06, 10, 0| 8 | CV_HAAR_DO_ROUGH_SEARCH     , cv::Size(20, 20), cv::Size(200, 200));

    Adapter::instance()->foundCar(grayImage,found);

    //vector<int> numDetections;
    //detectorClassifier.detectMultiScale(grayImage,found,numDetections,1.06, 10, CV_HAAR_DO_CANNY_PRUNING | 8 | CV_HAAR_DO_ROUGH_SEARCH     , cv::Size(20, 20), cv::Size(200, 200));

    if (found.size() > 5) {
        cout << "found size::" << found.size() << endl;
        found.erase(found.begin()+5,found.end());
    }

    g_found[g_detectCount % FDW_DETECT_FRAME_COUNT] = found;
    g_detectCount++;


    if (g_detectCount < FDW_DETECT_FRAME_COUNT) return;

    found = g_found[(g_detectCount - 1) % FDW_DETECT_FRAME_COUNT];
    for (int i = 2; i <= FDW_DETECT_FRAME_COUNT; ++i) {
        found = merge(found,g_found[(g_detectCount - i) % FDW_DETECT_FRAME_COUNT]);
    }

#ifndef _ENV_DESKTOP
    int holdTime = 2;
    if (found.size() == 0 && !result.empty()) {
        UtilMutexAutoLock lock(mutex);
        result.release();
        return;
    }
#else
    int holdTime = 5;
#endif
    //kickOverlap(found);
    static int holdCnt =holdTime ;
    if (found.size() == 0) {
        //cout << "car num:" << found.size() << " " << (int)holdCnt << endl;
        if (holdCnt-- == 0) {
            UtilMutexAutoLock lock(mutex);
            result.release();

        }
        return;
    }

    holdCnt = holdTime ;

    cv::Mat drawMat = cv::Mat(oriImage.rows,oriImage.cols,CV_8UC3,cv::Scalar(0,0,0));
    for (int i = 0; i < (int)found.size(); ++i) {

        int yPos = found[i].y+rowOffset+found[i].height;

        //cout << found[i].x << "  " << found[i].y << "  "  << found[i].width << "  "  << found[i].height << "  yPos:"  << yPos << endl;

        float dis = getDistance(yPos);
        if (dis >= INVALID_DISTANCE){// || dis >= 100){
            cout << "drop-------------" << (float)dis << endl;
            dis = 100;
            //continue;
        }
        char distance[100] = {0};
        float ttc = getTTC(dis);
        ttc = dis;
        if (dis >= 100){
            sprintf(distance,">=100m");
        } else {
            sprintf(distance,"%.1fm",ttc);
        }

        cv::Scalar color;

        if(ttc >= 10.0) {
            color = cv::Scalar(0x0, 0x66, 0x0);
        } else if (ttc > 5.0) {
            color = cv::Scalar(0xff, 0xff, 0);
        } else {
            color = cv::Scalar(0, 0xff, 0xff);
        }

        (void)this->checkAlarm(found[i].x, found[i].width,ttc);

        //cv::rectangle(drawMat,cv::Point(found[i].x + colOffset,found[i].y+rowOffset), cv::Point(found[i].x + colOffset+found[i].width,yPos), color,1,4);

        int drawLen = 15;
        int drawBold = 1;

#if 1
        //左上
        cv::line(drawMat,cv::Point(found[i].x + colOffset,found[i].y+rowOffset),cv::Point(found[i].x + colOffset,found[i].y+rowOffset+drawLen),color,drawBold );
        cv::line(drawMat,cv::Point(found[i].x + colOffset,found[i].y+rowOffset),cv::Point(found[i].x + colOffset+drawLen,found[i].y+rowOffset),color,drawBold );

        //右下
        cv::line(drawMat,cv::Point(found[i].x + colOffset+found[i].width,yPos),cv::Point(found[i].x + colOffset+found[i].width,yPos-drawLen),color,drawBold );
        cv::line(drawMat,cv::Point(found[i].x + colOffset+found[i].width,yPos),cv::Point(found[i].x + colOffset+found[i].width-drawLen,yPos),color,drawBold );

        cv::line(drawMat,cv::Point(found[i].x + colOffset+found[i].width,found[i].y+rowOffset),cv::Point(found[i].x + colOffset+found[i].width-drawLen,found[i].y+rowOffset),color,drawBold );
        cv::line(drawMat,cv::Point(found[i].x + colOffset+found[i].width,found[i].y+rowOffset),cv::Point(found[i].x + colOffset+found[i].width,found[i].y+rowOffset+drawLen),color,drawBold );


        cv::line(drawMat,cv::Point(found[i].x + colOffset,yPos),cv::Point(found[i].x + colOffset+drawLen,yPos),color,drawBold );
        cv::line(drawMat,cv::Point(found[i].x + colOffset,yPos),cv::Point(found[i].x + colOffset,yPos-drawLen),color,drawBold );
#endif
        int base;
        cv::Size size = cv::getTextSize(string(distance),cv::FONT_HERSHEY_COMPLEX_SMALL,1,2,&base);
        int textX = (found[i].width - size.width)/2;
        if (textX > 0) textX += colOffset + found[i].x;
        else textX = found[i].x + colOffset;

        cv::putText(drawMat,string(distance),cv::Point(textX/*found[i].x + colOffset+10*/,/*yPos*/found[i].y+rowOffset-2),
                    cv::FONT_HERSHEY_COMPLEX_SMALL, 1,color, 2);
#if 1
        int xCenter = found[i].x + colOffset + found[i].width / 2;
        int yCenter = found[i].y + rowOffset + found[i].height / 2;

        cv::circle(drawMat,cv::Point(xCenter,yCenter),found[i].width / 2 / 10 * 10,color,-1);
#endif
    }

    this->detectPeople(drawMat);

    do {
        UtilMutexAutoLock lock(mutex);
        result = drawMat;
    }while (0);
}

/**
 * FCW的图片叠加到原图
 * @param mat
 */
extern cv::Rect g_foundCarRect;
void FCW::drawResult(cv::Mat&mat){

    cv::Rect rect;
    rect.x = colOffset;
    rect.y = rowOffset;
    rect.width = roiWidth;
    rect.height = roiHeight;

    do {
        UtilMutexAutoLock lock(mutex);

        cv::rectangle(mat,rect,cv::Scalar(0,0,255),3);
#if 0
        if (g_foundCarRect.x != 0 || g_foundCarRect.y != 0){
			cv::Rect rect2;
			rect2.x = g_foundCarRect.x +  colOffset;
			rect2.y = g_foundCarRect.y + rowOffset;
			rect2.width = g_foundCarRect.width;
			rect2.height = g_foundCarRect.height;

			cv::rectangle(mat,rect2,cv::Scalar(0,255,255),3);
        }
#endif

        if (result.empty()) return;
        result = Util::RGB2YUV(result);
        cv::addWeighted(mat(rect), 1, result(rect), 1.0, 0, mat(rect));
    }while (0);
}

int FCW::detectPeople(cv::Mat& mat) {
    return 0;
    int roiY = oriImage.rows/2;
    cv::Rect roi = cv::Rect(0,roiY,oriImage.cols,oriImage.rows/2 - 1);
    cv::Mat peopleMat = oriImage(roi);
    std::vector<cv::Rect> regions ;
    PeopleDetect::instance()->detect(peopleMat,regions );

    // 显示
    for (size_t i = 0; i < regions.size(); i++) {
        regions[i].y += roiY;
        cv::rectangle(mat, regions[i], cv::Scalar(0, 0, 255), 2);
    }

}
