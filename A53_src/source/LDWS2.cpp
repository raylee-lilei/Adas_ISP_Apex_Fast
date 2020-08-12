//
// Created by wangxiaobao on 18-12-25.
//

#include "../include/LDWS.h"
#include "../include/Points.h"
#include "../include/Notify.h"
#include "../include/Util.h"
#include "../include/Adapter.h"
#include "../include/anyarg.h"
#include "../include/config.h"
#include "../include/MediaCapture.h"
using namespace cv;

#define LDWS_DEFAULT_HOLD_TIMES 40

#if 0
static float xldata= 290.0;//左边x的横坐标
static float xrdata= 900.0;//右边x的横坐标
static float xtdata= 628.0;//三角形顶点的横坐标
static float ytdata= 522.0;//三角形顶点的纵坐标
static float yblabel= 720.0;//buttomy的值
static int ytlabel= 565.0;//topy的值 绿线30m
#endif

LDWS::LDWS(MediaCapture *mediaCapture) {
    mutex = PTHREAD_MUTEX_INITIALIZER;
    this->capture = mediaCapture;

    this->isLeftShift = false;
    this->isRightShift = false;

    mHoldTimes = 0;

    memset(&mMessageNotify,0,sizeof(mMessageNotify));
    mMessageNotify.type = HHSQ_ADAS_TYPE_LDW;
    mLastNotifyTime = 0;
    mLastNotifyContent = -1;
}

/**
 * 直线拟合
 * @param width
 * @param height
 * @param points
 * @return
 */
PointLine LDWS::fitLaneLine(int width,int height,std::vector<cv::Point> points){
    PointLine pointLine;
    if (points.size() == 0){
        return pointLine;
    }
    Vec4f line;
    fitLine(points, line, CV_DIST_L2, 0, 0.01, 0.01);

    double k = line[1] / line[0];

    //计算直线的端点(y = k(x - x0) + y0)
    cv::Point point1, point2;

    point2.y = 0;
    point2.x = (0-line[3])/k+line[2];

    point1.y = height;
    point1.x = (height-line[3])/k+line[2];

    pointLine.top = point2;
    pointLine.bottom = point1;

    return pointLine;
}

/**
 * 通过Y查左X
 */
int getX2(int y2) {
    //float x2 = (xtdata - xldata) * 1.0 / (yblabel - ytdata) * (yblabel - y2) + xldata;
    float x2 = (Config::instance()->ldwsTop.x - Config::instance()->ldwsLeft.x) * 1.0
               / (Config::instance()->ldwsLeft.y - Config::instance()->ldwsTop.y) * (Config::instance()->ldwsLeft.y - y2)
               + Config::instance()->ldwsLeft.x;
    return x2;
}

/**
 * 通过Y查右X
 * @param y2
 * @return
 */
int getX3(int y2) {
    //float x2 = xrdata - (yblabel - y2) * 1.0 * (xrdata - xtdata) / (yblabel - ytdata);
    float x2 = Config::instance()->ldwsRight.x - (Config::instance()->ldwsRight.y - y2) * 1.0
               * (Config::instance()->ldwsRight.x - Config::instance()->ldwsTop.x) / (Config::instance()->ldwsRight.y - Config::instance()->ldwsTop.y);
    return x2;
}

/**
 * 找区域直方图最大连接区域,V1：认为白点最多的是车道线的所在区间
 * @param mat
 * @param beginIn
 * @param endIn
 * @param beginOut
 * @param endOut
 * @param maxIndexOut
 */
void LDWS::getHistSection(cv::Mat&mat, int beginIn,int endIn,int& beginOut,int& endOut,int& maxIndexOut){
    beginOut = 0;
    endOut = 0;
    maxIndexOut = 0;
    int maxValue = -1;
    int maxIndex = -1;
    int recordBeginIndex = -1;
    list<vector<int> > recordList;
    for (int i = beginIn; i < endIn; ++i){
        if (i < 0 || i >= mat.cols){
            cout << "eee\n";
            continue;
        }
        int colValue = 0;
        for (int j = 0; j < mat.rows; ++j){
            unsigned char value = mat.at<unsigned char>(j,i);
            if (value > 0) colValue++;
        }

        // 降噪
        colValue=colValue > 10 ? colValue  : 0;

        if (colValue > maxValue ){
            maxValue = colValue;
            maxIndex = i;
        }

        if (colValue > 0 && recordBeginIndex == -1){
            //找前
            recordBeginIndex = i;
        } else if (colValue == 0 && recordBeginIndex != -1){
            //找后
            vector<int> pos;
            pos.push_back(recordBeginIndex);
            pos.push_back(i);
            recordList.push_back(pos);

            recordBeginIndex = -1;
        }
    }

    //补列
    if (recordBeginIndex != -1){
        vector<int> pos;
        pos.push_back(recordBeginIndex);
        pos.push_back(endIn);
        recordList.push_back(pos);
    }

    //找区间
    for (list<vector<int> >::iterator it = recordList.begin(); it != recordList.end(); ++it){
        vector<int> value = *it;
        if (maxIndex >= value[0] && maxIndex <= value[1]){
            maxIndexOut = maxValue;
            beginOut = value[0];
            endOut = value[1];
            break;
        }
    }
}

/**
 * 查找车道线所在最可能的区域
 * @param mat
 * @return
 */
cv::Mat LDWS::getPossibleLaneDomain(cv::Mat& mat) {
    cv::Mat posIndex = cv::Mat::zeros(1,6,CV_32SC1);
    int beginLeft,endLeft,maxIndexLeft;
    int beginRight,endRight,maxIndexRight;

    getHistSection(mat,0,mat.cols/2 - 10,beginLeft,endLeft,maxIndexLeft);
    getHistSection(mat,mat.cols/2 + 10,mat.cols,beginRight,endRight,maxIndexRight);

    posIndex.at<int>(0,0) = beginLeft;
    posIndex.at<int>(0,1) = endLeft;
    posIndex.at<int>(0,4) = maxIndexLeft;

    posIndex.at<int>(0,2) = beginRight;
    posIndex.at<int>(0,3) = endRight;
    posIndex.at<int>(0,5) = maxIndexRight;

    return posIndex;
}

/**
 * 预处理，主要是赋值固定参数
 */
void LDWS::preProcess(){
    //变换矩阵
    if (mWrapMat.empty()) {
        //int height = yblabel - ytlabel;
        int height = Config::instance()->ldwsRight.y - Config::instance()->ldwsTopWrapY;

        vector<cv::Point2f> mOrigPoints;
        vector<cv::Point2f> mDestPoints;

        int btmOffset = 50;

        int xT = (getX3(Config::instance()->ldwsTopWrapY) - getX2(Config::instance()->ldwsTopWrapY)) / 2;
        int xB = (Config::instance()->ldwsRight.x - Config::instance()->ldwsLeft.x)/2;

        int topOffset = btmOffset/(xB / xT);
        int wrapOffset = 120;

        mROI.x = min(getX2(Config::instance()->ldwsTopWrapY) - topOffset,Config::instance()->ldwsLeft.x - btmOffset);
        mROI.y = min(Config::instance()->ldwsTopWrapY,Config::instance()->ldwsLeft.y);
        mROI.width = max(Config::instance()->ldwsRight.x + btmOffset - (Config::instance()->ldwsLeft.x - btmOffset),getX3(Config::instance()->ldwsTopWrapY) + topOffset -(getX2(Config::instance()->ldwsTopWrapY) - topOffset));
        mROI.height = Config::instance()->ldwsRight.y - Config::instance()->ldwsTopWrapY;

        mOrigPoints.push_back(cv::Point2f(getX2(Config::instance()->ldwsTopWrapY) - topOffset, Config::instance()->ldwsTopWrapY));
        mOrigPoints.push_back(cv::Point2f(getX3(Config::instance()->ldwsTopWrapY) + topOffset, Config::instance()->ldwsTopWrapY));
        mOrigPoints.push_back(cv::Point2f(Config::instance()->ldwsRight.x + btmOffset, Config::instance()->ldwsRight.y));
        mOrigPoints.push_back(cv::Point2f(Config::instance()->ldwsLeft.x - btmOffset, Config::instance()->ldwsRight.y));

        mDestPoints.push_back(cv::Point2f(0  + wrapOffset, 0));
        mDestPoints.push_back(cv::Point2f(Config::instance()->ldwsRight.x - Config::instance()->ldwsLeft.x - wrapOffset, 0));
        mDestPoints.push_back(cv::Point2f(Config::instance()->ldwsRight.x - Config::instance()->ldwsLeft.x - wrapOffset, height));
        mDestPoints.push_back(cv::Point2f(0 + wrapOffset, height));

        mWrapMat = cv::getPerspectiveTransform(mOrigPoints,mDestPoints);
        mWrapMatRsv  = cv::getPerspectiveTransform(mDestPoints,mOrigPoints);
    }

    //变换后的标准坐标点
    if (mStandardPosition.empty()){
        cv::Mat stand = cv::Mat::zeros(mOriImage.rows,mOriImage.cols,CV_8UC3);
        cv::line(stand , cv::Point2f(Config::instance()->ldwsLeft.x, Config::instance()->ldwsRight.y), cv::Point2f(Config::instance()->ldwsRight.x, Config::instance()->ldwsRight.y), cv::Scalar(255,255,0));
        cv::line(stand , cv::Point2f(Config::instance()->ldwsRight.x, Config::instance()->ldwsRight.y), cv::Point2f(Config::instance()->ldwsTop.x, Config::instance()->ldwsTop.y), cv::Scalar(255,255,0) );
        cv::line(stand , cv::Point2f(Config::instance()->ldwsTop.x, Config::instance()->ldwsTop.y), cv::Point2f(Config::instance()->ldwsLeft.x, Config::instance()->ldwsRight.y), cv::Scalar(255,255,0) );
        cv::warpPerspective(stand , stand, mWrapMat , cv::Size(Config::instance()->ldwsRight.x-Config::instance()->ldwsLeft.x, Config::instance()->ldwsRight.y - Config::instance()->ldwsTopWrapY));
        cv::Mat mat;
        Adapter::instance()->Rgb2Gray(stand,mat);
        cv::Point leftTop(0,0);
        cv::Point leftBtm(0,mat.rows-1);
        cv::Point rightTop(0,0);
        cv::Point rightBtm(0,mat.rows-1);

        for (int i = 0; i < mat.cols/2 ; ++i){
            int topValue = mat.at<unsigned char>(0,i);
            int btmValue = mat.at<unsigned char>(mat.rows-1,i);

            if (leftTop.x == 0 && topValue != 0){
                leftTop.x = i;
            } else if (leftTop.x !=0 && topValue != 0) {
                leftTop.x = (leftTop.x + i )/2;
            }

            if (leftBtm.x == 0 && btmValue != 0){
                leftBtm.x = i;
            } else if (leftBtm.x !=0 && btmValue != 0) {
                leftBtm.x = (leftBtm.x + i )/2;
            }
        }

        for (int i = mat.cols/2; i < mat.cols ; ++i){
            int topValue = mat.at<unsigned char>(0,i);
            int btmValue = mat.at<unsigned char>(mat.rows-1,i);

            if (rightTop.x == 0 && topValue != 0){
                rightTop.x = i;
            } else if (rightTop.x !=0 && topValue != 0) {
                rightTop.x = (rightTop.x + i )/2;
            }

            if (rightBtm.x == 0 && btmValue != 0){
                rightBtm.x = i;
            } else if (rightBtm.x !=0 && btmValue != 0) {
                rightBtm.x = (rightBtm.x + i )/2;
            }
        }

        mStandardPosition.LB = leftBtm;
        mStandardPosition.LT = leftTop;
        mStandardPosition.RT = rightTop;
        mStandardPosition.RB = rightBtm;

        mStandardROI.x  =(leftBtm.x < leftTop.x) ? leftBtm.x : leftTop.x;
        mStandardROI.y = leftTop.y;
        mStandardROI.width = ((rightBtm.x - mStandardROI.x) > (rightTop.x - mStandardROI.x)) ? (rightBtm.x - mStandardROI.x) :(rightTop.x - mStandardROI.x);
        mStandardROI.height = leftBtm.y - leftTop.y;
        mStandardROI.x -= 20;
        mStandardROI.width += 40;
    }
}

/**
 * 本次车道线检测结果
 * @param width
 * @param rect
 */
void LDWS::resultLane(int width,PointRect& rect){
    if (rect.empty()) {
        rect = mLastDetectLane;
        return;
    }

    if (mLastDetectLane.empty()){
        mLastDetectLane = rect;
        return;
    }

    //如果变化比较小，则保持一段时间，以免看 上去在不停的动
    int validPx = width / 15;
    static int holdTimes = 0;
    if (   abs(rect.RT.x - mLastDetectLane.RT.x) < validPx
        && abs(rect.RT.y - mLastDetectLane.RT.y) < validPx

        && abs(rect.RB.y - mLastDetectLane.RB.y) < validPx
        && abs(rect.RB.x - mLastDetectLane.RB.x) < validPx

        && abs(rect.LB.y - mLastDetectLane.LB.y) < validPx
        && abs(rect.LB.x - mLastDetectLane.LB.x) < validPx

        && abs(rect.LT.y - mLastDetectLane.LT.y) < validPx
        && abs(rect.LT.x - mLastDetectLane.LT.x) < validPx
            ){

        if (++holdTimes < 10){//(Config::instance()->videoFps*3)) { //3秒回归
            rect = mLastDetectLane;
            return;
        }
    }

    holdTimes = 0;

    mLastDetectLane = rect;
}

/**
 * 绘图
 * @param rect
 */
void LDWS::resultDraw(PointRect& rect){
    static PointRect lastDraw;
    if (lastDraw.empty()){
        lastDraw = rect;
        return;
    }

    if (lastDraw.equal(rect)){
        return;
    }

    //static int shiftPerTime = 2;
    static int shiftPerTime = 20;

    for (int i = 0; i < lastDraw.vPoints.size(); ++i){
        if (lastDraw.vPoints[i].x < rect.vPoints[i].x){
            lastDraw.vPoints[i].x += shiftPerTime;
            if (lastDraw.vPoints[i].x > rect.vPoints[i].x) lastDraw.vPoints[i].x = rect.vPoints[i].x;
        }

        if (lastDraw.vPoints[i].y < rect.vPoints[i].y){
            lastDraw.vPoints[i].y += shiftPerTime;
            if (lastDraw.vPoints[i].y > rect.vPoints[i].y) lastDraw.vPoints[i].y = rect.vPoints[i].y;
        }

        if (lastDraw.vPoints[i].x > rect.vPoints[i].x){
            lastDraw.vPoints[i].x -= shiftPerTime;
            if (lastDraw.vPoints[i].x < rect.vPoints[i].x) lastDraw.vPoints[i].x = rect.vPoints[i].x;
        }

        if (lastDraw.vPoints[i].y > rect.vPoints[i].y){
            lastDraw.vPoints[i].y -= shiftPerTime;
            if (lastDraw.vPoints[i].y < rect.vPoints[i].y) lastDraw.vPoints[i].y = rect.vPoints[i].y;
        }
    }

    rect = lastDraw;
}

/**
 * 调整检测到车道线
 * @param width
 * @param height
 * @param rect
 */
void LDWS::adjustLane(int width,int height,PointRect& rect){
    if (rect.empty()){
        return;
    }

    static int middle = width / 2;
    static int normal = (mStandardPosition.RB.x - mStandardPosition.LB.x) / 3;//(Config::instance()->ldwsRight.x - Config::instance()->ldwsLeft.x)/3;//width / 5; //正常偏移
    static int standardDiffLeftTop = 0;
    static int standardDiffLeftBtm = 0;
    static int standardDiffRightTop = 0;
    static int standardDiffRightBtm = 0;

    //计算标准差
    if (standardDiffLeftTop == 0){
        standardDiffLeftTop = middle - mStandardPosition.LT.x;
        standardDiffLeftBtm = middle - mStandardPosition.LB.x;
        standardDiffRightTop  = mStandardPosition.RT.x - middle;
        standardDiffRightBtm  = mStandardPosition.RB.x - middle;
    }

    //计算中间离两边的距离
    int leftTop = middle - rect.LT.x;
    int leftBtm = middle - rect.LB.x;
    int rightTop = rect.RT.x-middle;
    int rightBtm = rect.RB.x-middle;

    leftTop = abs(leftTop - standardDiffLeftTop);
    leftBtm = abs(leftBtm - standardDiffLeftBtm);
    rightTop = abs(rightTop - standardDiffRightTop);
    rightBtm = abs(rightBtm - standardDiffRightBtm);

    //没检测到
    if (rect.empty()){
        return;
    }

    //有交叉
    if (rect.LT.x > rect.RT.x || rect.LB.x > rect.RB.x){
        if (rect.LB.x > width/2) rect.zeroLeft();
        else rect.zeroRight();
    }

    static PointRect lastCheckRect;
    if (lastCheckRect.empty()){
        lastCheckRect = rect;
    }

    float leftTan = fabs((rect.LT.y - rect.LB.y) * 1.0 / (rect.LT.x - rect.LB.x + 0.1));
    float rightTan = fabs((rect.RT.y - rect.RB.y) * 1.0 / (rect.RT.x - rect.RB.x + 0.1));

    static long count = 0;
    //过滤跳变
    if (abs(lastCheckRect.LT.x - rect.LT.x) > normal || abs(lastCheckRect.LB.x - rect.LB.x) > normal){
        leftTan = 0;
        //printf("left--%ld\n",++count);
    }

    if (abs(lastCheckRect.RT.x - rect.RT.x) > normal || abs(lastCheckRect.RB.x - rect.RB.x) > normal){
        rightTan = 0;
        //printf("right--%ld\n",++count);
    }

    //最大的倾斜角
    float validArg = 0.2;

    //斜率太大或偏差太大
    if (leftTan < 1 || rightTan < 1){
        //以左为准
        if (leftTan > rightTan && leftTan >= validArg ) {
            rect.zeroRight();
            rightTop = normal + 1;
        } else if (rightTan > leftTan && rightTan >= validArg ){
            rect.zeroLeft();
            leftTop = normal + 1;
        } else {
            lastCheckRect.zero();
            rect.zero();
            //printf("zero--%ld,hold:%d\n",++count,lastCheckHold);
            return;
        }
    }

    if (leftTop < normal && leftBtm < normal && rightTop < normal && rightBtm < normal) {
        //都有效
    } else if ((leftTop < normal && leftBtm < normal) && (rightTop > normal || rightBtm > normal)){
        //左有效
        PointLine line = getRightLane(PointLine(rect.LT,rect.LB));
        rect.RT = line.top;
        rect.RB = line.bottom;
    } else if ((rightBtm < normal && rightTop < normal) && (leftBtm > normal || leftTop > normal)){
        //右有效
        PointLine line = getLeftLane(PointLine(rect.RT,rect.RB));
        rect.LT = line.top;
        rect.LB = line.bottom;
    } else {
        //都无效
        if (rect.LB.x == 0 && rect.LT.x == 0){
            //左为空，,以右为准
            PointLine line = getLeftLane(PointLine(rect.RT,rect.RB));
            rect.LT = line.top;
            rect.LB = line.bottom;
        }else{
            PointLine line = getRightLane(PointLine(rect.LT,rect.LB));
            rect.RT = line.top;
            rect.RB = line.bottom;
        }
    }

    lastCheckRect = rect;
}

PointLine LDWS::getLeftLane(PointLine line){

    PointLine result;
    result.top.x = mStandardPosition.LT.x + line.top.x - mStandardPosition.RT.x;
    result.top.y = mStandardPosition.LT.y + line.top.y - mStandardPosition.RT.y;

    result.bottom.x = mStandardPosition.LB.x + line.bottom.x - mStandardPosition.RB.x;
    result.bottom.y = mStandardPosition.LB.y + line.bottom.y - mStandardPosition.RB.y;

    return result;
}

PointLine LDWS::getRightLane(PointLine line){

    PointLine result;
    result.top.x = mStandardPosition.RT.x + line.top.x - mStandardPosition.LT.x;
    result.top.y = mStandardPosition.RT.y + line.top.y - mStandardPosition.LT.y;

    result.bottom.x = mStandardPosition.RB.x + line.bottom.x - mStandardPosition.LB.x;
    result.bottom.y = mStandardPosition.RB.y + line.bottom.y - mStandardPosition.LB.y;

    return result;
}

/**
 * 车道线颜色填充
 * @param perspectiveFill
 * @param detectLane
 */
void LDWS::resultFill(cv::Mat& perspectiveFill,PointRect& detectLane){

    matFillPolly(perspectiveFill,detectLane.LT,detectLane.RT,detectLane.RB,detectLane.LB,cv::Scalar(0xff,0,0));
    cv::line(perspectiveFill,detectLane.LT,detectLane.LB,cv::Scalar(128,128,128),20);
    cv::line(perspectiveFill,detectLane.RB,detectLane.RT,cv::Scalar(128,128,128),20);
}

void LDWS::colorEnhance(cv::Mat& mat){
    if (mat.empty() || mat.channels() != 3){
        cout << "channels:" << (int)mat.channels() << endl;
        return;
    }

    int bLuminance = 0;
    int gLuminance = 10;
    int rLuminance = 10;
    for ( int i = 0; i < mat.rows; i++)
    {
        for (int j = 0; j < mat.cols; j++)
        {
            int b = mat.at<Vec3b>(i, j)[0] + bLuminance;
            int g = mat.at<Vec3b>(i, j)[1] + gLuminance;
            int r = mat.at<Vec3b>(i, j)[2] + rLuminance;

            mat.at<Vec3b>(i, j)[0] = saturate_cast<uchar>(b > 255 ? 255 : b);
            mat.at<Vec3b>(i, j)[1] = saturate_cast<uchar>(g > 255 ? 255 : b);
            mat.at<Vec3b>(i, j)[2] = saturate_cast<uchar>(r > 255 ? 255 : b);
        }
    }
}

/**
 * 检测车道偏移告警
 * @param rect
 */
void LDWS::detectAlarm(PointRect& rect){

    static int distance = abs(rect.LB.x - mStandardPosition.LB.x);

    //求平均
    //distance = (distance + abs(rect.LB.x - mStandardPosition.LB.x)) / 2;
    distance = ( abs(rect.LB.x - mStandardPosition.LB.x)) ;

    if (distance < (mStandardPosition.RB.x - mStandardPosition.LB.x)/3) {
        return;
    }
    long curTime = Util::sysTimeMs();

    //TODO: 当前只处理了左右偏告警
    mMessageNotify.command = HHSQ_COMMAND_LDWS_ALARM;
    int curContent;
    int curCommand =HHSQ_COMMAND_LDWS_ALARM;
    if (rect.LB.x < mStandardPosition.LB.x){
        //right
        curContent = HHSQ_LDWS_SHIFT_RIGHT;
    } else {
        //left
        curContent = HHSQ_LDWS_SHIFT_LEFT;
    }

    static map<long,int> notifyData;

    int curNotifyContent = curCommand * 100 + curContent;

    if (mLastNotifyContent ==  -1){
        mLastNotifyContent = curNotifyContent;
    }

    int lastCommand = mLastNotifyContent / 100;
    static int laneChangeTime = 5000;

    //TODO: 当前只处理了左右偏告警
    notifyData[curTime] = curNotifyContent;
    notifyData.erase(notifyData.begin(),notifyData.lower_bound(curTime-laneChangeTime));

    if (notifyData.size() < 1 && (curCommand == lastCommand )) {
        return;
    }

    cout << "50000000000000------------" <<endl;
    if (curCommand == HHSQ_COMMAND_LDWS_ALARM){
        //变更时间内告警压制
        if (curTime - mLastNotifyTime < laneChangeTime){
            mLastNotifyContent = curNotifyContent;
            cout << "60000000000000------------" <<endl;
            return;
        }
    }

    mLastNotifyContent = curNotifyContent;
    mLastNotifyTime = curTime;

    mMessageNotify.command = curCommand;
    mMessageNotify.content[0] = mMessageNotify.content[0] = (notifyData.begin()->second) % 100;

    cout << "777777700000000------------" <<endl;
    Notify::notify(mMessageNotify);
    notifyData.clear();
}

/**
 * 车道线检测
 */
extern int g_cannyH;
extern int g_cannyL;
void LDWS::detectLane() {

    cv::Mat perspective;
    cv::Mat perspectiveTmp;
    cv::warpPerspective(mOriImage , perspective, mWrapMat, cv::Size(Config::instance()->ldwsRight.x-Config::instance()->ldwsLeft.x, Config::instance()->ldwsLeft.y - Config::instance()->ldwsTopWrapY));

    std::vector<cv::Point> leftPoints;
    std::vector<cv::Point> rightPoints;

    //colorEnhance(perspective);
    cv::Mat perspectiveGray;
    Adapter::instance()->Rgb2Gray(perspective,perspectiveGray);

    //this->capture->syncShow("perspectiveGray0",perspectiveGray);

    //GaussianBlur(perspectiveGray, perspectiveGray, Size(7, 7), 1.5);
    //cv::medianBlur(perspectiveGray,perspectiveGray,5);
    //cv::threshold(perspectiveGray,perspectiveGray,128,255,cv::THRESH_BINARY);

    cv::Canny(perspectiveGray,perspectiveGray,g_cannyL,g_cannyH);

#if 0
    cv::Mat ss;
    cv::cvtColor(perspectiveGray,ss,cv::COLOR_GRAY2RGB);
    //this->capture->syncShow("ss",ss);
    Adapter::instance()->showImage("perspectiveGray",ss,10);
#endif
    //cv::Canny(perspectiveGray,perspectiveGray,40,80);
#if 0
    //前后两图合并以加强车道线长度
    static cv::Mat prePerspective;
    if (prePerspective.empty()){
        prePerspective = perspectiveGray;
    } else {
        perspectiveTmp = perspectiveGray;
        cv::addWeighted(prePerspective,0.5,perspectiveGray,0.5,0,perspectiveGray);
        prePerspective = perspectiveTmp;
    }
    //this->capture->syncShow("perspectiveGray",perspectiveGray);
#endif
    cv::Mat posIndex = getPossibleLaneDomain(perspectiveGray);

    cv::line(perspective,cv::Point(posIndex.at<int>(0,0),0),cv::Point(posIndex.at<int>(0,0),perspective.rows-1),cv::Scalar(134,54,165),3);
    cv::line(perspective,cv::Point(posIndex.at<int>(0,1),0),cv::Point(posIndex.at<int>(0,1),perspective.rows-1),cv::Scalar(134,54,165),3);

    cv::line(perspective,cv::Point(posIndex.at<int>(0,2),0),cv::Point(posIndex.at<int>(0,2),perspective.rows-1),cv::Scalar(134,154,165),3);
    cv::line(perspective,cv::Point(posIndex.at<int>(0,3),0),cv::Point(posIndex.at<int>(0,3),perspective.rows-1),cv::Scalar(134,154,165),3);

    vector<cv::Vec4i>lines;
    cv::HoughLinesP(perspectiveGray(mStandardROI),lines,4,CV_PI/180,10, 20, 5);
    for (int i = 0; i < lines.size(); ++i) {
        PointLine line(lines[i][0] + mStandardROI.x,lines[i][1], lines[i][2] + mStandardROI.x, lines[i][3]);

        //剔除太平的线
        if (fabs(line.second.y - line.first.y) / fabs(line.second.x - line.first.x) < Util::getTanByAngle(60)){
            continue;
        }

        //按所在点位置判断是左还是右
        if (line.first.x <= posIndex.at<int>(0,1) && line.first.x >= posIndex.at<int>(0,0)) {
            leftPoints.push_back(cv::Point(line.first.x,line.first.y));
            leftPoints.push_back(cv::Point(line.second.x,line.second.y));
            //cv::line(perspective,cv::Point(line.first.x,line.first.y),cv::Point(line.second.x,line.second.y),cv::Scalar(0,255,255));
        } else if (line.first.x <= posIndex.at<int>(0,3) && line.first.x>= posIndex.at<int>(0,2)) {
            rightPoints.push_back(cv::Point(line.first.x,line.first.y));
            rightPoints.push_back(cv::Point(line.second.x,line.second.y));
            //cv::line(perspective,cv::Point(line.first.x,line.first.y),cv::Point(line.second.x,line.second.y),cv::Scalar(0,255,255));
        }
    }

    PointLine fitLeft,fitRight;
    fitLeft = fitLaneLine(perspective.cols,perspective.rows,leftPoints);
    fitRight = fitLaneLine(perspective.cols,perspective.rows,rightPoints);

    cv::Mat perspectiveFill = cv::Mat::zeros(perspective.rows,perspective.cols,CV_8UC3);

    PointRect detectLane;
    if (!fitLeft.empty()){
        detectLane.LT = fitLeft.top;
        detectLane.LB = fitLeft.bottom;
    }

    if (!fitRight.empty()){
        detectLane.RT = fitRight.top;
        detectLane.RB = fitRight.bottom;
    }

    //调速左右车道线
    adjustLane(perspectiveFill.cols,perspectiveFill.rows,detectLane);
    if (detectLane.empty()){
        return;
    }

    //最终的显示结果
    resultLane(perspectiveFill.cols,detectLane);

    detectAlarm(detectLane);

    //取得动画画图
    resultDraw(detectLane);
    cv::line(perspective,detectLane.LB,detectLane.LT,cv::Scalar(0,0xff,0));
    cv::line(perspective,detectLane.RB,detectLane.RT,cv::Scalar(0,0xff,0));

    //车道线填充颜色
    resultFill(perspectiveFill,detectLane);

    cv::Mat perspectiveShow = cv::Mat::zeros(mOriImage.rows,mOriImage.cols,CV_8UC3);
    cv::warpPerspective(perspectiveFill , perspectiveShow , mWrapMatRsv, cv::Size(mOriImage.cols, mOriImage.rows));

    //分段填充边界线颜色
    fillFrame(perspectiveShow,detectLane);

    //this->capture->syncShow("perspectiveShow",perspectiveShow);
    //Adapter::instance()->showImage("perspectiveShow",perspectiveShow,10);

    do{
        UtilMutexAutoLock lock(mutex);
        mLastShowMat = perspectiveShow;
        mHoldTimes = LDWS_DEFAULT_HOLD_TIMES;
    }while (0);

    return;
    //以下为调试用
    if (!fitLeft.empty() || !fitRight.empty()) {
        if (!fitLeft.empty()) {
            cv::line(perspective, fitLeft.top, fitLeft.bottom, cv::Scalar(255, 0, 0));
        }

        if (!fitRight.empty()) {
            cv::line(perspective, fitRight.top, fitRight.bottom, cv::Scalar(0, 0,255));
        }
    }
    cv::line(perspective,mStandardPosition.LB,mStandardPosition.LT,cv::Scalar(0,33,56));
    cv::line(perspective,mStandardPosition.RB,mStandardPosition.RT,cv::Scalar(0,33,56));
    this->capture->syncShow("perspective2",perspective);
}

/**
 * 分段填充边界线颜色
 * @param mat
 * @param detectLane
 */
void LDWS::fillFrame(cv::Mat &mat, PointRect &detectLane) {
    vector<cv::Point2f> srcPoints;
    vector<cv::Point2f> dstPoints;
    srcPoints.push_back(cv::Point2f(detectLane.LT.x,detectLane.LT.y));
    srcPoints.push_back(cv::Point2f(detectLane.RT.x,detectLane.RT.y));
    srcPoints.push_back(cv::Point2f(detectLane.RB.x,detectLane.RB.y));
    srcPoints.push_back(cv::Point2f(detectLane.LB.x,detectLane.LB.y));

    cv::perspectiveTransform(srcPoints,dstPoints,mWrapMatRsv);

    PointRect detectPoints;
    detectPoints.LT.x = dstPoints[0].x;
    detectPoints.LT.y = dstPoints[0].y;
    detectPoints.RT.x = dstPoints[1].x;
    detectPoints.RT.y = dstPoints[1].y;
    detectPoints.RB.x = dstPoints[2].x;
    detectPoints.RB.y = dstPoints[2].y;
    detectPoints.LB.x = dstPoints[3].x;
    detectPoints.LB.y = dstPoints[3].y;

    //得到10m,20m的纵坐标
    static int y5m = -1;
    static int y10m = -1;
    if (y5m < 0) {
        for (int i = mOriImage.rows - 1; i >= 0; --i) {
            int distance = Util::getDistance(i);
            if (distance >= 5 && distance != INVALID_DISTANCE && y5m == -1)  y5m = i;
            if (distance >= 10 && distance != INVALID_DISTANCE && y10m == -1) y10m = i;

            if (y10m >= 0)  break;
        }
    }

    int beginLeft =min(detectPoints.LB.x,detectPoints.LT.x);
    int endRight = max(detectPoints.RT.x,detectPoints.RB.x) + 1;

    //整体思路，先找出分割的行，填充成车面颜色，然后分开的几段滴水灌成不同的颜色

    //中间的分割线分两段,每段占三行
    int split[2] = {
            (detectPoints.LB.y > (y5m + 1) && y5m > 0) ? y5m -1 : 0,
            (detectPoints.LB.y > (y10m + 1) && y10m > 0) ? y10m -1 : 0,
    };
    for (int k = 0; k < sizeof(split)/sizeof(split[0]); ++k){
        if (split[k] == 0) continue;
        for  (int i = beginLeft; i < endRight; ++i){
            for (int j = split[k]; j < split[k] + 3; ++j) {
                if (j <0 || j >= mat.rows || i < 0 || i >= mat.cols){
                    //printf("rows:%d cols:%d j:%d i:%d\n",mat.rows,mat.cols,j,i);
                    continue;
                }
                Vec3b &value = mat.at<Vec3b>(j, i);
                if (value[0] == 128 && value[1] == 128 && value[2] == 128) {
                    value[0] = 255; value[1] = 0; value[2] = 0;
                }
            }
        }
    }

    //颜色从上往下分三段
    int color[3] = {
            (y10m -2 > 0 && (y10m - 2) >= detectPoints.LT.y) ? (y10m - 2) : 0,
            (y5m -2 > 0 && (y5m - 2) >= detectPoints.LT.y) ? (y5m - 2) : 0,
            (y5m +2 > 0 && (y5m + 2) <  detectPoints.LB.y) ? (y5m + 2) : 0,
    };

    cv::Scalar scalar[3] = {cv::Scalar(0,255,0),cv::Scalar(0,255,255),cv::Scalar(0,0,255)};
    //cv::Scalar scalar[3] = {cv::Scalar(128,128,128),cv::Scalar(128,128,128),cv::Scalar(128,128,128)};

    for (int x = beginLeft; x < endRight; ++x){
        for (int i = 0; i  < sizeof(color)/sizeof(color[0]); ++i){
            if (color[i] == 0) continue;

            if (color[i] <0 || color[i] >= mat.rows || x < 0 || x >= mat.cols){
                //printf("rows:%d cols:%d j:%d i:%d\n",mat.rows,mat.cols,color[i],x);
                continue;
            }
            Vec3b &value = mat.at<Vec3b>(color[i], x);
            if (value[0] == 128 && value[1] == 128 && value[2] == 128) {
                cv::floodFill(mat,cv::Point(x,color[i]),scalar[i]);
            }
        }
    }
}

void LDWS::matFillPolly(cv::Mat mat,cv::Point p1,cv::Point p2,cv::Point p3,cv::Point p4,cv::Scalar scalar){
    cv::Point points[1][4];
    points[0][0] = p1;
    points[0][1] = p2;
    points[0][2] = p3;
    points[0][3] = p4;
    const cv::Point* ppt[1] = {points[0]};
    int npt[] = {4};
    cv::fillPoly(mat,ppt,npt,1,scalar);
}
/**
 * 覆盖已检测到的轨道线
 * @param mat
 */
void LDWS::drawResult(cv::Mat &mat){
    if (Config::instance()->isLdwCalibration) {
        cv::line(mat,cv::Point(Config::instance()->ldwsLeft.x,Config::instance()->ldwsLeft.y),cv::Point(Config::instance()->ldwsTop.x,Config::instance()->ldwsTop.y),cv::Scalar(0,255,0));
        cv::line(mat,cv::Point(Config::instance()->ldwsRight.x,Config::instance()->ldwsRight.y),cv::Point(Config::instance()->ldwsTop.x,Config::instance()->ldwsTop.y),cv::Scalar(0,255,0));
    }
    if (mHoldTimes <= 0 || mLastShowMat.empty()){
        return;
    }
    do {
        UtilMutexAutoLock lock(mutex);
        --mHoldTimes;
        cv::addWeighted(mat(mROI), 1, mLastShowMat(mROI), 0.2, 0, mat(mROI));
    }while(0);
}

/**
 * 处理流程
 */
void LDWS::process(cv::Mat& mat) {
    //UtilTimeDiff diff("LDWS::process");

    this->mOriImage = mat.clone();

    if (this->mOriImage.empty()) {
        cout << "LDWS:empty image" << endl;
        return;
    }
    preProcess();
    detectLane();
}
