//
// Created by wangxiaobao on 18-8-29.
//

#include "../include/TSR.h"
#include "../include/Predict.h"
#include "../include/Util.h"
#include "../include/MessageType.h"
#include "../include/Notify.h"
#include "../include/MediaCapture.h"



#define RECORD_SPEED_TIME 6
//#define RECORD_VALID_TIME 2//RECORD_SPEED_TIME/2
#define RECORD_SPEED_SUCCESS_RATE 2

#define INVALID_SPEED_VALUE -1

int TSR::showTsrCount = 0;
cv::Mat TSR::showLastSpeed;

pthread_mutex_t TSR::mutex = PTHREAD_MUTEX_INITIALIZER;


TSR::TSR(MediaCapture *mediaCapture, cv::Mat &mat) {
    mMediaCapture =  mediaCapture;
    oriImage = mat.clone();
    speedLimitPredict = INVALID_SPEED_VALUE;
}

/**
 * 去掉图片上方和下方的空白
 * @param analysisMat
 */
void TSR::adjustTopBtm(cv::Mat& analysisMat){
    int mid = analysisMat.rows /2;
    int top = mid;
    int bottom = mid;
    bool isTopOk = false;
    bool isBottomOk = false;
    for (int i = mid; i >= 0; --i) {
        cv::Mat row;
        if (!isTopOk && top > 0) {
            row = analysisMat.rowRange(top,top + 1);
            if ((unsigned char)*std::max_element(row.data,row.data + row.cols) == 0){
                isTopOk = true;
            } else {
                top --;
            }
        }

        if (!isBottomOk && bottom < analysisMat.rows) {
            row = analysisMat.rowRange(bottom,bottom + 1);
            if ((unsigned char)*std::max_element(row.data,row.data + row.cols) == 0){
                isBottomOk = true;
            } else {
                bottom ++;
            }
        }
    }

    //cout <<"top:" << top << " btm:" << bottom << endl;
    analysisMat = cv::Mat(analysisMat,cv::Rect(0,top,analysisMat.cols,bottom - top));
}

/**
 * 最高限速的黑白转换
 * @param analysisMat
 */
void TSR::adjustWhiteBlack(cv::Mat& analysisMat){
    int wCnt = 0;
    int bCnt = 0;
    for (int i= 0; i < analysisMat.rows; ++i) {
        for (int j = 0; j < analysisMat.cols; ++j) {
            if (analysisMat.at<unsigned char>(i,j) == 255)  wCnt++;
            else bCnt++;
        }
    }

    //cout << "white:" << wCnt << " black:" << bCnt << endl;

    if (wCnt > bCnt) {
        analysisMat = 255 - analysisMat;
        cv::floodFill(analysisMat, cv::Point(0, 0), cv::Scalar(0));
        cv::floodFill(analysisMat, cv::Point(analysisMat.cols-1, 0), cv::Scalar(0));
        cv::floodFill(analysisMat, cv::Point(0, analysisMat.rows-1), cv::Scalar(0));
        cv::floodFill(analysisMat, cv::Point(analysisMat.cols-1, analysisMat.rows-1), cv::Scalar(0));
    }
}

/**
 * 纵向分割，把每个数字的矩形找出来
 * @param mat
 * @return
 */
vector<int> TSR::getDigitRect(cv::Mat& mat){
    vector<int> xPoint;

    //cout << "mat cols:" << mat.cols << " rows:" << mat.rows << endl;
    bool isBeforeWhile = false;
    bool isFoundWhite = false;
    for (int i = 0; i < mat.cols; ++i) {
        cv::Mat col = mat.colRange(i,i + 1);
        unsigned char value = 0;

        for (int j = 0; j < mat.rows; ++j) {
            if (value < mat.at<unsigned char>(j,i)){
                value = mat.at<unsigned char>(j,i);
            }
        }

        if (value != 0){
            if (!isFoundWhite) {
                xPoint.push_back(i == 0 ? 0 : (i - 1)); //第一个白色线起点
                isFoundWhite = true;
            }

            isBeforeWhile = true;
        } else {   //找白变黑
            if (!isFoundWhite){
                continue;
            }

            if (isBeforeWhile) {
                xPoint.push_back(i); //第一个起点
            }

            isBeforeWhile = false;
        }
    }

    if (isBeforeWhile) {
        xPoint.push_back(mat.cols-1);
    }

    return xPoint;
}

/**
 * 根据边界找里面的数字
 * @param xPoint
 * @param mat
 * @return
 */
int  TSR::getLimitSpeedPredict(vector<int> &xPoint, cv::Mat &mat){
    int speed = 0;
    int count = xPoint.size() - 1;
    if (count < 1 || count > 3) {
        return INVALID_SPEED_VALUE;
    }

    time_t t;
    time(&t);
    long cc = (long)t;
    int predict = 0;
    for (int i = 0; i < count; ++i) {
        cc++;
        cv::Mat splitMat =cv::Mat(mat,cv::Rect(xPoint[i],0,xPoint[i+1]-xPoint[i],mat.rows));
        //cv::imshow("9",splitMat);

        //cout << "x:" << xPoint[i] << " --- " << xPoint[i+1] << endl;

        predict = Predict::instance()->predict(splitMat);
        //cout << "predict:" << predict << endl;

        if (predict < 0) {
            return INVALID_SPEED_VALUE;
        } else {
            speed = speed * Predict::instance()->offset() + predict;
        }
    }
    int speedPredict = speed;
    speed = Predict::instance()->realSpeed(speed);

    if (speed < 0) {
        return INVALID_SPEED_VALUE;
    }

    //速度不是0或5结尾，无效
    if (((speed % 10) != 0) && ((speed % 10) != 5) ) {
        return INVALID_SPEED_VALUE;
    }

    //只有一个检索区域时速度不能小于10
    if (count == 1 && speed < 10) {
        return INVALID_SPEED_VALUE;
    }

    //小于40公里或大于120公里无效
    if (speed < 40 || speed > 120) {
        return INVALID_SPEED_VALUE;
    }

    return speedPredict;
}

//#define TSR_SHOW_DEBUG

int TSR::process(cv::Rect &rect) {

    //UtilTimeDiff diff("TSR::process");

    cv::Mat grayImage;
    //cv::Rect rect(0,0,oriImage.cols,oriImage.rows/2 );
    //cv::Rect rect(0,0,oriImage.cols,TSR_CUT_HEIGHT);//矩形

    grayImage = oriImage(rect);//裁剪
#ifdef TSR_SHOW_DEBUG
    //this->mMediaCapture->syncShow("tsr gray",grayImage);
    cv::Mat titimg=grayImage.clone();
#endif
    cv::cvtColor(grayImage,grayImage,cv::COLOR_RGB2GRAY);//灰度

    cv::Mat smooth = grayImage;
    cv::GaussianBlur(grayImage,smooth,cv::Size(3, 3), 0, 0);//率波
    vector<cv::Vec3f> circles;
    /*
     * @PARAM_PARAM_2
     * 减小错误圆的识别率
     * */
    //cv::imshow("fdsfsd",smooth);
    cv::HoughCircles(smooth, circles, CV_HOUGH_GRADIENT, 1, 10, 120, PARAM_PARAM_2, 6, 35);//找圆

    int detectSpeed = INVALID_SPEED_VALUE;
    for (int i = 0; i < (int)circles.size(); ++i) {

#ifdef TSR_SHOW_DEBUG
        cv::circle(titimg,cv::Point(circles[i][0],circles[i][1]),circles[i][2],cv::Scalar(0,0,255));
        cv::imshow("c500",titimg);
#endif
        //去掉部分圆框
        float radius = circles[i][2];

        radius -= 1.5 / 8 * radius;
        int width = radius*2;
        int height =  radius*2;
        int x = circles[i][0] - radius;
        int y = circles[i][1] - radius;

        if (x < 0) x = 0;
        if (y < 0) y = 0;

        if ((circles[i][0] + radius) > grayImage.cols) width = grayImage.cols - circles[i][0] + radius;
        if ((circles[i][1] + radius) > grayImage.rows) height = grayImage.rows - circles[i][1] + radius;

        cv::Mat analysisMat =cv::Mat(grayImage,cv::Rect(x,y,width,height));
        cv::threshold(analysisMat,analysisMat,0,255,CV_THRESH_OTSU);
#ifdef TSR_SHOW_DEBUG
        cv::Mat analysisMat2 = analysisMat.clone();
#endif
        int speedPredict = Predict::instance()->predict(analysisMat);

#ifdef TSR_SHOW_DEBUG
        cv::resize(analysisMat2,analysisMat2,cv::Size(100,100));
        cv::imshow("tsr",analysisMat2);
        cv::waitKey(0);
#endif
        if (speedPredict < 0) {
            continue;
        }

        if (detectSpeed  < speedPredict) {
            detectSpeed  = speedPredict;
        }
    }

    return detectSpeed;
}

/**
 * 处理
 */
#define TSR_CUT_HEIGHT oriImage.rows/2
void TSR::process(){
    cv::Rect rect(0,0,oriImage.cols,TSR_CUT_HEIGHT);//矩形
    int speed = process(rect);

    cv::Rect rect2(oriImage.cols/2,TSR_CUT_HEIGHT,oriImage.cols/2,oriImage.rows-TSR_CUT_HEIGHT);//矩形
    int speed2 = process(rect2);

    this->speedLimitPredict = speed > speed2 ? speed : speed2;
    calcOkRate();


#if 0
    UtilTimeDiff diff("TSR::process");

    cv::Mat grayImage;
    //cv::Rect rect(0,0,oriImage.cols,oriImage.rows/2 );
    cv::Rect rect(0,0,oriImage.cols,TSR_CUT_HEIGHT);//矩形

    grayImage = oriImage(rect);//裁剪
    //cv::Mat titimg=grayImage.clone();
    cv::cvtColor(grayImage,grayImage,cv::COLOR_RGB2GRAY);//灰度

    cv::Mat smooth = grayImage;
    cv::GaussianBlur(grayImage,smooth,cv::Size(3, 3), 0, 0);//率波
    vector<cv::Vec3f> circles;
    /*
     * @PARAM_PARAM_2
     * 减小错误圆的识别率
     * */
    cv::HoughCircles(smooth, circles, CV_HOUGH_GRADIENT, 1, 10, 120, PARAM_PARAM_2, 6, 35);//找圆

    this->speedLimitPredict = INVALID_SPEED_VALUE;
    for (int i = 0; i < circles.size(); ++i) {


        //cv::circle(titimg,cv::Point(circles[i][0],circles[i][1]),circles[i][2],cv::Scalar(0,0,255));
        //cv::imshow("c500",titimg);
        //去掉部分圆框
        float radius = circles[i][2];

        radius -= 1.5 / 8 * radius;
        int width = radius*2;
        int height =  radius*2;
        int x = circles[i][0] - radius;
        int y = circles[i][1] - radius;

        if (x < 0) x = 0;
        if (y < 0) y = 0;

        if ((circles[i][0] + radius) > grayImage.cols) width = grayImage.cols - circles[i][0] + radius;
        if ((circles[i][1] + radius) > grayImage.rows) height = grayImage.rows - circles[i][1] + radius;

        cv::Mat analysisMat =cv::Mat(grayImage,cv::Rect(x,y,width,height));
        cv::threshold(analysisMat,analysisMat,0,255,CV_THRESH_OTSU);
        //cv::Mat analysisMat2 = analysisMat.clone();
        int speedPredict = Predict::instance()->predict(analysisMat);

        //cv::resize(analysisMat2,analysisMat2,cv::Size(100,100));
        //cv::imshow("tsr",analysisMat2);
        //cv::waitKey(0);

        if (speedPredict < 0) {
            continue;
        }


        if (this->speedLimitPredict < speedPredict) {
            this->speedLimitPredict = speedPredict;
        }
    }

    if (speedLimitPredict > 0)
        cout << "RESULT speedLimitPredict:" << speedLimitPredict << endl;

    calcOkRate();
#endif
}



/**
 * 图片去噪
 * @param mat
 * @return
 */
cv::Mat TSR::removeNoise(cv::Mat mat){

    //https://blog.dn.net/ysc6688/article/details/50772382/
    for (int n = 0; n < 2; ++n) {
        int check = n * 255;
        int set = 255 - check;

        int noiseThr = 10;
        int color = 1;
        for (int i = 0; i < mat.rows; ++i) {
            for (int j = 0; j < mat.cols; ++j) {
                if (mat.at<unsigned char>(i, j) == check) {
                    cv::floodFill(mat, cv::Point(j, i), cv::Scalar(color));
                    color++;
                }
            }
        }

        int colorCount[256] = {0};
        for (int i = 0; i < mat.rows; ++i) {
            for (int j = 0; j < mat.cols; ++j) {
                if (mat.at<unsigned char>(i, j) != set) {
                    colorCount[mat.at<unsigned char>(i, j)]++;
                }
            }
        }

        for (int i = 0; i < mat.rows; ++i) {
            for (int j = 0; j < mat.cols; ++j) {
                if (colorCount[mat.at<unsigned char>(i, j)] < noiseThr) {
                    mat.at<unsigned char>(i, j) = set;
                } else {
                    mat.at<unsigned char>(i, j) = check;
                }
            }
        }
    }

    return mat;
}

int TSR::getResultPredict(int* data,int count){
    std::map<int,int> recordCount;
    for (int i = 0; i < count; ++i,++data){
        if (*data == INVALID_SPEED_VALUE) {
            continue;
        }

        if (recordCount.find(*data) == recordCount.end()){
            recordCount[*data] = 1;
        } else {
            recordCount[*data] = recordCount[*data] + 1;
        }
    }

    if (recordCount.size() == 0) {
        return INVALID_SPEED_VALUE;
    }

    cout << "\n---------------------------";
    int key = INVALID_SPEED_VALUE;
    for (std::map<int,int>::iterator it = recordCount.begin(); it != recordCount.end(); ++it){

        if (it->second < RECORD_SPEED_SUCCESS_RATE){
            continue;
        }

        key = it->first;

        cout << "\nfind speed:" << (int) key << " count:" << (int)it->second;
    }

    cout << endl <<  "last select speed:" << (int) key << "----------------------------" << endl;

    return key;
}

void TSR::calcOkRate(){

    static int recordPredict[RECORD_SPEED_TIME] = {0};
    static long lastSpeedEnd = 0;
    static long lastSpeedStart = 0;

    if (lastSpeedStart > lastSpeedEnd) lastSpeedStart = lastSpeedEnd;

    if (speedLimitPredict < 0) {
        recordPredict[lastSpeedEnd % RECORD_SPEED_TIME] = INVALID_SPEED_VALUE;

        lastSpeedEnd++;
        if (lastSpeedEnd - lastSpeedStart >= RECORD_SPEED_TIME){
            lastSpeedStart++;
        }
    } else {

        recordPredict[lastSpeedEnd % RECORD_SPEED_TIME] = speedLimitPredict;
        if (lastSpeedEnd - lastSpeedStart >= RECORD_SPEED_TIME){
            lastSpeedStart++;
        }

        lastSpeedEnd++;

        int countRelation = 0;
        int countSame = 0;
        int countValid = 0;
        cout << "----------------" << endl;
        for (long i = lastSpeedStart; i < lastSpeedEnd; ++i) {

            if (i == lastSpeedStart) cout << "RESULT ";

            int pos = i % RECORD_SPEED_TIME;

            cout << recordPredict[pos] << " _ ";
            if (Predict::instance()->realSpeed(recordPredict[pos]) == Predict::instance()->realSpeed(speedLimitPredict)){
                countRelation++;
            }
            if (recordPredict[pos] == speedLimitPredict ) {
                countSame++;
            }

            if (recordPredict[pos] != INVALID_SPEED_VALUE ) {
                countValid++;
            }
        }

        /**
         * 预测是否有效依据
         * 1：关联预测值（速度相同）的概率大于等于RECORD_SPEED_SUCCESS_RATE；
         * 2：同一预测图片的出现根率不小于RECORD_SPEED_SAME_RATE
         */
        if (countRelation< RECORD_SPEED_SUCCESS_RATE){
            return;
        }

        showTsrCount = 0;

        speedLimitPredict   = getResultPredict(recordPredict,RECORD_SPEED_TIME);
        if (speedLimitPredict == INVALID_SPEED_VALUE) {
            return;
        }
        cout << "############last:speedLimitPredict:" << speedLimitPredict << endl;

        cv::Mat showMat = cv::Mat(100,100,CV_8UC3,cv::Scalar(255,255,255));
        cv::circle(showMat,cv::Point(50,50),45,CV_RGB(255,0,0),10);
        int showSpeed = Predict::instance()->realSpeed(speedLimitPredict);

        char text[10] = {0};
        sprintf(text,"%d",showSpeed);

        if (showSpeed < 100) {
            cv::putText(showMat, text, cv::Point(12, 70), cv::FONT_HERSHEY_SIMPLEX, 1.8, cv::Scalar(0, 0, 0), 5);
        } else{
            cv::putText(showMat, text, cv::Point(9, 65), cv::FONT_HERSHEY_SIMPLEX, 1.3, cv::Scalar(0, 0, 0), 5);
        }

        HHSQ_ADAS_MESSAGE_S message;
        message.type = HHSQ_ADAS_TYPE_TSR;
        message.command = HHSQ_COMMAND_TSR_LIMIT_BEGIN;
        message.content[0] = showSpeed;

        Notify::notify(message);

        UtilMutexAutoLock lock(mutex);
        showLastSpeed.release();
        showLastSpeed = showMat;
        showTsrCount = 35;
    }
}


void TSR::drawResult(cv::Mat& mat){

    UtilMutexAutoLock lock(mutex);

    if (showTsrCount <=0 || showLastSpeed.empty()) {
        return;
    }

    showTsrCount--;

    showLastSpeed = Util::RGB2YUV(showLastSpeed);

    cv::Mat roi = mat(cv::Rect(mat.cols /2 - 50,0,100,100));
    showLastSpeed.convertTo(roi,roi.type(),1,0);

    return;
}
