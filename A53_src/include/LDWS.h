//
// Created by wangxiaobao on 18-8-23.
//

#ifndef ADASPROJECT_LDWS_H
#define ADASPROJECT_LDWS_H

#include "define.h"
#include "Points.h"
#include "MessageType.h"

class MediaCapture;
class LDWS {
private:
    MediaCapture *capture;

    cv::Mat mOriImage;

    cv::Mat mWrapMat;
    cv::Mat mWrapMatRsv;
    cv::Rect mROI;
    PointRect mLastDetectLane;

    pthread_mutex_t mutex;

    cv::Mat mLastShowMat;

	PointRect mStandardPosition;
	cv::Rect mStandardROI;

	int mHoldTimes;
	HHSQ_ADAS_MESSAGE_S mMessageNotify;
	long mLastNotifyTime;
	int  mLastNotifyContent;

    void detectLane();
    void matFillPolly(cv::Mat mat,cv::Point p1,cv::Point p2,cv::Point p3,cv::Point p4,cv::Scalar scalar);
	void preProcess();
	PointLine fitLaneLine(int width,int height,std::vector<cv::Point> points);
	cv::Mat getPossibleLaneDomain(cv::Mat& mat);
	void adjustLane(int width,int height,PointRect& rect);
	PointLine getLeftLane(PointLine line);
	PointLine getRightLane(PointLine line);

	void resultLane(int width,PointRect& rect);
	void resultDraw(PointRect& rect);
	void resultFill(cv::Mat&mat, PointRect& rect);

    void getHistSection(cv::Mat&mat, int beginIn,int endIn,int& beginOut,int& endOut,int& maxIndexOut);

	void colorEnhance(cv::Mat& mat);

	void detectAlarm(PointRect& rect);
	void fillFrame(cv::Mat& mat,PointRect & rect);
public:
    bool isDebug;
    bool isLeftShift;
    bool isRightShift;
	LDWS(MediaCapture *mediaCapture);

    void process(cv::Mat& mat);
    void drawResult(cv::Mat &mat);
};


#endif //ADASPROJECT_LDWS_H
