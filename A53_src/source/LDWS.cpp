#if 0
 /*
 * 1.首先确定三角形三个点的位置
 * 2.确定车头多出来的部分,用index4接收
 * 3.再根据图片画出车道线的实际距离
 * 主要问题:ROI区域的干扰线不好排除
 * */
#include "../include/LDWS.h"
#include "../include/Points.h"
#include "../include/Notify.h"
#include "../include/Util.h"
#include "../include/Adapter.h"
#include "../include/anyarg.h"
#include "../include/config.h"
using namespace cv;
cv::Mat LDWS::mWrapMat;
cv::Mat LDWS::mWrapInverseMat;
cv::Mat LDWS::laneFillPoly;
cv::Rect LDWS::laneFillPolyROI;
PointRect LDWS::s_pointsLeft;
PointRect LDWS::s_pointsRight;
pthread_mutex_t LDWS::mutex = PTHREAD_MUTEX_INITIALIZER;
#define CHECKED_LANE_HOLD_TIME 30
#define CHECKED_LANE_SHIFT_TIME 0
static float xldata= 290.0;//左边x的横坐标
static float xrdata= 900.0;//右边x的横坐标
static float xtdata= 628.0;//三角形顶点的横坐标
static float xtlabel= 522.0;//三角形顶点的纵坐标
static float ytlabel= 565.0;//topy的值 绿线30m
static float yblabel= 720.0;//buttomy的值
float index1=xldata+136,index2=xrdata-xldata-184,index3=720/5*4,index4=0,index5;//index4为车头多余部分
float xldata2 = (ytlabel - xtlabel) / (yblabel - xtlabel) * (xldata - xtdata) + xtdata;//以ytlabel为纵坐标,求得左斜线的x坐标
float xldata3 = (ytlabel - xtlabel) / (yblabel - xtlabel) * (xrdata - xtdata) + xtdata;//以ytlabel为纵坐标,求得右斜线的x坐标
LDWS::LDWS(MediaCapture *mediaCapture,cv::Mat &mat) {
    this->mOriImage = mat.clone();
    this->mHeight = mat.rows;
    this->mWidth = mat.cols;
    this->capture = mediaCapture;
    
    
    this->isLeftShift = false;
    this->isRightShift = false;
	cv::Rect rect;
	rect=Rect(index1,index3-index4, index2, mOriImage.rows-index3-index4);
	index5=index3-index4;
	this->LROI = mOriImage(rect)(Rect(0,0,mOriImage(rect).cols/3,mOriImage(rect).rows));
    this->RROI = mOriImage(rect)(Rect(mOriImage(rect).cols*0.6,0,mOriImage(rect).cols*0.3,mOriImage(rect).rows));
}

PointLine fitLaneLine(int width,int height,std::vector<cv::Point> points){
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


/*
static float xldata= 290.0;//左边x的横坐标
static float xrdata= 900.0;//右边x的横坐标
static float xtdata= 628.0;//三角形顶点的横坐标
static float xtlabel= 522.0;//三角形顶点的纵坐标
static float ytlabel= 565.0;//topy的值 绿线30m
static float yblabel= 720.0;//buttomy的值
*/
void matFillPolly(cv::Mat mat,cv::Point p1,cv::Point p2,cv::Point p3,cv::Point p4,cv::Scalar scalar){
    cv::Point points[1][4];
    points[0][0] = p1;
    points[0][1] = p2;
    points[0][2] = p3;
    points[0][3] = p4;

    const cv::Point* ppt[1] = {points[0]};
    int npt[] = {4};
    cv::fillPoly(mat,ppt,npt,1,scalar);
}


float getX2(int y2) {
    float x2 = (xtdata - xldata) * 1.0 / (yblabel - xtlabel) * (yblabel - y2) + xldata;
    return x2;
}

float getX3(int y2) {
    float x2 = xrdata - (yblabel - y2) * 1.0 * (xrdata - xtdata) / (yblabel - xtlabel);
    return x2;
}
cv::Mat g_mWrapMat;
cv::Mat g_mWrapMatRsv;
cv::Mat g_mWrapInverseMat;
cv::Mat get_perspective_mat()
{
    if (!g_mWrapMat.empty()) {
        return g_mWrapMat;
    }
    int height = yblabel - ytlabel;
    vector<cv::Point2f>mOrigPoints;
    vector<cv::Point2f>mDestPoints;
    mOrigPoints.push_back(cv::Point2f(getX2(ytlabel)-20, ytlabel));
    mOrigPoints.push_back(cv::Point2f(getX3(ytlabel)+20, ytlabel));
    mOrigPoints.push_back(cv::Point2f(xrdata+50, yblabel));
    mOrigPoints.push_back(cv::Point2f(xldata-50, yblabel));

    mDestPoints.push_back(cv::Point2f(0, 0));
    mDestPoints.push_back(cv::Point2f(xrdata-xldata, 0));
    mDestPoints.push_back(cv::Point2f(xrdata-xldata, height));
    mDestPoints.push_back(cv::Point2f(0, height));

    g_mWrapMat = cv::findHomography(mOrigPoints,mDestPoints);
    g_mWrapMatRsv = cv::findHomography(mDestPoints,mOrigPoints);
    g_mWrapInverseMat = g_mWrapMat.inv();


    return g_mWrapMat ;
}

cv::Mat calcHists(cv::Mat& mat) {
    cout << "----------------------\n";
    vector<int> data;
    for (int i = 0; i < mat.cols; ++i){
        int colValue = 0;
        for (int j = 0; j < mat.rows; ++j){
            unsigned char value = mat.at<unsigned char>(j,i);
            if (value > 0) colValue++;
        }

        data.push_back(colValue > 10 ? colValue  : 0);
    }

    int leftSave[3] = {0};
    int rightSave[3] = {0};
    int recordIndex = -1;
    int recordData = 0;
    for (int i = 0; i < mat.cols /2; ++i){
        if (data[i] > 0){
            if (recordIndex < 0){
                recordIndex = i;
                recordData = data[i];
            } else {
                recordData += data[i];
            }
        } else {
            if (recordIndex < 0) {recordData = 0;  continue;}

            //if (i - recordIndex > leftSave[1] - leftSave[0]){
            if (recordData > leftSave[2]){
                leftSave[0] = recordIndex;
                leftSave[1] = i;
                leftSave[2] = recordData;
            }

            recordIndex = -1;
            recordData = 0;
        }
    }
    recordIndex = -1;
    recordData = 0;
    for (int i = mat.cols /2; i < mat.cols ; ++i){
        if (data[i] > 0){
            if (recordIndex < 0){
                recordIndex = i;
                recordData = data[i];
            } else {
                recordData += data[i];
            }
        } else {
            if (recordIndex < 0) {recordData = 0;  continue;}
            //if (i - recordIndex > rightSave[1] - rightSave[0]){
            if (recordData > rightSave[2]){
                rightSave[0] = recordIndex;
                rightSave[1] = i;
                rightSave[2] = recordData;
            }

            recordIndex = -1;
            recordData = 0;
        }
    }

    printf("last:left:%4d %4d %4d right:%4d %4d %4d",leftSave[0],leftSave[1],leftSave[2],rightSave[0],rightSave[1],rightSave[2]);
    if (leftSave[2] > rightSave[2]){
        cout << " left good" << endl;
    } else {
        cout << " right good" << endl;
    }
    cv::Mat posIndex = cv::Mat::zeros(1,6,CV_32SC1);
    posIndex.at<int>(0,0) = leftSave[0];
    posIndex.at<int>(0,1) = leftSave[1];
    posIndex.at<int>(0,2) = rightSave[0];
    posIndex.at<int>(0,3) = rightSave[1];
    posIndex.at<int>(0,4) = leftSave[2];
    posIndex.at<int>(0,5) = rightSave[2];
    return posIndex;
}

PointRect getStandPos(cv::Mat& mat){
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

    PointRect result;
    result.LB = leftBtm;
    result.LT = leftTop;
    result.RT = rightTop;
    result.RB = rightBtm;
    return result;
}

vector<cv::Vec4i>lines2;
static float leftbuttom=0.0;
static float lefttop=0.0;
static float rightbuttom=0.0;
static float righttop=0.0;
void LDWS::getWrapMat() {
	ROI=Mat(LROI.rows,LROI.cols+ RROI.cols,RROI.type());
    LROI.colRange(0,LROI.cols).copyTo(ROI.colRange(0,LROI.cols));
    RROI.colRange(0,RROI.cols).copyTo(ROI.colRange(LROI.cols,ROI.cols));
    if (!mWrapMat.empty()){
        return;
    }
    //ROI = mOriImage(cv::Rect(xldata,xtlabel,xrdata - xldata,mOriImage.rows - xtlabel));

    cv::Mat oriClone = mOriImage.clone();
    cv::line(oriClone,
             cv::Point2f(xldata, yblabel),
             cv::Point2f(xrdata, yblabel),
             cv::Scalar(255,255,0)
            );
    cv::line(oriClone,
             cv::Point2f(xrdata, yblabel),
             cv::Point2f(xtdata, xtlabel),
             cv::Scalar(255,255,0)
    );
    cv::line(oriClone,
             cv::Point2f(xtdata, xtlabel),
             cv::Point2f(xldata, yblabel),
             cv::Scalar(255,255,0)
    );

    cv::Mat stand = cv::Mat::zeros(mOriImage.rows,mOriImage.cols,CV_8UC3);
    cv::line(stand ,
             cv::Point2f(xldata, yblabel),
             cv::Point2f(xrdata, yblabel),
             cv::Scalar(255,255,0)
    );
    cv::line(stand ,
             cv::Point2f(xrdata, yblabel),
             cv::Point2f(xtdata, xtlabel),
             cv::Scalar(255,255,0)
    );
    cv::line(stand ,
             cv::Point2f(xtdata, xtlabel),
             cv::Point2f(xldata, yblabel),
             cv::Scalar(255,255,0)
    );
    cv::warpPerspective(stand , stand, get_perspective_mat(), cv::Size(xrdata-xldata, yblabel - ytlabel));
    cv::Mat standGray;
    Adapter::instance()->Rgb2Gray(stand,standGray);
    PointRect standPos = getStandPos(standGray);

    cv::line(stand,standPos.LT,standPos.LB,cv::Scalar(0,33,56));
    cv::line(stand,standPos.RT,standPos.RB,cv::Scalar(0,33,56));

    //cv::rectangle(oriClone,cv::Point2f(xldata, yblabel), cv::Point2f(xrdata, yblabel - xtdata),cv::Scalar(0,255,255));
    cv::Mat perspective;
    cv::warpPerspective(mOriImage , perspective, get_perspective_mat(), cv::Size(xrdata-xldata, yblabel - ytlabel));


    cv::imshow("perspective",perspective);
    cv::imshow("oriClone ",oriClone );

    std::vector<cv::Point> leftPoints;
    std::vector<cv::Point> rightPoints;

    cv::Mat perspectiveCanny;
    Adapter::instance()->Rgb2Gray(perspective,perspectiveCanny);
    GaussianBlur(perspectiveCanny, perspectiveCanny, Size(5, 5), 0, 0, BORDER_DEFAULT);
    //Adapter::instance()->canny(perspectiveCanny,perspectiveCanny,0,200);
    cv::threshold(perspectiveCanny,perspectiveCanny,130,255,cv::THRESH_BINARY);

    cv::Mat posIndex = calcHists(perspectiveCanny);

    cv::HoughLinesP(perspectiveCanny,lines,4,CV_PI/180,10, 20, 5);
    for (int i = 0; i < lines.size(); ++i) {
        //cout << "found:" << (int)lines.size() << endl;
        if (fabs(lines[i][3] - lines[i][1]) / fabs(lines[i][2] - lines[i][0]) < 0.6){
            continue;
        }
        cv::line(perspective,cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]),cv::Scalar(0,255,255));
        if (lines[i][0] <= posIndex.at<int>(0,1) && lines[i][0] >= posIndex.at<int>(0,0)) {
            leftPoints.push_back(cv::Point(lines[i][0],lines[i][1]));
            leftPoints.push_back(cv::Point(lines[i][2],lines[i][3]));
        } else if (lines[i][0] <= posIndex.at<int>(0,3) && lines[i][0] >= posIndex.at<int>(0,2)) {
            rightPoints.push_back(cv::Point(lines[i][0],lines[i][1]));
            rightPoints.push_back(cv::Point(lines[i][2],lines[i][3]));
        }
    }

    PointLine fitLeft,fitRight;
    fitLeft = fitLaneLine(perspective.cols,perspective.rows,leftPoints);
    fitRight = fitLaneLine(perspective.cols,perspective.rows,rightPoints);

    cv::Mat perspectiveFile = cv::Mat::zeros(perspective.rows,perspective.cols,CV_8UC3);

    if (!fitLeft.empty() && fitRight.empty()){

        cv::Point rightTop;
        cv::Point rightBtm;

        rightTop.x = standPos.RT.x + fitLeft.top.x - standPos.LT.x;
        rightTop.y = standPos.RT.y + fitLeft.top.y - standPos.LT.y;

        rightBtm.x = standPos.RB.x + fitLeft.bottom.x - standPos.LB.x;
        rightBtm.y = standPos.RB.y + fitLeft.bottom.y - standPos.LB.y;

        matFillPolly(perspectiveFile,fitLeft.top,rightTop,rightBtm,fitLeft.bottom,cv::Scalar(0,255,0));
        cv::line(perspectiveFile,standPos.RT,standPos.RB,cv::Scalar(43,12,54));
        cv::imshow("perF",perspectiveFile);
        cv::Mat perspectiveShow = cv::Mat::zeros(mOriImage.rows,mOriImage.cols,CV_8UC3);

        cv::warpPerspective(perspectiveFile , perspectiveShow , g_mWrapMatRsv, cv::Size(mOriImage.cols, mOriImage.rows));

        cv::imshow("perF2",perspectiveShow);
        cv::Mat s = mOriImage.clone();
        cv::addWeighted(s,1,perspectiveShow,0.8,0.1,s);
        cv::imshow("ssss",s);
    } else if (fitLeft.empty() && !fitRight.empty()){
        cv::Point leftTop;
        cv::Point leftBtm;

        leftTop.x = standPos.LT.x + fitRight.top.x - standPos.RT.x;
        leftTop.y = standPos.LT.y + fitRight.top.y - standPos.RT.y;

        leftBtm.x = standPos.LB.x + fitRight.bottom.x - standPos.RB.x;
        leftBtm.y = standPos.LB.y + fitRight.bottom.y - standPos.RB.y;

        matFillPolly(perspectiveFile,fitRight.top,leftTop,leftBtm,fitRight.bottom,cv::Scalar(0,255,0));
        cv::line(perspectiveFile,standPos.RT,standPos.RB,cv::Scalar(43,12,54));
        cv::imshow("perF",perspectiveFile);
        cv::Mat perspectiveShow = cv::Mat::zeros(mOriImage.rows,mOriImage.cols,CV_8UC3);

        cv::warpPerspective(perspectiveFile , perspectiveShow , g_mWrapMatRsv, cv::Size(mOriImage.cols, mOriImage.rows));

        cv::imshow("perF2",perspectiveShow);
        cv::Mat s = mOriImage.clone();
        cv::addWeighted(s,1,perspectiveShow,0.8,0.1,s);
        cv::imshow("ssss",s);
    } else if (!fitLeft.empty() && !fitRight.empty()){

        matFillPolly(perspectiveFile,fitRight.top,fitLeft.top,fitLeft.bottom,fitRight.bottom,cv::Scalar(0,255,0));
        cv::line(perspectiveFile,standPos.RT,standPos.RB,cv::Scalar(43,12,54));
        cv::imshow("perF",perspectiveFile);
        cv::Mat perspectiveShow = cv::Mat::zeros(mOriImage.rows,mOriImage.cols,CV_8UC3);

        cv::warpPerspective(perspectiveFile , perspectiveShow , g_mWrapMatRsv, cv::Size(mOriImage.cols, mOriImage.rows));

        cv::imshow("perF2",perspectiveShow);
        cv::Mat s = mOriImage.clone();
        cv::addWeighted(s,1,perspectiveShow,0.8,0.1,s);
        cv::imshow("ssss",s);
    }


    if (!fitLeft.empty() || !fitRight.empty()) {
        if (!fitLeft.empty()) {
            cv::line(perspective, fitLeft.top, fitLeft.bottom, cv::Scalar(255, 0, 0));
        }

        if (!fitRight.empty()) {
            cv::line(perspective, fitRight.top, fitRight.bottom, cv::Scalar(0, 255, 0));
        }
    }
    cv::line(perspective,standPos.LB,standPos.LT,cv::Scalar(0,33,56));
    cv::line(perspective,standPos.RB,standPos.RT,cv::Scalar(0,33,56));
    cv::imshow("perspective2",perspective);
    cv::imshow("perspectiveC",perspectiveCanny);

    lines.clear();
    leftPoints.clear();
    rightPoints.clear();
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat canny_output;
	GaussianBlur(ROI, ROI, Size(5, 5), 0, 0, BORDER_DEFAULT);
	Adapter::instance()->Rgb2Gray(ROI,ROI);
	Laplacian(ROI, ROI, CV_8UC1, 3, 1, 0, BORDER_DEFAULT);

	//cv::imshow("Laplacian",ROI);
#if 0
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(ROI, ROI, element);
    cv::imshow("dilate",ROI);
    erode(ROI, ROI, element);
    cv::imshow("erode",ROI);
#endif
    Adapter::instance()->canny(ROI,canny_output,50,200);
    cv::HoughLinesP(canny_output,lines,4,CV_PI/180,10, 20, 5);

    //cv::imshow("canny_output",canny_output);
    cv::waitKey(1);


    int leftcount=0;
    int rightcount=0;
    cvtColor(ROI,ROI,COLOR_GRAY2RGB);
    for (size_t i=0;i<lines.size();i++){
        if(lines[i][0]<520){
            float a=(float)(lines[i][3]-lines[i][1])/(lines[i][2]-lines[i][0]);
            float b=fabs(a);
            if(b>0.5&&b<2){
                if(b>1&&lines[i][0]<ROI.cols/2){
                } else{
                    
                    lines2.push_back(lines[i]);
                    if (a < 0){leftcount++; leftPoints.push_back(cv::Point(lines[i][0],lines[i][1]));leftPoints.push_back(cv::Point(lines[i][2],lines[i][3]));}
                    if (a > 0){rightcount++;rightPoints.push_back(cv::Point(lines[i][0],lines[i][1]));rightPoints.push_back(cv::Point(lines[i][2],lines[i][3]));}
                }
            }
        }
    }


    fitLeft = fitLaneLine(ROI.cols,ROI.rows,leftPoints);
    fitRight = fitLaneLine(ROI.cols,ROI.rows,rightPoints);

    if (!fitLeft.empty() || !fitRight.empty()) {
        cv::Mat showROI = ROI.clone();
        if (!fitLeft.empty()) {
            cv::line(showROI, fitLeft.top, fitLeft.bottom, cv::Scalar(255, 0, 0));
        }

        if (!fitRight.empty()) {
            //printf("fit right:x0:%d y0:%d x1:%d y1:%d\n",fitRight[0].x,fitRight[0].y,fitRight[1].x,fitRight[1].y);
            cv::line(showROI, fitRight.top, fitRight.bottom, cv::Scalar(0, 255, 0));
        }
        //cv::imshow("showROI",showROI);
        cv::waitKey(1);
    }
#if 0
    if (leftcount>0){
        for (int il = 0; il < leftPoints.size(); ++il){
            cout << "show:" << leftPoints[il].x << " " << leftPoints[il].y << endl;
        }
        cv::Mat showROI = ROI.clone();
        Vec4f line;
        fitLine(leftPoints, line, CV_DIST_L2, 0, 0.01, 0.01);

        double k = line[1] / line[0];

        //计算直线的端点(y = k(x - x0) + y0)
        cv::Point point1, point2;
        point1.x = 0;
        point1.y = k * (0 - line[2]) + line[3];
        point2.x = 640;
        point2.y = k * (640 - line[2]) + line[3];

        point2.y = 0;
        point2.x = (0-line[3])/k+line[2];

        cv::line(showROI,point1,point2,cv::Scalar(255,0,0));
        cv::imshow("showROI",showROI);
        cv::waitKey(1);
        std::cout << "left---->line: (" << (int)line[0] << "," << (int)line[1] << ")(" << (int)line[2] << "," << (int)line[3] << ")\n";
    }
#endif
    sort(lines2.begin(),lines2.end(),[](Vec4i a, Vec4i b){return a[0] < b[0];});
    Config::instance()->ldwsLeft.x = xldata;// = cv::Point(xldata,yblabel);
    Config::instance()->ldwsLeft.y = yblabel;
    Config::instance()->ldwsRight.x = xrdata;//cv::Point(xrdata,yblabel);
    Config::instance()->ldwsRight.y = yblabel;
    Config::instance()->ldwsTop.x = xtdata;//cv::Point(xtdata,xtlabel);
    Config::instance()->ldwsTop.y = xtlabel;
    if(leftcount>1&&rightcount==0){//左车道
            leftbuttom = 0.0;
            lefttop = 0.0;
            for (size_t i = 0; i < leftcount; i++) {
                leftbuttom += (yblabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1;
                lefttop += (ytlabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1;
            }
            leftbuttom /= leftcount;
            lefttop /= leftcount;
		for (size_t i = 0; i < leftcount; i++) {
            if(fabs((yblabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1-xldata)>15){
                lines2.erase(lines2.begin()+i);
            }

        }
		if(lines2.size()<leftcount&&lines2.size()!=0){
            leftbuttom = 0.0;
            lefttop = 0.0;
            for (size_t i = 0; i < lines2.size(); i++) {
                leftbuttom += (yblabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1;
                lefttop += (ytlabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1;
            }
            leftbuttom /= lines2.size();
            lefttop /= lines2.size();
        }
            if(lines2.size()!=0){
                if(fabs(leftbuttom-xldata)<200){
                    if((xldata-leftbuttom)>(xrdata-xldata)/3.5*0.5){
                        isRightShift = true;
                        HHSQ_ADAS_MESSAGE_S message = {0};
                        message.type = HHSQ_ADAS_TYPE_LDW;
                        message.command = HHSQ_COMMAND_LDWS_ALARM;
                        message.content[0] = HHSQ_LDWS_SHIFT_RIGHT;
                        Notify::notify(message);
                    }else if((xldata-leftbuttom)<-(xrdata-xldata)/3.5*0.5){
                        isLeftShift = true;
                        HHSQ_ADAS_MESSAGE_S message = {0};
                        message.type = HHSQ_ADAS_TYPE_LDW;
                        message.command = HHSQ_COMMAND_LDWS_ALARM;
                        message.content[0] = HHSQ_LDWS_SHIFT_LEFT;
                        Notify::notify(message);
                    }
                }
                lines3.push_back({(int)(leftbuttom-index1),(int)(yblabel-index5),(int)(lefttop-index1),(int)(ytlabel-index5)});
                lines3.push_back({(int)(lefttop + xldata3 - xldata2-index1),(int)(ytlabel-index5),(int)(leftbuttom + xrdata - xldata-index1),(int)(yblabel-index5)});
            }
        } else if(leftcount==0&&rightcount>1){//右车道
         rightbuttom = 0.0;
		 righttop = 0.0;
            for (size_t i = 0; i < rightcount; i++) {
                rightbuttom += (yblabel - (lines2[i][1] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][0] + index1+LROI.cols-index4;
                righttop += (ytlabel - (lines2[i][1] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][0] + index1+LROI.cols-index4;
            }
            rightbuttom /= rightcount;
            righttop /= rightcount;
			for (size_t i = 0; i < rightcount; i++) {
                if(fabs((yblabel - (lines2[i][1] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][0] + index1+LROI.cols-index4-xrdata)>100){
                    lines2.erase(lines2.begin()+i);
                }
            }
			if(lines2.size()<rightcount&&lines2.size()!=0){
            rightbuttom = 0.0;
            righttop = 0.0;
            for (size_t i = 0; i < lines2.size(); i++) {
                rightbuttom += (yblabel - (lines2[i][1] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][0] + index1+LROI.cols-index4;
                righttop += (ytlabel - (lines2[i][1] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][0] + index1+LROI.cols-index4;
            }
            rightbuttom /= lines2.size();
            righttop /= lines2.size();



        }
	if(lines2.size()!=0){
        if(fabs(rightbuttom-xrdata)<200){
            if((rightbuttom-xrdata)>(xrdata-xldata)/3.5*0.5){
				isLeftShift = true;
                HHSQ_ADAS_MESSAGE_S message = {0};
                message.type = HHSQ_ADAS_TYPE_LDW;
                message.command = HHSQ_COMMAND_LDWS_ALARM;
                message.content[0] = HHSQ_LDWS_SHIFT_LEFT;
                Notify::notify(message);
            }else if((rightbuttom-xrdata)<-(xrdata-xldata)/3.5*0.5){
				isRightShift = true;
                HHSQ_ADAS_MESSAGE_S message = {0};
                message.type = HHSQ_ADAS_TYPE_LDW;
                message.command = HHSQ_COMMAND_LDWS_ALARM;
                message.content[0] = HHSQ_LDWS_SHIFT_RIGHT;
                Notify::notify(message);
            }
        }
            lines3.push_back({(int) (rightbuttom + xldata - xrdata - index1), (int) (yblabel - index5), (int) (righttop + xldata2 - xldata3 - index1), (int) (ytlabel - index5)});
            lines3.push_back({(int) (righttop - index1), (int) (ytlabel - index5), (int) (rightbuttom - index1), (int) (yblabel - index5)});
	}
    } else if(leftcount>1&&rightcount>1){//左右车道
        if(leftcount-3*rightcount>=0){
                lines2.erase(lines2.begin()+leftcount-1,lines2.end());
                leftbuttom = 0.0;
                lefttop = 0.0;
                rightbuttom = 0.0;
                righttop = 0.0;
                for (size_t i = 0; i < lines2.size(); i++) {
                    leftbuttom += (yblabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1;
                    lefttop += (ytlabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1;
                }
                leftbuttom /= lines2.size();
                lefttop /= lines2.size();
                rightbuttom = leftbuttom + xrdata - xldata;
                righttop =lefttop + xldata3 - xldata2;
            } else if(rightcount-3*leftcount>=0){
                lines2.erase(lines2.begin(),lines2.begin()+leftcount);
                leftbuttom = 0.0;
                lefttop = 0.0;
                rightbuttom = 0.0;
                righttop = 0.0;
                for (size_t i = 0; i < lines2.size(); i++) {
                    rightbuttom += (yblabel - (lines2[i][1] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][0] + index1+LROI.cols-index4;
                    righttop += (ytlabel - (lines2[i][1] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][0] + index1+LROI.cols-index4;
                }
                rightbuttom /= lines2.size();
                righttop /= lines2.size();
                leftbuttom = rightbuttom + xldata - xrdata;
                lefttop = righttop + xldata2 - xldata3;
            } else{
                leftbuttom = 0.0;
                lefttop = 0.0;
                rightbuttom = 0.0;
                righttop = 0.0;            
				for (size_t i = 0; i < leftcount; i++) {
                leftbuttom += (yblabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1;
                lefttop += (ytlabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1;
            }
            leftbuttom /= leftcount;
            lefttop /= leftcount;
            for (size_t i = 0; i < rightcount; i++) {
                rightbuttom += (yblabel - (lines2[leftcount + i][1] + index5)) / (lines2[leftcount + i][3] - lines2[leftcount + i][1]) * (lines2[leftcount + i][2] - lines2[leftcount + i][0]) + lines2[leftcount + i][0] + index1+LROI.cols-20;
                righttop += (ytlabel - (lines2[leftcount + i][1] + index5)) / (lines2[leftcount + i][3] - lines2[leftcount + i][1]) * (lines2[leftcount + i][2] - lines2[leftcount + i][0]) + lines2[leftcount + i][0] + index1+LROI.cols-20;
            }
            rightbuttom /= rightcount;
            righttop /= rightcount;
			if(fabs(lefttop-righttop)<80){
                    if(fabs(leftbuttom-xldata)>2*fabs(rightbuttom-xrdata)){
                        leftbuttom=rightbuttom + xldata - xrdata;
                        lefttop=righttop + xldata2 - xldata3;
                    } else{
                        rightbuttom=leftbuttom + xrdata - xldata;
                        righttop=lefttop + xldata3 - xldata2;
                    }
                }
                if(fabs(leftbuttom-xldata)>300||fabs(rightbuttom-xrdata)>300){
                    lines2.clear();
                }
            }
			}   
			if(lines2.size()!=0){
            if(fabs(rightbuttom-xrdata)>70&&fabs(rightbuttom-xrdata)-fabs(leftbuttom-xldata)>0){
                bool less50= false;
                bool more50= false;
                for (size_t i = 0; i < leftcount; i++) {
                    if(fabs((yblabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1-xldata)>50)more50= true;
                    if(fabs((yblabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1-xldata)<50)less50= true;
                }
                if(less50&&more50){
                    leftbuttom = 0.0;
                    lefttop = 0.0;
                    for (size_t i = 0; i < leftcount; i++) {

                        if(fabs((yblabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1-xldata)>50){
                            lines2.erase(lines2.begin()+i);
                            leftcount--;
                        }
                    }
                    for (size_t i = 0; i < lines2.size()-rightcount; i++) {
                        leftbuttom += (yblabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1;
                        lefttop += (ytlabel - (lines2[i][3] + index5)) / (lines2[i][3] - lines2[i][1]) * (lines2[i][2] - lines2[i][0]) + lines2[i][2] + index1          ;
                    }
                    leftbuttom /= lines2.size()-rightcount;
                    lefttop /= lines2.size()-rightcount;
                }

                rightbuttom=leftbuttom + xrdata - xldata;
                righttop=lefttop + xldata3 - xldata2;
            }
            if(fabs(leftbuttom-xldata)>70&&fabs(leftbuttom-xldata)-fabs(rightbuttom-xrdata)>0){
                leftbuttom=rightbuttom + xldata - xrdata;
                lefttop=righttop + xldata2 - xldata3;
            }
        }  
		if(lines2.size()!=0) {       
			if(xldata-leftbuttom>(xrdata-xldata)/3.5*0.5){
				isRightShift = true;
                HHSQ_ADAS_MESSAGE_S message = {0};
                message.type = HHSQ_ADAS_TYPE_LDW;
                message.command = HHSQ_COMMAND_LDWS_ALARM;
                message.content[0] = HHSQ_LDWS_SHIFT_RIGHT;
                Notify::notify(message);
            } else if(xrdata-rightbuttom<-(xrdata-xldata)/3.5*0.5){
				isLeftShift = true;
                HHSQ_ADAS_MESSAGE_S message = {0};
                message.type = HHSQ_ADAS_TYPE_LDW;
                message.command = HHSQ_COMMAND_LDWS_ALARM;
                message.content[0] = HHSQ_LDWS_SHIFT_LEFT;
                Notify::notify(message);
            }
            lines3.push_back(cv::Vec4i(leftbuttom-mOriImage.cols/3,yblabel-mOriImage.rows/5*4,lefttop-mOriImage.cols/3,ytlabel-mOriImage.rows/5*4));
			lines3.push_back(cv::Vec4i(righttop-mOriImage.cols/3,ytlabel-mOriImage.rows/5*4,rightbuttom-mOriImage.cols/3,yblabel-mOriImage.rows/5*4));
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
 * 根据方向分类车道线
 */
void LDWS::sortLinesByAngle()
{
    int imageWidth = ROI.cols;
    double slope = 0.0;
    for( size_t i = 0; i < lines3.size(); i++ )
    {
        cv::Vec4i l = lines3[i];
        if(l[2] - l[0] != 0)
        {
            slope = (float(l[3]) - l[1])/(l[2] - l[0]);
            //Storing lanes by slope
            if(slope > 0.5)
            {
                //Storing lanes by space coordinates
                if(l[2] > (imageWidth - (2 * imageWidth / 3)))
                    rightLines.push_back(cv::Vec4i(l[0],l[1],l[2],l[3]));
                else
                    leftLines.push_back(cv::Vec4i(l[0],l[1],l[2],l[3]));
            }
                //Storing left lanes
            else if(slope < 0 )
            {
                //Storing lanes by space coordinates
                if(l[0] < (imageWidth - (imageWidth / 3)))
                    leftLines.push_back(cv::Vec4i(l[0],l[1],l[2],l[3]));
                else
                    rightLines.push_back(cv::Vec4i(l[0],l[1],l[2],l[3]));
            }
        }
    }
}
/**
 * 找最合适的车道线
 */
void LDWS::getSingleLaneSegment()
{
    double lineLength = 0.0;
    double previousLineLength = 0.0;
    if(rightLines.size()!=0) {
        for (size_t i = 0; i < rightLines.size(); i++) {
            cv::Vec4i l = rightLines[i];
            lineLength = sqrt(pow(float(l[3] - l[1]), 2) + pow(l[2] - l[0], 2));
            if (lineLength > previousLineLength) {
                rightLine = cv::Vec4i(l[0], l[1], l[2], l[3]);
                previousLineLength = lineLength;
            }
        }
        sourceImageRectPoint.RT.x = rightLines[0][0] + index1;
        sourceImageRectPoint.RT.y = rightLines[0][1] + index5;
        sourceImageRectPoint.RB.x = rightLines[0][2] + index1;
        sourceImageRectPoint.RB.y = rightLines[0][3] + index5;
    }
    //Left Lane
    previousLineLength = 0.0;
    if(leftLines.size()!=0) {
        for (size_t i = 0; i < leftLines.size(); i++) {
            cv::Vec4i l = leftLines[i];
            lineLength = sqrt(pow(float(l[3] - l[1]), 2) + pow(l[2] - l[0], 2));
            if (lineLength > previousLineLength) {
                leftLine = cv::Vec4i(l[0], l[1], l[2], l[3]);
                previousLineLength = lineLength;
            }
        }
        sourceImageRectPoint.LB.x = leftLines[0][0] + index1;
        sourceImageRectPoint.LB.y = leftLines[0][1] + index5;
        sourceImageRectPoint.LT.x = leftLines[0][2] + index1;
        sourceImageRectPoint.LT.y = leftLines[0][3] + index5;
    }
}

/**
 * 通过已有线段计算扩展
 */
void LDWS::getLineFromSegment()
{
    double slope = 0.0;
    double b = 0.0;
    //Right Lane
    if(rightLine.first.x - rightLine.second.x != 0)
    {
        slope = (float(rightLine.first.y - rightLine.second.y)) / (rightLine.first.x - rightLine.second.x);
        b = rightLine.first.y - (slope * rightLine.first.x);
        rightLine.first.x  = - b / slope;
        rightLine.first.y  = 0;
        rightLine.second.x = (mWrapImage.rows - b) / slope;
        rightLine.second.y = mWrapImage.rows;
        //rightLaneDetected  = !rightLine.empty();
    }
    //Left Lane
    if(leftLine.first.x - leftLine.second.x != 0)
    {
        slope = (float(leftLine.first.y - leftLine.second.y)) / (leftLine.first.x - leftLine.second.x);
        b = leftLine.first.y - (slope * leftLine.first.x);
        leftLine.first.x  = - b / slope;;
        leftLine.first.y  = 0;
        leftLine.second.x = (mWrapImage.rows - b) / slope;
        leftLine.second.y = mWrapImage.rows;
        //leftLaneDetected = !leftLine.empty();
    }
}
/**
 * 图像透视变换
 */
void LDWS::reverseIPM(){
    vector<cv::Vec2f> linePointsIPMImage;
    //Right Lane
    if(rightLines.size() != 0)
    {
        linePointsIPMImage.push_back(cv::Vec2f(float(rightLine.first.x), float(rightLine.first.y)));
        linePointsIPMImage.push_back(cv::Vec2f(float(rightLine.second.x), float(rightLine.second.y)));
    }
    //Left Lane
    if(leftLines.size() != 0)
    {
        linePointsIPMImage.push_back(cv::Vec2f(float(leftLine.first.x), float(leftLine.first.y)));
        linePointsIPMImage.push_back(cv::Vec2f(float(leftLine.second.x), float(leftLine.second.y)));
    }
    if (linePointsIPMImage.size() == 0){
        return;
    }
    vector<cv::Vec2f> linePointsSourceImage;
    cv::perspectiveTransform(linePointsIPMImage, linePointsSourceImage, mWrapInverseMat);
    //如果只有一条线，则放到右边的空间
    if (linePointsSourceImage.size() >= 4) {
        sourceImageRectPoint.LB.x = linePointsSourceImage[3][0];
        sourceImageRectPoint.LB.y = linePointsSourceImage[3][1];
        sourceImageRectPoint.LT.x = linePointsSourceImage[2][0];
        sourceImageRectPoint.LT.y = linePointsSourceImage[2][1];
    }
    sourceImageRectPoint.RT.x = linePointsSourceImage[0][0];
    sourceImageRectPoint.RT.y = linePointsSourceImage[0][1];
    sourceImageRectPoint.RB.x = linePointsSourceImage[1][0];
    sourceImageRectPoint.RB.y = linePointsSourceImage[1][1];
}
/**
 * 获取边框的坐标值，如果偏移不大，原框保持
 * @param preDraw
 * @param dstDraw
 */
void LDWS::getDrawFilterBox(PointRect &preDraw, PointRect &dstDraw) {
    preDraw.LB.y =  dstDraw.LB.y;
    preDraw.LT.y =  dstDraw.LT.y;
    preDraw.RT.y =  dstDraw.RT.y;
    preDraw.RB.y =  dstDraw.RB.y;
    if (abs(preDraw.LB.x -  dstDraw.LB.x) > 5) {
        if (preDraw.LB.x < dstDraw.LB.x) {
            if (preDraw.LB.x + 5 < dstDraw.LB.x) {
                preDraw.LB.x = preDraw.LB.x + 5;
                preDraw.RB.x = preDraw.RB.x + 5;
            }
            else {
                preDraw.LB.x = dstDraw.LB.x;
                preDraw.RB.x = dstDraw.RB.x;
            }
        } else {
            if (preDraw.LB.x - 5 < dstDraw.LB.x) {
                preDraw.LB.x = preDraw.LB.x - 5;
                preDraw.RB.x = preDraw.RB.x - 5;
            }
            else {
                preDraw.LB.x = dstDraw.LB.x;
                preDraw.RB.x = dstDraw.RB.x;
            }
        }
    }
    if (abs(preDraw.LT.x -  dstDraw.LT.x) > 2) {
        if (preDraw.LT.x < dstDraw.LT.x) {
            if (preDraw.LT.x + 2 < dstDraw.LT.x) {
                preDraw.LT.x = preDraw.LT.x + 2;
                preDraw.RT.x = preDraw.RT.x + 2;
            }
            else{
                preDraw.LT.x =  dstDraw.LT.x;
                preDraw.RT.x =  dstDraw.RT.x;
            }
        } else {
            if (preDraw.LT.x - 2 < dstDraw.LT.x) {
                preDraw.LT.x = preDraw.LT.x - 2;
                preDraw.RT.x = preDraw.RT.x - 2;
            }
            else {
                preDraw.LT.x =  dstDraw.LT.x;
                preDraw.RT.x =  dstDraw.RT.x;
            }
        }
    }
}
/**
 * 处理边框的顔色过渡
 * @param drawPoints
 * @param image
 */
void LDWS::drawFilterBox(PointRect &drawPoints, cv::Mat& image) {
    float stepX = (drawPoints.LT.x -  drawPoints.LB.x) / 11.5;
    float stepY = (drawPoints.LB.y -  drawPoints.LT.y) / 11.5;
    for (int i = 0; i < 1; ++i) {
        cv::Scalar scalar;
        if (i == 0)      scalar = cv::Scalar(0x00,0x00,0xff);
        matFillPolly(image,
                     cv::Point(drawPoints.LB.x + (i+0) * stepX,drawPoints.LB.y -  (i+0) * stepY),
                     cv::Point(drawPoints.LB.x + (i+5.2) * stepX,drawPoints.LB.y -  (i+5.2) * stepY),
                     cv::Point(drawPoints.RB.x + (i+5.2) * stepX,drawPoints.RB.y -  (i+5.2) * stepY),
                     cv::Point(drawPoints.RB.x + (i+0) * stepX,drawPoints.RB.y -  (i+0) * stepY),
                     scalar);
    }
    for (int i = 6; i < 7; ++i) {
        cv::Scalar scalar;
        if (i == 6)      scalar = cv::Scalar(0,255,255);
        matFillPolly(image,
                     cv::Point(drawPoints.LB.x + (i+0) * stepX,drawPoints.LB.y -  (i+0) * stepY),
                     cv::Point(drawPoints.LB.x + (i+2.3) * stepX,drawPoints.LB.y -  (i+2.3) * stepY),
                     cv::Point(drawPoints.RB.x + (i+2.3) * stepX,drawPoints.RB.y -  (i+2.3) * stepY),
                     cv::Point(drawPoints.RB.x + (i+0) * stepX,drawPoints.RB.y -  (i+0) * stepY),
                     scalar);
    }
    for (int i = 9; i < 10; ++i) {
        cv::Scalar scalar;
        if (i == 9)      scalar = cv::Scalar(0x00,0xff,0x00);
        float ratio = stepX > 0 ? 3.0 : 4.0;
        matFillPolly(image,
                     cv::Point(drawPoints.LB.x + (i+0) * stepX,drawPoints.LB.y -  (i+0) * stepY),
                     cv::Point(drawPoints.LB.x + (i+2.5) * stepX,drawPoints.LB.y -  (i+2.5) * stepY),
                     cv::Point(drawPoints.RB.x + (i+2.5) * stepX,drawPoints.RB.y -  (i+2.5) * stepY),
                     cv::Point(drawPoints.RB.x + (i+0) * stepX,drawPoints.RB.y -  (i+0) * stepY),
                     scalar);
    }
}
/**
 * 覆盖已检测到的轨道线
 * @param mat
 */
void LDWS::drawResult(cv::Mat &mat){
    if(Config::instance()->isLdwCalibration){
        //cv::line(mat, cv::Point(599.023,501.52),cv::Point(223.345,720), cv::Scalar(92, 187, 251),1,CV_AA);
        //cv::line(mat, cv::Point(599.023,501.52),cv::Point(926.5,720), cv::Scalar(92, 187, 251),1,CV_AA);
        cv::line(mat, cv::Point(xtdata,xtlabel),cv::Point(xldata,yblabel), cv::Scalar(92, 187, 251),1,CV_AA);
        cv::line(mat, cv::Point(xtdata,xtlabel),cv::Point(xrdata,yblabel), cv::Scalar(92, 187, 251),1,CV_AA);
        /*
        ytlabel=480.0;//topy的值
        yblabel=720.0;//buttomy的值
        xldata=223.0;//左边x的横坐标
        xrdata=999.0;//右边x的横坐标
        xtdata=619.982;//三角形顶点的横坐标
        xtlabel=455.944;//三角形顶点的纵坐标
         */
    }
    do {
        UtilMutexAutoLock lock(mutex);
		cv::Rect rect(mat.cols/3,mat.rows/5*4, mat.cols/3, mat.rows-mat.rows/5*4);
        for(Vec4i i:lines2){
            if(i[0]+index1<(xrdata-xldata)/2+xldata)cv::line(mat, cv::Point(i[0] + index1, i[1] + index5), cv::Point(i[2] + index1, i[3] + index5), cv::Scalar(0, 0, 255), 1, CV_AA);
        	else cv::line(mat, cv::Point(i[0] + index1+mat(rect)(Rect(0,0,mat(rect).cols/3,mat(rect).rows)).cols-index4, i[1] + index5), cv::Point(i[2] + index1+mat(rect)(Rect(0,0,mat(rect).cols/3,mat(rect).rows)).cols-index4, i[3] + index5), cv::Scalar(0, 0, 255), 1, CV_AA);
        }
        lines2.clear();
        if (laneFillPoly.empty()) {
            return;
        }
        laneFillPoly = Util::RGB2YUV(laneFillPoly);
        cv::addWeighted(mat(laneFillPolyROI), 1, laneFillPoly(laneFillPolyROI), 0.25, 0, mat(laneFillPolyROI));
    }while(0);
}

/**
 * 判断检测到的轨道线是否是对的
 * @return
 */
bool LDWS::isDrawLane(){
    float bomLen = sourceImageRectPoint.RB.x - sourceImageRectPoint.LB.x;
    float topLen = sourceImageRectPoint.RT.x - sourceImageRectPoint.LT.x;
    bool bomOk = (bomLen > mOriImage.cols/3 ) && (bomLen < 4.0*mOriImage.cols/3);
    bool topOk = (topLen > mOriImage.cols/10) && (topLen < 2.0*mOriImage.cols/3);
    //cout << "found:" << bomOk << " " << topOk << " bomLen:"<< bomLen << " topLen:"<< topLen << endl;
    return bomOk && topOk;
}
/**
 * 画轨道线的矩形框，主要是动画处理
 */
void LDWS::drawLaneBox(){
    static PointRect drawLeft;
    static PointRect drawRight;
    if (drawLeft.LB.x == 0 || drawLeft.LB.y == 0 || drawRight.LB.x == 0 || drawRight.LB.y == 0
        ||  abs(s_pointsLeft.LB.x - drawLeft.LB.x)>mOriImage.cols/8 || abs(drawRight.LB.x - s_pointsRight.LB.x)>mOriImage.cols/8){
        drawLeft = s_pointsLeft;
        drawRight = s_pointsRight;
    } else {
        getDrawFilterBox(drawLeft, s_pointsLeft);
        getDrawFilterBox(drawRight, s_pointsRight);
    }
    Mat laneFillPoly2 =cv::Mat(mOriImage.rows, mOriImage.cols, CV_8UC3, cv::Scalar(0));
    /*
    if(Config::instance()->isLdwCalibration){
        cv::line(laneFillPoly2, cv::Point(619.982,455.944),cv::Point(223,720), cv::Scalar(92, 187, 251),1,CV_AA);
        cv::line(laneFillPoly2, cv::Point(619.982,455.944),cv::Point(999,720), cv::Scalar(92, 187, 251),1,CV_AA);
    }
     */
    matFillPolly(laneFillPoly2,drawLeft.RB,drawLeft.RT,drawRight.LT,drawRight.LB,cv::Scalar(255));
    drawFilterBox(drawLeft,laneFillPoly2);
    drawFilterBox(drawRight,laneFillPoly2);
    Adapter::instance()->gaussianFilter(laneFillPoly2,laneFillPoly2,3);
    //int xOffset = (drawLeft.LT.x - drawLeft.LB.x)/3;
    //cv::Mat roi = laneFillPoly2(cv::Rect(drawLeft.LT.x-xOffset,drawLeft.LT.y-10,drawRight.RT.x - drawLeft.LT.x+xOffset*2,(mOriImage.rows - drawLeft.LT.y)/4));
    do{
        UtilMutexAutoLock lock(mutex);
        laneFillPolyROI.x = drawLeft.LB.x;
        laneFillPolyROI.y = drawLeft.RT.y;
        laneFillPolyROI.width = drawRight.RB.x - laneFillPolyROI.x;
        laneFillPolyROI.height = drawLeft.LB.y - laneFillPolyROI.y;
        laneFillPoly=laneFillPoly2;
    }while (0);
}
/**
 * 轨道线整合
 */
void LDWS::drawFilter(){
    int offsetBot = 10;
    int offsetTop = 10;
    if (s_pointsLeft.LB.x == 0 && s_pointsLeft.LB.y == 0) {
        s_pointsLeft.LB = cv::Point(sourceImageRectPoint.LB.x - 12, sourceImageRectPoint.LB.y);
        s_pointsLeft.LT = cv::Point(sourceImageRectPoint.LT.x - 8, sourceImageRectPoint.LT.y);
        s_pointsLeft.RT = cv::Point(sourceImageRectPoint.LT.x + 8, sourceImageRectPoint.LT.y);
        s_pointsLeft.RB = cv::Point(sourceImageRectPoint.LB.x + 12, sourceImageRectPoint.LB.y);
    }
    if (s_pointsRight.LB.x == 0 && s_pointsRight.LB.y == 0) {
        s_pointsRight.LB = cv::Point(sourceImageRectPoint.RB.x - 12, sourceImageRectPoint.RB.y);
        s_pointsRight.LT = cv::Point(sourceImageRectPoint.RT.x - 8, sourceImageRectPoint.RT.y);
        s_pointsRight.RT = cv::Point(sourceImageRectPoint.RT.x + 8, sourceImageRectPoint.RT.y);
        s_pointsRight.RB = cv::Point(sourceImageRectPoint.RB.x + 12, sourceImageRectPoint.RB.y);
    }
    //头部变化比较大
    if (
        (abs(sourceImageRectPoint.LT.x - s_pointsLeft.LT.x) > offsetTop && abs(sourceImageRectPoint.LT.x - s_pointsLeft.RT.x) > offsetTop)
        &&
        (abs(sourceImageRectPoint.RT.x - s_pointsRight.LT.x) > offsetTop && abs(sourceImageRectPoint.RT.x - s_pointsRight.RT.x) > offsetTop)
        &&
        (
            //但是变化不能太大，太大说明有跳变
            //abs(abs(sourceImageRectPoint.LT.x - s_pointsLeft.LT.x)  -  abs(sourceImageRectPoint.RT.x - s_pointsRight.LT.x)) <=   40
			abs(sourceImageRectPoint.RT.x - s_pointsRight.LT.x)>=   20
        )
        ) {
        s_pointsLeft.LT = cv::Point(sourceImageRectPoint.LT.x - 8, sourceImageRectPoint.LT.y);
        s_pointsLeft.RT = cv::Point(sourceImageRectPoint.LT.x + 8, sourceImageRectPoint.LT.y);
        s_pointsRight.LT = cv::Point(sourceImageRectPoint.RT.x - 8, sourceImageRectPoint.RT.y);
        s_pointsRight.RT = cv::Point(sourceImageRectPoint.RT.x + 8, sourceImageRectPoint.RT.y);
    }
    //尾部变化比较大
    if (
        (abs(sourceImageRectPoint.LB.x - s_pointsLeft.LB.x) > offsetBot && abs(sourceImageRectPoint.LB.x - s_pointsLeft.RB.x) > offsetBot)
        &&
        (abs(sourceImageRectPoint.RB.x - s_pointsRight.RB.x) > offsetBot && abs(sourceImageRectPoint.RB.x - s_pointsRight.LB.x) > offsetBot)
        &&
        (
              //但是变化不能太大，太大说明有跳变
              //abs(abs(sourceImageRectPoint.LB.x - s_pointsLeft.LB.x)  -  abs(sourceImageRectPoint.RB.x - s_pointsRight.LB.x)) <=  100
			abs(sourceImageRectPoint.RB.x - s_pointsRight.LB.x)>=  50
        )
        ) {
        s_pointsLeft.LB = cv::Point(sourceImageRectPoint.LB.x - 12, sourceImageRectPoint.LB.y);
        s_pointsLeft.RB = cv::Point(sourceImageRectPoint.LB.x + 12, sourceImageRectPoint.LB.y);
        s_pointsRight.LB = cv::Point(sourceImageRectPoint.RB.x - 12, sourceImageRectPoint.RB.y);
        s_pointsRight.RB = cv::Point(sourceImageRectPoint.RB.x + 12, sourceImageRectPoint.RB.y);
    }
    drawLaneBox();
}
/**
 * X方向轨道偏移阈值
 * @return
 */
int  LDWS::getLaneShiftThresholdX(){
    return this->ROI.cols/8;
}
/**
 * 检测车道偏移
 */
void LDWS::checkLaneShift(){
    static int rightShift = 0;
    static int leftShift = 0;
    if (s_pointsRight.LB.x == 0 || s_pointsLeft.LB.x == 0) {
        rightShift = 0;
        leftShift = 0;
        return;
    }
    int threshold = getLaneShiftThresholdX();
    int checkLeft = 0;
    int checkRight = 0;
    //有两条
//    if (sourceImageRectPoint.LB.x > 0 && sourceImageRectPoint.RB.x > 0) {
//        checkLeft = Util::abs(s_pointsLeft.RB.x,sourceImageRectPoint.LB.x);
//        checkRight = Util::abs(s_pointsRight.LB.x,sourceImageRectPoint.RB.x);
//    } else { //只有一条
//        int left = Util::abs(s_pointsLeft.RB.x,sourceImageRectPoint.RB.x);
//        int right = Util::abs(s_pointsRight.LB.x,sourceImageRectPoint.RB.x);
//
//        if (left > right) {//近右线
//            checkRight = right;
//        } else { //近左线
//            checkLeft = left;
//        }
//    }
    checkLeft=s_pointsLeft.RB.x;
    checkRight=926-s_pointsRight.LB.x;
    if (checkLeft>400) {
        leftShift++;
        rightShift = 0;
    } else if (checkRight>100) {
        rightShift++;
        leftShift = 0;
    } else {
        rightShift = 0;
        leftShift = 0;
    }
    if (rightShift > CHECKED_LANE_SHIFT_TIME) {
        isRightShift = true;
        HHSQ_ADAS_MESSAGE_S message = {0};
        message.type = HHSQ_ADAS_TYPE_LDW;
        message.command = HHSQ_COMMAND_LDWS_ALARM;
        message.content[0] = HHSQ_LDWS_SHIFT_RIGHT;
        Notify::notify(message);
    } else if (leftShift > CHECKED_LANE_SHIFT_TIME) {
        isLeftShift = true;
        HHSQ_ADAS_MESSAGE_S message = {0};
        message.type = HHSQ_ADAS_TYPE_LDW;
        message.command = HHSQ_COMMAND_LDWS_ALARM;
        message.content[0] = HHSQ_LDWS_SHIFT_LEFT;
        Notify::notify(message);
    }
}
/**
 * 处理检测结果
 */
void LDWS::drawFinalLines() {
    static int drawCount = -1;
    if (drawCount > 0) drawCount--;
    if(sourceImageRectPoint.validPoint() ==  2) { //只有一条线，检测有没有偏移
//        checkLaneShift();
    } else if(sourceImageRectPoint.validPoint() == 4) {
        this->drawFilter();
//        checkLaneShift();
        drawCount = CHECKED_LANE_HOLD_TIME;
    }
    if (drawCount <= 0){
        UtilMutexAutoLock lock(mutex);
        s_pointsRight.zero();
        s_pointsLeft.zero();
        laneFillPoly.release();
    }
}
/**
 * 处理流程
 */
void LDWS::process() {
    UtilTimeDiff diff("LDWS::process");
    if (this->mOriImage.empty()) {
        cout << "LDWS:empty image" << endl;
        return;
    }
    getWrapMat();
    sortLinesByAngle();
    getSingleLaneSegment();
    drawFinalLines();
}

#endif