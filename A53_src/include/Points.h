//
// Created by wangxiaobao on 18-8-31.
//

#ifndef ADASPROJECT_POINTS_H
#define ADASPROJECT_POINTS_H

#include "define.h"

class Points {
protected:
    Points(int i);
public:
    vector<cv::Point> vPoints;
    void zero();
    bool empty();
    bool equal(const Points& input);
    string toString();
};

class PointLine :public Points{
public:
    PointLine();
    PointLine(int x1,int y1,int x2,int y2);
    PointLine(cv::Point point1,cv::Point point2);

    PointLine& operator=(cv::Vec4i points);
    PointLine& operator=(const PointLine&);
    cv::Point& first;
    cv::Point& second;
    cv::Point& top;
    cv::Point& bottom;
    cv::Point& left;
    cv::Point& right;
};

class PointRect : public Points{
public:
    PointRect();
    cv::Point& LB; //左下
    cv::Point& LT; //左上
    cv::Point& RT; //右上
    cv::Point& RB; //右下

    PointRect& operator=(const PointRect&);
    int validPoint();
    void zeroLeft()  {LB.x=0; LB.y=0; LT.x=0; LT.y=0;}
    void zeroRight() {RB.x=0; RB.y=0; RT.x=0; RT.y=0;}
    void zeroTop()   {LT.x=0; LT.y=0; RT.x=0; RT.y=0;}
    void zeroBottom(){LB.x=0; LB.y=0; RB.x=0; RB.y=0;}
};

#endif //ADASPROJECT_POINTS_H
