//
// Created by wangxiaobao on 18-8-31.
//

#include "../include/Points.h"


#pragma region (点集合)

Points::Points(int i) {
    cv::Point point(0,0);
    for (int j = 0; j < i; ++j){
        vPoints.push_back(point);
    }
}

/**
 * 点赋空
 */
void Points::zero() {
    cv::Point point(0,0);
    for (int i= 0; i < vPoints.size(); ++i) {
        vPoints[i] = point;
    }
}

/**
 * 是否所有的点为空
 * @return
 */
bool Points::empty() {
    for (int i = 0; i < vPoints.size(); ++i) {
        if (vPoints.at(i).x != 0 || vPoints.at(i).y != 0) {
            return false;
        }
    }

    return true;
}

bool Points::equal(const Points& input){
    if (vPoints.size() != input.vPoints.size()){
        return false;
    }

    for (int i = 0; i < vPoints.size(); ++i){
        if (vPoints[i].x != input.vPoints[i].x){
            return false;
        }

        if (vPoints[i].y != input.vPoints[i].y){
            return false;
        }
    }

    return true;
}

string Points::toString() {
    string str;
    for (int i =0; i < vPoints.size(); ++i) {
        char s[100] = {0};
        sprintf(s,"point[%d]-> x:%d y:%d\n",i,vPoints.at(i).x,vPoints.at(i).y);
        str += string(s);
    }

    return str;
}


#pragma endregion

#pragma region (线坐标)
PointLine::PointLine() : Points(2),
                         first(vPoints[0]),
                         second(vPoints[1]),
                         top(vPoints[0]),
                         bottom(vPoints[1]),
                         left(vPoints[0]),
                         right(vPoints[1])
                         {
    first.x = 0;
    first.y = 0;
    second.x = 0;
    second.y = 0;
}
PointLine::PointLine(int x1,int y1,int x2,int y2) : Points(2),
                         first(vPoints[0]),
                         second(vPoints[1]),
                         top(vPoints[0]),
                         bottom(vPoints[1]),
                         left(vPoints[0]),
                         right(vPoints[1])
{
    first.x = x1;
    first.y = y1;
    second.x = x2;
    second.y = y2;
}

PointLine::PointLine(cv::Point point1,cv::Point point2) : Points(2),
                                                    first(vPoints[0]),
                                                    second(vPoints[1]),
                                                    top(vPoints[0]),
                                                    bottom(vPoints[1]),
                                                    left(vPoints[0]),
                                                    right(vPoints[1])
{
    first = point1;second = point2;
}

PointLine& PointLine::operator=(cv::Vec4i points){
    first.x = points[0];
    first.y = points[1];
    second.x = points[2];
    second.y = points[3];

    return *this;
}

PointLine& PointLine::operator=(const PointLine& points){
    first.x = points.first.x;
    first.y = points.first.y;
    second.x = points.second.x;
    second.y = points.second.y;

    return *this;
}

#pragma endregion

#pragma region (矩形坐标)
PointRect::PointRect() : Points(4),
                         LB(vPoints[0]),
                         LT(vPoints[1]),
                         RT(vPoints[2]),
                         RB(vPoints[3]) {
}

PointRect& PointRect::operator=(const PointRect& pointRect){
    vPoints = pointRect.vPoints;
    return *this;
}

/**
 * 矩形的有效点个数
 * @return
 */
int PointRect::validPoint(){
    int count = 0;
    for (int i = 0; i < vPoints.size(); ++i) {
        if (vPoints[i].x != 0 || vPoints[i].y != 0) {
            count ++;
        }
    }

    return count;
}
#pragma endregion