#ifndef ARCDETECTTOOL_H
#define ARCDETECTTOOL_H
#include "ToolBase.h"
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

typedef struct Line{
    Point p1;
    Point p2;
}Line;
struct area{
    Point p1,p2,p3,p4;
};
class ArcDetectTool:public ToolBase
{
private:
    vector<int>relyNums={};
    Point2f center;    //模板特征定位位置
    float center_angle;    //特征定位后测算出的角度
    Point2f tempDatum_point;    //模板图像的特征定位位置
    Point2f testDatum_point;    //检测图像的特征定位位置
    
    int edge_depth;    //深度阈值，若再扫描线上找到的下一个反色点到当前找到的点的距离小于阈值，则不选取当前点。
    int selectPointCount;    //选点数量，扫描线的数量
    int blurWidth;    //滤波宽度
    int edgeThreshold;    //边缘阈值
    int selectionMethod;    //选点方式
    int colorChange;    //颜色变化
    int blurLength;     //滤波器的半长度
    Point2f detect_Center;    //模板中进行圆弧检测的中心
    int inner_radius;    //模板圆弧检测的内半径
    int external_radius;    //模板圆弧检测的外半径
    float start_angle;    //模板圆弧检测的初始角度
    float end_angle;    //模板圆弧检测的终止角度
    Point2f test_center;    //计算得到的检测图像的圆弧检测中心
    float start_an;    //计算得到的检测图像的圆弧检测初始角度
    float end_an;    //计算得到的检测图像的圆弧检测终止角度
    float _radius;    //检测到的半径
    Point2f _center;    //检测到的圆心
    float angleLimit;    //角度阈值
    bool edg_isok(int,vector<int>);     
    bool fittingCircle(std::vector<cv::Point>&pts);
    int Rotate_center();
    int Rotate_rect(cv::Point center, float center_angle, area temp_area, area &sample_area);
    int arcDetect(Mat src);
    int EdgeSpotDetection(Mat src,vector<vector<LineIterator>> scanPoints);
public:
	ArcDetectTool(){};
	virtual~ArcDetectTool() {};
        virtual int getRelyOnParam(std::vector<int> &nums);
        virtual int initTool(Json::Value json);
        virtual void updateTool(Json::Value json){};
	virtual int run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result);
        virtual int run(cv::Mat src,cv::Mat &dst ){};
        virtual int emptyTool(){};
};

#endif // ARCDETECTTOOL_H
