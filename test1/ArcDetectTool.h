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

    // 判断滤波时的深度阈值    
    //输入：index,当前检测到的边缘点在扫描线上的索引。
    //blur_result，一条扫描线的滤波结果
    bool edg_isok(int index,vector<int> blur_result);

    //根据点拟合圆 
    // 输入：pts，进行拟合的点集   
    bool fittingCircle(std::vector<cv::Point>&pts);    

    //根据模板特征定位和模板圆弧检测的中心和检测图像的特征定位，计算检测图像圆弧检测的中心
    int Rotate_center();    

    //计算每一个旋转的矩形框  
    //输入：center 检测图像进行圆弧检测的检测中心 
    //center_angle 每个检测框和初始角度对应的检测框的角度差  
    //temp_area 初始角度的矩形框的四个顶点坐标  
    //sample_area 初始角度对应的矩形框旋转center_angle后的矩形框
    int Rotate_rect(cv::Point center, float center_angle, area temp_area, area &sample_area);   

    //圆弧检测   
    //输入： src 检测图像
    int arcDetect(Mat src);    

    //扫描线检测 输入：src检测图像 scanPoints 扫描线对应的点集
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
