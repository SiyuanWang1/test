#ifndef IVS_LINEDETECTTOOL_H
#define IVS_LINEDETECTTOOL_H
#include "ToolBase.h"
#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;
//【颜色变化】沿着ROI设定的扫描方向进行扫描，寻找从【黑到白】或者【白到黑】的边缘点，二选一。
//【选点方式】沿着ROI设定的扫描方向进行扫描，寻找【峰值点】或者【最先找到点】作为边缘点。【峰值点】指扫描方向上边缘最强且超过【检出阈值】的点，【最先找到点】指扫描方向上最先找到的边缘强度超过【检出阈值】的点。
//【选点数量】沿着ROI设定的扫描方向等间隔设置扫描线进行扫描，扫描线的数量。数量为0时表示自动选择，一般为逐像素设置扫描线；数量大于0时表示实际的扫描线数量。
//【边缘阈值】，图像上检出边缘点时的阈值，参数范围为0到255。默认值20。【检出阈值】也可以通过【直方图显示窗口】中调整蓝色竖直线来调节。
//【线宽度】为了抑制噪声，在扫描方向上扫面边缘点时，可以选择一定的线宽，综合扫描方向上多个像素的信息寻找边缘，边缘位置会更稳定一点。参数范围为非负整数，默认值为3。
//【滤波宽度】差分滤波器的半长度。
//【深度阈值】此参数仅用户选择寻找第一个满足上述要求的边缘点时有效。大于设定阈值的边缘点与下一个大于同样阈值的反色边缘点的距离定义为此边缘点的深度。深度小于深度阈值的边缘点被认为是噪声，此时算法将跳过继续寻找满足要求的边缘点。
//【找圆得分】根据边缘点落在圆上点的比例可计算得到圆的得分，用来赛选某些虚假的圆；
//【使用忽略掩膜过滤边缘点】选中后可使用忽略掩膜过滤部分边缘点；
/*
Mat  src                      ：输入图像
scanArea detectArea       :检测区域 
scanArea templateArea       :模板区域 
int selectPointCount    :  选点数量
int blurWidth              ：滤波宽度
int edgeThreshold      ： 边缘阈值
int selectionMethod   ： 选点方式，0为峰值点，1为最先找到点
int colorChange         ： 颜色变换，0为黑到白，1为白到黑
int blurLength            ： 差分滤波器长度
int depthThreshold     :   深度阈值
float score              :   找圆得分
float step               :   点距
*/
typedef struct scanArea{
	Point center;//检测区域圆心坐标	
	int exRadius;//外半径长度
	int inRadius;//内半径长度
}scanArea;
typedef struct node{
	Point point;//位置
	float value=0;//求平均值后的结果
	float result=0;//滤波后的结果
}node;  
   
typedef struct Circle{
	Point2f center;
	float radius;
}Circle;
class CircleDetectTool :public ToolBase
{
private:
	std::vector<int> relyNums={};
	scanArea detectArea ;      //扫描区域 
	Mat image;                //存储输入图片
	int selectPointCount;    //选点数量
	int blurWidth;           //滤波宽度
	int edgeThreshold ;      //边缘阈值
	int selectionMethod;     //选点方式，0为峰值点，1为最先找到点
	int colorChange;         //颜色变换，0为黑到白，1为白到黑
	int blurLength;          //差分滤波器长度
	int depthThreshold;      //深度阈值
	float score;             //找圆得分
	float step;              //点距
	float center_angle;//模板旋转角度
	scanArea templateArea;//模板区域
	Point2f center;//矩形旋转中心坐标：模板特征定位区域中心点
	//float center_angle;//模板旋转角度
	Point2f tempDatum_point;//矩形计算相对位移基准点坐标：模板特征定位区域中心点
	Point2f testDatum_point;//矩形计算相对位移基准点坐标：待测图定位的中心点
	Circle circle;          //检测出的圆形
	//int  Rotate_rect(cv::Point2f center, double center_angle, cv::Point2f tempDatum_point, cv::Point2f testDatum_point,scanArea &sample_area);
	

	void workPoints(Mat src,scanArea area, int selectPointCount, int blurWidth,vector<vector<Point>>&results);//将所有扫描线所在的点选出放入vector<vector<Point>>&results
	int pointsDetection(Mat src,scanArea area,vector<vector<Point>> scanPoints, int edgeThreshold, int selectionMethod, int colorChange, int blurLenth,int blurWidth,int depthThreshold,vector<Point2f> &points );//根据判断条件每条扫描线返回一个边缘点
	int circleDetect(Mat src,scanArea area, int selectPointCount, int blurWidth,int blurLength,int edgeThreshold, int selectionMethod,int colorChange,int depthThreshold,Circle &circle);//检测圆
	int move_circle(Point2f center, float center_angle, Point2f tempDatum_point,Point2f testDatum_point,scanArea &detectArea);//根据特征定位的位置变化移动圆
	bool fittingCircle(std::vector<cv::Point2f>& pts, cv::Point2f& center, float& radius);//最小二乘法拟合圆
public:

	CircleDetectTool(){};
	virtual~CircleDetectTool() {};
        virtual int getRelyOnParam(std::vector<int> &nums);
        virtual int initTool(Json::Value json);
        virtual void updateTool(Json::Value json){};
	virtual int run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result);
        virtual int run(cv::Mat src,cv::Mat &dst ){};
        virtual int emptyTool(){};

};
#endif 
