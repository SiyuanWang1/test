#ifndef IVS_LINEDETECTTOOL_H
#define IVS_LINEDETECTTOOL_H
#include "ToolBase.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<math.h>
using namespace cv;
using namespace std;
/*
【边缘数量】序列化ROI中子ROI的数量就是检出边缘点的数量，即在每个子ROI中检出一个边缘点；
【区域尺寸】表示序列化ROI中每个子ROI的长度和宽度；
【颜色变化】边缘点颜色变化参数。边缘的扫描方向由序列化ROI确定，用户可以在图像上调整。子ROI中显示了每个子矩形ROI中边缘扫描方向；
【选点方式】扫描线上选择满足颜色变化需求并且强度大于阈值的峰值点或者选择第一个点；
【边缘阈值】，图像上检出边缘点时的阈值，参数范围为0到255。默认值20。【检出阈值】也可以通过【直方图显示窗口】中调整蓝色竖直线来调节。
【点数阈值】找到的边缘点数量少于此阈值时直接报错。
【扫描线宽度】边缘扫描线的宽度，等于子矩形ROI的宽度；
【滤波宽度】差分滤波器的半长度。
【深度阈值】此参数仅用户选择寻找第一个满足上述要求的边缘点时有效。大于设定阈值的边缘点与下一个大于同样阈值的反色边缘点的距离定义为此边缘点的深度。深度小于深度阈值的边缘点被认为是噪声，此时算法将跳过继续寻找满足要求的边缘点。
【长度范围】找到的边缘点拟合得到直线段之后，此直线段长度范围。最大值为0时表示不限制。
【角度差异】找到的边缘点拟合得到直线段之后，此直线段与【边定位】ROI的角度差异最大值，参数范围为0-180度，默认值为10度。*/
/*
Mat  src                      ：输入图像 
int selectPointCount    :  选点数量
int blurWidth              ：扫描线宽度
int roiHeight                :子ROI高度
int edgeThreshold      ： 边缘阈值
int selectionMethod   ： 选点方式，0为峰值点，1为最先找到点
int colorChange         ： 颜色变换，0为黑到白，1为白到黑
int blurLength            ： 差分滤波器长度
int depthThreshold     :   深度阈值
int pointnumThreshold:  点数阈值
int minlength         :    最小长度
int maxlength         :    最大长度
int angleDiffer            :   角度差异
scanArea  detectArea   :   扫描区域
scanArea  templateArea  :   模板区域
float step                        
*/
Mat image;
typedef struct Line{
	Point2f p1;
	Point2f p2;
	Point2f center;
}Line;
typedef struct scanArea{
	Point2f p1;
	Point2f p2;
}scanArea;
typedef struct NNode{
	Point2f point;//位置
	float value;//求平均值后的结果
	float result=0;//滤波后的结果
}NNode;
class DisperseEdgeDetectTool :public ToolBase
{
private:
	std::vector<int> relyNums={};
	scanArea detectArea ;      //扫描区域 
	Mat image;                //存储输入图片
	int selectPointCount;    //选点数量
	int blurWidth;           //滤波宽度
	int roiHeight;            //子ROI高度
	int edgeThreshold ;      //边缘阈值
	int selectionMethod;     //选点方式，0为峰值点，1为最先找到点
	int colorChange;         //颜色变换，0为黑到白，1为白到黑
	int blurLength;          //差分滤波器长度
	int depthThreshold;      //深度阈值
	int pointnumThreshold;     // 点数阈值
	int minLength;            //  最小长度
	int maxLength;            //   最大长度
	int angleDiffer;           //  角度差异
	float step;              //点距
	float center_angle;//模板旋转角度
	scanArea templateArea;//模板区域
	Point2f center;//矩形旋转中心坐标：模板特征定位区域中心点
	Point2f tempDatum_point;//矩形计算相对位移基准点坐标：模板特征定位区域中心点
	Point2f testDatum_point;//矩形计算相对位移基准点坐标：待测图定位的中心点
	Line line;          //检测出的边缘
	

	void workPoints(Mat src,scanArea detectArea, int selectPointCount, int blurWidth,int roiHeight,vector<vector<Point2f>>& results);//将所有扫描线存入vector<vector<Point2f>>& results。
	int pointsDetection(Mat src,scanArea area,vector<vector<Point2f>> scanPoints, int edgeThreshold, int selectionMethod, int colorChange, int blurLenth,int blurWidth,int depthThreshold,vector<Point2f> &points );//每个子ROI区域检测返回一个点
	int  edgeDetect(Mat src,scanArea area, int selectPointCount, int blurWidth,int blurLength,int roiHeight,int edgeThreshold,int depthThreshold,int pointnumThreshold,int selectionMethod,int colorChange,int minLength,int maxLength,int angleDiffer,Line &line);//对所有边缘点进行边拟合，并根据条件判断边
	int move_templateArea(Point2f center, float center_angle, Point2f tempDatum_point,Point2f testDatum_point,scanArea &detectArea);//移动检测区域
	
public:

	DisperseEdgeDetectTool(){};
	virtual~DisperseEdgeDetectTool() {};
        virtual int getRelyOnParam(std::vector<int> &nums);
        virtual int initTool(Json::Value json);
        virtual void updateTool(Json::Value json){};
	virtual int run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result);
        virtual int run(cv::Mat src,cv::Mat &dst ){};
        virtual int emptyTool(){};

};
#endif 
