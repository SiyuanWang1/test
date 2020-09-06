#include "ArcDetectTool.h"
#include "ivs_algorithm_utils.h"
#include <string>
ToolBase *CreateTool(void)
{
    return (new ArcDetectTool());
}

int ArcDetectTool::initTool(Json::Value json)
{
        if(json["toolRelyNums"].isNull())
        {
           std::cout<<"Error:ArcDetectTool tool init error:NULL!"<<std::endl;
           //throw std::exception();
           return 0;
        }
        std::cout<<"ArcDetect tool init start"<<std::endl;
	//初始化参数
        unsigned int rely_size =  json["toolRelyNums"].size();
        for (unsigned int i = 0; i < rely_size; ++i)
        {
              int num=json["toolRelyNums"][i].asInt();
              relyNums.push_back(num);
        }
        //初始化参数
        center.x = json["centerX"].asDouble();
               center.y = json["centerY"].asDouble();
               tempDatum_point.x = json["templatePointX"].asDouble();
               tempDatum_point.y = json["templatePointY"].asDouble();
               detect_Center.x = json["detect_centerX"].asInt();
               detect_Center.y = json["detect_centerY"].asInt();
               inner_radius = json["inner_radius"].asInt();
               external_radius = json["ext_radius"].asInt();
               colorChange = json["colorChange"].asInt();
               selectPointCount = json["selectPointCount"].asInt();
               selectionMethod = json["selectMethod"].asInt();
               edgeThreshold = json["edgeThreshold"].asInt();
               blurWidth = json["blurWidth"].asInt();
               blurLength = json["blurLength"].asInt();
               start_angle= json["start_angle"].asDouble();
               end_angle = json["end_angle"].asDouble();
	       edge_depth= json["edge_Depth"].asInt();
	       angleLimit= json["angleLimit"].asDouble();
        std::cout<<"ArcDetect tool init down********************"<<std::endl;
        return 1;
}
int ArcDetectTool::getRelyOnParam(std::vector<int> &nums) 
{
        //通过依赖工具参数获取当前工具的结果编号nums
        if(relyNums.size()==0)
          return 0;
        nums=relyNums;
        return 1;
}
int ArcDetectTool::run(cv::Mat src,std::vector<Json::Value> relyOnResults, Json::Value &result)
{
        TimeTracker time;
        time.start();
        int arcflag = 0;
        int toolIsOk = 0;
        try
	{
            if(src.channels() != 1)
                cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);
	    for (int i = 0; i < relyOnResults.size(); i++)
	    {
                //std::cout<<"CenterX:"<<relyOnResults[i]["CenterX"].asInt()<<std::endl;
                //std::cout<<"CenterY:"<<relyOnResults[i]["CenterY"].asInt()<<std::endl;
		//从依赖工具的结果中读参数
                testDatum_point.x = relyOnResults[i]["CenterX"].asDouble();
                testDatum_point.y = relyOnResults[i]["CenterY"].asDouble();
		center_angle=relyOnResults[i]["Angle"].asInt();
	    }
            //std::cout<<"detectRect:"<<detectRect.x<<":"<<detectRect.y<<std::endl;
	    //计算检测图像的圆弧检测中心
	    Rotate_center();
	    //cout<<"********************"<<endl;
            //检测圆弧
	    arcflag=arcDetect(src); 
            if(arcflag)
                toolIsOk = 1;
	}
        catch (std::exception e)
        {
            std::cout<<e.what()<<" :LineDetect fail!"<< std::endl;   //捕获异常，然后程序结束
            return 0;
        }
        time.stop();
        int times=time.duration();
        
	//将匹配结果以Json形式存储
        std::string resultStr="{\"ToolIsOk\":"+std::to_string(toolIsOk)+",\"_center.x\":"+std::to_string(_center.x)+",\"_center.y\":"+std::to_string(_center.y)+",\"_radius\":"+std::to_string(_radius)+",\"RunTimes\":"+std::to_string(times)+"}";
        std::cout<<resultStr.c_str()<<std::endl; 
        Json::Reader reader;
        reader.parse(resultStr, result);
        return 1;
}
//进行深度阈值判断
//index为当前找到的边缘点再扫描线上的索引
//blur_result为整个扫描线的滤波结果
bool ArcDetectTool:: edg_isok(int index,vector<int> blur_result)
{
    if(index+edge_depth<blur_result.size())
    {
        for(int i=0;i<=edge_depth;i++)
        {
            //color_change==1,表示为找白到黑的边缘，且此边缘点滤波结果为负，找下一个反色边缘点，即滤波结果为正的，若在深度阈值范围内找到反色边缘
            //说明此边缘点的后对应的黑色区域的宽度不满足条件，此点不是满足条件的边缘点
            if(colorChange==1)
                if(blur_result[index+i]>0)
                    return false;
            if(colorChange==0)
                if(blur_result[index+i]<0)
                    return false;
        }
    }
    return true;
}
//特征检测到特征中心位置后，根据模板中的特征中心和圆弧检测中心计算检测图像的圆弧检测中心和初始，终止角度。
int  ArcDetectTool::Rotate_center()
{
    cv::Point p,p1;
      
            if(center_angle==0){
            p.x=detect_Center.x;
            p.y=detect_Center.y;
            //return 0;
        }
            p.x=detect_Center.x;
            p.y=detect_Center.y;
            p1.x=detect_Center.x;
            p1.y=detect_Center.y;

        //旋转后点坐标
        if(center_angle!=0){
        if (center_angle > 180)
        {
            center_angle = 360 - center_angle;
            double angle = center_angle * CV_PI / 180; // 弧度
            p.x = (p1.x - center.x)*cos(angle) - (p1.y - center.y)*sin(angle) + center.x;
            p.y = (p1.x - center.x)*sin(angle) + (p1.y - center.y)*cos(angle) + center.y;
        }
        else
        {
            double angle = center_angle * CV_PI / 180; // 弧度
            p.x = (p1.x - center.x)*cos(angle) - (p1.y - center.y)*sin(angle) + center.x;
            p.y = (p1.y - center.y)*cos(angle) + (p1.x - center.x)*sin(angle) + center.y;
        }
        }
        //计算相对位移
        cv::Point pp1 = cv::Point(p.x - tempDatum_point.x, p.y - tempDatum_point.y);
        
	//新的坐标
        p.x = testDatum_point.x + pp1.x;
        p.y = testDatum_point.y + pp1.y;
	
	//cout<<pp1.x<<" "<<pp1.y<<endl;
	//cout<<testDatum_point.x<<" "<<testDatum_point.y<<endl;
	//cout<<p.x<<" "<<p.y<<endl;
	

        test_center.x=p.x;
        test_center.y=p.y;
        start_an=start_angle;
        end_an=end_angle;
        start_an+=center_angle;
        end_an+=center_angle;
        return 1;
}




/*
旋转检测区域Rotate_rect：以模板图像中center为旋转中心，旋转检测区域，再根据基准点计算相对位移
Point2f center: 用于旋转矩形框的旋转中心
float center_angle： 用于旋转矩形框的旋转角度
Point2f tempDatum_point：模板图用于计算相对位移的基准点
Point testDatum_point：待测图用于计算相对位移的基准点
Rect &detectRect：模板图像中旋转前检测工具区域矩形，以及计算后待测图像中边检测区域（左上角坐标，宽度、高度）
int t_w：待测图像的宽
int t_h：待测图像的高
*/
int ArcDetectTool::Rotate_rect(Point center,float center_angle,area temp_area,area &sample_area)
{
    cv::Point2f h11, h12, h13, h14;
    //旋转前坐标，依次为左上、右上、左下、右下
    h11.x = temp_area.p1.x;
    h11.y = temp_area.p1.y;
    h12.x = temp_area.p2.x;
    h12.y = temp_area.p2.y;
    h13.x = temp_area.p3.x;
    h13.y = temp_area.p3.y;
    h14.x = temp_area.p4.x;
    h14.y = temp_area.p4.y;
    if(center_angle==0)
    {
        sample_area.p1.x=h11.x;
        sample_area.p1.y=h11.y;
        sample_area.p2.x=h12.x;
        sample_area.p2.y=h12.y;
        sample_area.p3.x=h13.x;
        sample_area.p3.y=h13.y;
        sample_area.p4.x=h14.x;
        sample_area.p4.y=h14.y;
        return 1;
    }
    //旋转后点坐标
    if(center_angle!=0)
    {
        cv::Point2f h1, h2, h3, h4;
        if (center_angle > 180)
        {
            center_angle = 360 - center_angle;
            double angle = center_angle * CV_PI / 180; // 弧度
            h1.x = (h11.x - center.x)*cos(angle) - (h11.y - center.y)*sin(angle) + center.x;
            h1.y = (h11.x - center.x)*sin(angle) + (h11.y - center.y)*cos(angle) + center.y;
            h2.x = (h12.x - center.x)*cos(angle) - (h12.y - center.y)*sin(angle) + center.x;
            h2.y = (h12.x - center.x)*sin(angle) + (h12.y - center.y)*cos(angle) + center.y;
            h3.x = (h13.x - center.x)*cos(angle) - (h13.y - center.y)*sin(angle) + center.x;
            h3.y = (h13.x - center.x)*sin(angle) + (h13.y - center.y)*cos(angle) + center.y;
            h4.x = (h14.x - center.x)*cos(angle) - (h14.y - center.y)*sin(angle) + center.x;
            h4.y = (h14.x - center.x)*sin(angle) + (h14.y - center.y)*cos(angle) + center.y;
        }
        else
        {
            double angle = center_angle * CV_PI / 180; // 弧度
            h1.x = (h11.x - center.x)*cos(angle) - (h11.y - center.y)*sin(angle) + center.x;
            h1.y = (h11.y - center.y)*cos(angle) + (h11.x - center.x)*sin(angle) + center.y;
            h2.x = (h12.x - center.x)*cos(angle) - (h12.y - center.y)*sin(angle) + center.x;
            h2.y = (h12.y - center.y)*cos(angle) + (h12.x - center.x)*sin(angle) + center.y;
            h3.x = (h13.x - center.x)*cos(angle) - (h13.y - center.y)*sin(angle) + center.x;
            h3.y = (h13.y - center.y)*cos(angle) + (h13.x - center.x)*sin(angle) + center.y;
            h4.x = (h14.x - center.x)*cos(angle) - (h14.y - center.y)*sin(angle) + center.x;
            h4.y = (h14.y - center.y)*cos(angle) + (h14.x - center.x)*sin(angle) + center.y;
        }
    

        sample_area.p1.x=h1.x;
        sample_area.p1.y=h1.y;
        sample_area.p2.x=h2.x;
        sample_area.p2.y=h2.y;
        sample_area.p3.x=h3.x;
        sample_area.p3.y=h3.y;
        sample_area.p4.x=h4.x;
        sample_area.p4.y=h4.y;
    }
	return 1;
}
//根据检测到的点拟合圆弧
bool ArcDetectTool::fittingCircle(std::vector<cv::Point>& pts)
{
    _center = cv::Point(0, 0);
    _radius = 0.0;
    if (pts.size() < 3) return false;;

    double sumX = 0.0;
    double sumY = 0.0;
    double sumX2 = 0.0;
    double sumY2 = 0.0;
    double sumX3 = 0.0;
    double sumY3 = 0.0;
    double sumXY = 0.0;
    double sumX1Y2 = 0.0;
    double sumX2Y1 = 0.0;
    const double N = (double)pts.size();
    for (int i = 0; i < pts.size(); ++i)
    {
        double x = pts.at(i).x;
        double y = pts.at(i).y;
        double x2 = x * x;
        double y2 = y * y;
        double x3 = x2 * x;
        double y3 = y2 * y;
        double xy = x * y;
        double x1y2 = x * y2;
        double x2y1 = x2 * y;

        sumX += x;
        sumY += y;
        sumX2 += x2;
        sumY2 += y2;
        sumX3 += x3;
        sumY3 += y3;
        sumXY += xy;
        sumX1Y2 += x1y2;
        sumX2Y1 += x2y1;
    }
    double C = N * sumX2 - sumX * sumX;
    double D = N * sumXY - sumX * sumY;
    double E = N * sumX3 + N * sumX1Y2 - (sumX2 + sumY2) * sumX;
    double G = N * sumY2 - sumY * sumY;
    double H = N * sumX2Y1 + N * sumY3 - (sumX2 + sumY2) * sumY;

    double denominator = C * G - D * D;
    if (std::abs(denominator) < DBL_EPSILON) return false;
    double a = (H * D - E * G) / (denominator);
    denominator = D * D - G * C;
    if (std::abs(denominator) < DBL_EPSILON) return false;
    double b = (H * C - E * D) / (denominator);
    double c = -(a * sumX + b * sumY + sumX2 + sumY2) / N;

    _center.x = a / (-2);
    _center.y = b / (-2);
    _radius = std::sqrt(a * a + b * b - 4 * c) / 2;
    return true;
}


int ArcDetectTool::EdgeSpotDetection(Mat src,vector<vector<LineIterator>> scanPoints)
{
    int resultPointLocation = -1;//输出结果
                                 //初始化滤波器
                                 //申请滤波核的存储器
    float* blurKernel = (float*)malloc((blurLength * 2 + 1) * sizeof(float));
    //初始化滤波核
    //int blurKernel[5];
    int index = 0;
    for (; index < blurLength; index++)
    {
        blurKernel[index] = -1;
    }
    blurKernel[index] = 0;
    index++;
    for (; index < blurLength * 2 + 1; index++)
    {
        blurKernel[index] = 1;
    }
    //cout<<"1"<<scanPoints.size()<<endl;
    //差分滤波器滤波
    vector<vector<int>> blurResultsTemp;
    for (int i = 0; i < scanPoints.size(); i++)
    {
        vector<int> blurResultsTempItem;

        //cout<<"1";
        for (int j = blurLength; j < scanPoints[i].size() - blurLength; j++)//滤波窗口
        {
            int blurResult = 0;
            //cout<<"1";
            int index1 = 0;
            for (int k = j - blurLength; k < j + blurLength + 1; k++)
            {
                //cout<<"1";
                //cout<<scanPoints[i][k].pos().y<<"**"<<scanPoints[i][k].pos().x<<endl;
                blurResult += (int)src.at<uchar>(scanPoints[i][k].pos().y,scanPoints[i][k].pos().x) * blurKernel[index1];
                index1++;
                //cout<<"1";
                //cout<<(int)src.at<uchar>(scanPoints[i][k].pos().x,scanPoints[i][k].pos().y)<<
                      //"****"<<blurKernel[index1]<<"***"<<index1<<endl;
            }
            //cout<<blurResult<<endl;
            blurResultsTempItem.push_back(blurResult);
        }
        blurResultsTemp.push_back(blurResultsTempItem);
    }

    int min_count=100000;
    for(int i=0;i<blurResultsTemp.size();i++)
    {
        if(min_count>blurResultsTemp[i].size())
            min_count=blurResultsTemp[i].size();
    }
    //滤波结果
    vector<int> blurResults;
    for (int i = 0; i < min_count; i++)
    {
        int blurResult = 0;
        for (int j = 0; j < blurResultsTemp.size(); j++)
        {
            blurResult += blurResultsTemp[j][i];
        }
        int blurWidth = blurResultsTemp.size();
        //std::cout << blurResult<<","<< blurResultsTemp.size()<<","<< blurResult / blurWidth << std::endl;
        //cout<<blurResult<<endl;
        blurResult = blurResult / blurWidth;
        //cout<<blurResult<<"**"<<endl;
        if (abs(blurResult) > edgeThreshold)
        {
            blurResults.push_back(blurResult);
        }
        else
        {
            blurResults.push_back(0);
        }
    }
    int ed_de=edge_depth;
    //cout<<"******************"<<endl;
    //for(int i=0;i<blurResults.size();i++)
      //  cout<<blurResults[i]<<endl;
    //cout<<blurResults.size()<<endl;
    //for(int i=0;i<blurResults.size();i++)
       // cout<<blurResults[i]<<endl;
    //选点方式，得出边缘点结果
    //若【选点方式】为峰值点，选择边缘对比度大于【边缘阈值】中最显著的。
    if (selectionMethod == 0)
    {
        //首先顺序遍历blurResults
        //颜色变换判断，选择最大值
        if (colorChange == 0)//0为黑到白,差分为正是黑到白的边缘
        {
            int max_value = 0;
            for (int i = 0; i < blurResults.size(); i++)
            {
                if (blurResults[i] > max_value&&edg_isok(i,blurResults))
                {
                    max_value = blurResults[i];
                    resultPointLocation = i + blurLength;
                }
            }
        }
        else if (colorChange == 1)//1为白到黑,差分为负是白到黑边缘
        {
            int min_value = 0;
            for (int i = 0; i < blurResults.size(); i++)
            {
                if (blurResults[i] < min_value&&edg_isok(i,blurResults))
                {
                    min_value = blurResults[i];
                    resultPointLocation = i + blurLength;
                }
            }
        }
    }
    //若【选点方式】为最先找到的点，选择第一个大于【边缘阈值】的点。
    else if (selectionMethod == 1)
    {
        //首先顺序遍历blurResults
        //首先找到第一个与颜色判断相对应的点
        //颜色变换判断，选择最大值
        if (colorChange == 0)//0为黑到白,差分为正是黑到白的边缘
        {
            for (int i = 0; i < blurResults.size(); i++)
            {
                if (blurResults[i]>0&&edg_isok(i,blurResults))
                {

                    resultPointLocation = i + blurLength;
                    break;
                }
            }
        }
        else if (colorChange == 1)//1为白到黑,差分为负是白到黑边缘
        {
            for (int i = 0; i < blurResults.size(); i++)
            {
                if (blurResults[i]<0&&edg_isok(i,blurResults))
                {

                    resultPointLocation = i + blurLength;
                    //cout<<resultPointLocation<<endl;
                    break;
                }
            }
        }
        //接着找与颜色判断相反的点
        //计算它们的距离，判断是否大于边缘阈值
        //迭代以上过程
    }
    return resultPointLocation;
}


/*
lineDetect边检出：通过边缘检测提取边缘点，再经霍夫变换拟合直线段
Mat input:输入图像
Rect ROI:用于检测边的区域

输出：Line，在该区域检测出来的线
*/
//首先将所有的举行扫描框存起来
int ArcDetectTool::arcDetect(Mat src)
{
    vector<Point> edg_points;
    //计算扫描线的数量
    float angle_step;
    if(selectPointCount==0)
        angle_step=0.5;
    else angle_step=(end_angle-start_angle)/(selectPointCount+1);
    vector<area> detect_rect;

    //standard_rect表示与圆环中心角度为0度的长度为圆环长半径-短半径，宽度为滤波宽度的矩形。
    //通过旋转这个矩形来进行有滤波宽度的滤波。
    area standard_rect;
    standard_rect.p1.x=test_center.x+inner_radius;
    standard_rect.p1.y=test_center.y-blurWidth/2;
    standard_rect.p2.x=test_center.x+external_radius;
    standard_rect.p2.y=test_center.y-blurWidth/2;
    standard_rect.p3.x=test_center.x+inner_radius;
    standard_rect.p3.y=test_center.y+blurWidth/2;
    standard_rect.p4.x=test_center.x+external_radius;
    standard_rect.p4.y=test_center.y+blurWidth/2;
    
    //将所有的以扫描线长度为长，扫描线宽度为宽，但角度不同的矩形保存下来
    for(float angle=start_an;angle<end_an;angle+=angle_step)
    {
        area detect_rect_item;
        Rotate_rect(test_center,angle,standard_rect,detect_rect_item);
        detect_rect.push_back(detect_rect_item);
    }
    
    //对每个矩形框进行滤波
    for(int i=0;i<detect_rect.size();i++)
    {
        vector<vector<LineIterator>> scanPoints;
        LineIterator startPoints(src,detect_rect[i].p1,detect_rect[i].p3,4);
        LineIterator endPoints(src,detect_rect[i].p2,detect_rect[i].p4,4);
	
        for(int j=0;j<min(startPoints.count,endPoints.count);j++)
        {
            vector<LineIterator>scanPoint;
            LineIterator iterator(src,startPoints.pos(),endPoints.pos(),4);

            for(int k=0;k<iterator.count;k++,iterator++)
            {
                scanPoint.push_back(iterator);
            }
            scanPoints.push_back(scanPoint);
            startPoints++;
            endPoints++;
        }
	
        int edgePointLocation = EdgeSpotDetection(src,scanPoints);
        
        int connectivity = 4;
        
        //计算滤波矩形中间的那条扫描线位置
        LineIterator iterator(src,Point((detect_rect[i].p1.x+detect_rect[i].p3.x)/2,(detect_rect[i].p1.y+detect_rect[i].p3.y)/2),
                              Point((detect_rect[i].p2.x+detect_rect[i].p4.x)/2,(detect_rect[i].p2.y+detect_rect[i].p4.y)/2),4);
        if(edgePointLocation==-1)
            continue;
        while (--edgePointLocation)
        {
            iterator++;
        }
        edg_points.push_back(iterator.pos());
    }

    //检测到的边缘点数量少于3个，返回0
    if (edg_points.size() > 3)
    {
        //fittingCircle(edg_points, _center, _radius);
        fittingCircle(edg_points);
    }
    else
    {
        return 0;
    }
   
    //计算圆弧的角度，根据检测点集的第一个点和最后一个点计算
    float a,b,c;
    a=sqrt((edg_points[0].x-edg_points[edg_points.size()-1].x)*(edg_points[0].x-edg_points[edg_points.size()-1].x)+(edg_points[0].y-edg_points[edg_points.size()-1].y)*(edg_points[0].y-edg_points[edg_points.size()-1].y));
    b=sqrt((edg_points[0].x-_center.x)*(edg_points[0].x-_center.x)+(edg_points[0].y-_center.y)*(edg_points[0].y-_center.y));
    c=sqrt((edg_points[edg_points.size()-1].x-_center.x)*(edg_points[edg_points.size()-1].x-_center.x)+(edg_points[edg_points.size()-1].y-_center.y)*(edg_points[edg_points.size()-1].y-_center.y));
    float an=acos((b*b+c*c-a*a)/(2*b*c));
    an=an*180/3.14;
    if(fabs(an)<angleLimit)
    return 0;

    return 1;
}


