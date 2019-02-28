//
//  mtypes.h
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/19.
//

#ifndef __MTYPES_H_H__
#define __MTYPES_H_H__
#include "CvHeader.h"
#include <vector>
#include <iostream>
#include <iomanip>
namespace Monocular {
    
    /******** macro define *********/
    #define SAVEPATH           "../../result"   //输出路径
    
    #define DESTROYOBJECT(obj)    if(NULL != obj){obj->release();obj = NULL;}
    #define CHECKVALUE(v)         ((v).x > 0)
    
    #define TESTOUTPUT          1 //结果
    #define NEEDWRITEFILE       1 //允许输出到文件
    #define STEREO_TEST         0 //双目数据测试
    
    #define NEEDPRINTDEBUGFINO _DEBUG        //是否打印输出信息
    
    #define DEBUGINFOBG          std::cout
    #define DEBUGINFOED          std::endl
    #define DEBUGPRECI(l)        std::setprecision(l)
    
    #define PRINTGEO(g)          (g).y << "," << (g).x
    #define PRINTGEOSTR(label,g) DEBUGINFOBG << DEBUGPRECI(20) << (label) << PRINTGEO(g) << DEBUGINFOED;
    #define PRINTLABEL(label,v)  DEBUGINFOBG << DEBUGPRECI(20) << (label) << (v) << DEBUGINFOED;
    
    #define WRONGVALUE       -1         //错误值
    #define MAXLON           180.0      //经度最大值
    
    #define MINOBJECTSIZE    50.0f      //目标可检测到物体的最小包围盒大小
    
    #define EPILINESEARCHLEN 300.0f     //极线搜索长度
    #define EPILINESEARCHSP  0.5f       //极线搜索步长
    #define EPILINESEARCHSC  0.75f      //匹配判定阀值
    

    
    /******** enum define *********/
    //x:lon y:lat
    typedef cv::Point2d GeoPos;
    
    typedef std::vector<Point2f> PtVector;
    typedef std::vector<float>   FloatVector;
    
    typedef std::vector<KeyPoint>  KeyPointVector;
    typedef std::vector<DMatch>    MatchVector;
    
    /******** struct define *********/
    //目标
    struct TargetItem      // 目标物
    {
        int            _id;         //类型
        Point2f        _center;     //像素坐标
        Rect2f         _box;        //包围盒
        GeoPos         _pos;        //绝对世界坐标
#if TESTOUTPUT
        GeoPos         _realpos;    //真实坐标 仅测试有用
        float          a,b,c;       //极线a,b,c
#endif
        TargetItem(){}
        TargetItem(int id, const Point2f &center, const Rect2f &box):
        _id(id),_center(center),_box(box),_pos(MAXLON+1)
        {
#if TESTOUTPUT
            _realpos = GeoPos(-1,-1);    //真实坐标 仅测试有用
            a = b = c = 0.0;       //极线a,b,c
#endif
        }
        
        inline bool isValid()const
        {
            return (_pos.x < MAXLON) && (_pos.x > -MAXLON);
        }
    };
    //目标集合
    typedef std::vector<TargetItem> TargetItems;
    
    //相机参数
    struct Camera
    {
        Point2d principal_point;        //光心坐标
        
        double  focal_length;           //焦距
        
        Mat K;                          //相机内参
        //add more..
    };

    /******** enum define *********/
    
    //帧类型枚举
    enum eFrameType
    {
        eFeatureFrame,
        eOpticalFlowFrame
    };
    
    //特征点提取类型枚举
    enum eFeatureDetectorType
    {
        eORB_TYPE,
        eFAST_TYPE
        //add more..
    };
    
    //匹配类型枚举
    enum eMatcherType
    {
        eHammingDistanceMatcher,
        eOpticalFlowMatcher
    };
}


#endif /* __MTYPES_H_H__ */
