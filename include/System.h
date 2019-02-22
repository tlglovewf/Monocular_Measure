//
//  System.h
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/18.
//
#ifndef __SYSTEM_H_H__
#define __SYSTEM_H_H__
#include "mtypes.h"

namespace Monocular {
    class Tracking;
    class PoseEstimation;
    //跟踪模式
    enum eTrackingMode
    {
        eFeaturesMode,
        OpticalFlowMode
    };
    
    //系统控制类
    class  System 
    {
    public:
        /*
         * 构造函数
         */
        System(const Camera &cam, eTrackingMode mode);
        /*
         * 析构函数
         */
        ~System();
        
        /*
         * 目标检测
         */
        TargetItems objectDetect(const Mat &img);
        
        /*
         * 特征点追踪
         * @param 图像
         */
        void handle(const Mat &img,const GeoPos &geopt,const TargetItems &items);
        
        /*
         * 状态重置
         */
        void reset();
    protected:
        Tracking            *mpTracker;
        PoseEstimation      *mpEstimation;
    };
}

#endif /*__SYSTEM_H_H__*/

