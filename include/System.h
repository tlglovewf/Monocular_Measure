//
//  System.h
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/18.
//
#ifndef __SYSTEM_H_H__
#define __SYSTEM_H_H__
#include "mtypes.h"
#include <list>
#include <string>
#include "Serialization.h"

namespace Monocular {
    class Tracking;
    class PoseEstimation;
    class Frame;
    class Viewer;
    //跟踪模式
    enum eTrackingMode
    {
        eFeaturesMode,
        eOpticalFlowMode
    };
    
    //系统控制类
    class  System 
    {
    public:
        /*
         * 构造函数
         */
        System(const Camera &cam, eTrackingMode mode,const std::string &outPath);
        
        System(const Camera &cam, eTrackingMode mode,Viewer *pViewer);
        
        /*
         * 析构函数
         */
        ~System();
        
        /*
         * 目标检测
         */
        TargetItems objectDetect(const Mat &img,int sample);
        
        /*
         *
         */
        TargetItems stereoObjDetect(const Mat &img);
        
        /*
         * 特征点追踪
         * @param img   图像
         * @param geopt 坐标
         * @param items 目标列表
         */
        void handle(const Mat &img,const GeoPos &geopt,const TargetItems &items);
        
        void handle(const Mat &img, const GeoPos &geopt);
        
        /*
         * 跟踪轨迹
         *
         */
        void track(const Mat &img, const GeoPos &geopt);
        
        /*
         * 状态重置
         */
        void reset();
        
        /*
         * 优化
         */
        void optimize()
        {
            // add more..
        }
        
        /*
         * 输出结果
         */
        void printResult();
        
    protected:
        /*
         * 加入到帧列表中
         */
        void addList(Frame *frame);
        
        /*
         * 重置列表
         */
        void resetList();
    protected:
        typedef std::list<Frame*>        FrameList;
        typedef FrameList::iterator      FrameListIter;
        
        Tracking                        *mpTracker;
        PoseEstimation                  *mpEstimation;

        //序列化对象
        Serialization                   *mpSerialization;
        Viewer                          *mpViewer;
        FrameList                       mFrameList;
    };
}

#endif /*__SYSTEM_H_H__*/

