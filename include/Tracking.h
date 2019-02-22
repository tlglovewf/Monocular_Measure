//
//  Tracking.h
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/18.
//

#ifndef __TRACKING_H_H__
#define __TRACKING_H_H__
#include "mtypes.h"
#include "BaseObject.h"
namespace Monocular {
    
    class Frame;
    class Matcher;
    class FeatureDetector;
    /*
     * 跟踪状态
     */
    enum eTrackingState
    {
        eNo_Image_Yet,//无图像
        eOk
    };
    
    /*
     * 帧标识
     */
    enum eFrameIndex
    {
        ePreFrame,
        eCurFrame
    };
    //追踪类
    class Tracking : public BaseObject
    {
    public:
        /*
         * 构造函数
         * @ftype 帧类型
         * @mtype 追踪类型
         */
        Tracking(eFrameType ftype,eMatcherType mtype = eHammingDistanceMatcher);
        /*
         * 抓取图像
         */
        virtual bool grabImage(const Mat &img,const GeoPos &geopt,const TargetItems &items);
        
        /*
         * 重置
         */
        void reset();
        
        /*
         * 追踪
         */
        bool track();
        
        /*
         * 获取指定帧中匹配对像素点集合
         */
        void getMatchVector( PtVector &prepts, PtVector &curpts )const;
        
        /*
         * 获取指定帧
         */
        Frame* getFrame(eFrameIndex index)
        {
            if(ePreFrame == index)
            {
                return mpPreFrame;
            }
            else
            {
                return mpCurrentFrame;
            }
        }
        
    protected:
        /*
         * 重置状态
         */
        void resetState(){ mState = eNo_Image_Yet;}
        
    protected:
        eTrackingState  mState;
        eFrameType      mFmType;
        Frame           *mpCurrentFrame;    //当前帧
        Frame           *mpPreFrame;        //前一帧
        Matcher         *mpMatcher;
        FeatureDetector *mpFeatureDetector;
    };
}



#endif /* __TRACKING_H_H */

