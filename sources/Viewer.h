//
//  Viewer.h 可视化
//  Monocular_Measure
//
//  Created by TuLigen on 2019/3/12.
//

#ifndef __VIEWER_H_H_
#define __VIEWER_H_H_

#include "BaseObject.h"
#include "CvHeader.h"

namespace Monocular {
    
    class Viewer : public BaseObject
    {
    public:
        
        /*
         * 绘制
         */
        virtual void render() = 0;
        
        /*
         * 设置当前位姿
         */
        virtual void setCurrentPose(const Mat &img,const Mat &R, const Mat &t,double scale) = 0;
        
        /*
         * 设置起始位姿
         */
        virtual void setStartingPose(const Mat &img,const Mat &R,const Mat &t) = 0;
        
        /*
         * 设置当前目标位置
         */
        virtual void setTargetPose(const Point3d &pt) = 0;
        
        virtual bool isInit()const = 0;
    };
    
    // 默认绘制对象
    class DefaultViewer : public Viewer
    {
    public:
        
        /*
         * 构造
         */
        DefaultViewer();
        /*
         * 析构
         */
        ~DefaultViewer();
        
        /*
         * 绘制
         */
        virtual void render() ;
        
        /*
         * 设置当前位姿
         */
        virtual void setCurrentPose(const Mat &img,const Mat &R, const Mat &t,double scale) ;
        
        /*
         * 设置起始位姿
         */
        virtual void setStartingPose(const Mat &img,const Mat &R,const Mat &t)
        {
            mIndicatorR = R.clone();
            mIndicatorT = t.clone();
            mCurImg = img;
        }
        
        /*
         * 设置当前目标位置
         */
        virtual void setTargetPose(const Point3d &pt);
        
        virtual bool isInit()const
        {
            return !mIndicatorR.empty();
        }
    protected:
        Mat     mCanvas;
        Mat     mIndicatorR;
        Mat     mIndicatorT;
        Mat     mCurImg;
    };
}


#endif /* __VIEWER_H_H_ */
