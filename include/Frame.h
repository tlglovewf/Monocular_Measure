//
//  Frame.h
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/18.
//
#ifndef __FRAME_H_H__
#define __FRAME_H_H__
#include "BaseObject.h"
#include "mtypes.h"

namespace Monocular {
    
    class FeatureDetector;
    
    //帧对象
    class Frame : public BaseObject
    {
    public:
        Frame(const Mat &img,const GeoPos &pt):mPos(pt),mImg(img)
        {}
        
        
        /*
         * 设置帧目标集合
         */
        void setTargetItems( const TargetItems &items);
        
        /*
         * 获取目标集合
         */
        TargetItems& getTargetItems() {return mTargets;}
        
        
        /*
         * 设置经纬度坐标
         */
        void setPosition(const GeoPos &geopt)
        {
            mPos = geopt;
        }
        
        /*
         * 获取经纬度坐标
         */
        GeoPos getPosition()const
        {
            return mPos;
        }
        
        /*
         * 获取特征点集
         */
        virtual KeyPointVector getKeyPoints()const
        {
            return mKeyPoints;
        }
        
        /*
         * 获取关键点像素坐标
         */
        Point2f getKeyPtCoord(int index)
        {
            assert( index < mKeyPoints.size() );
            return mKeyPoints[index].pt;
        }
        
        /*
         * 获取描述子
         */
        virtual Mat getDescriptor()const = 0;
        
        /*
         * 获取图像
         */
        Mat getImg()const
        {
            return mImg;
        }
        
        /*
         * 设置世界变换矩阵
         */
        void setWordTransform(const Mat &tcw)
        {
            mTcw = std::move(tcw);
        }
        
        
        /*
         * 打印信息
         */
        virtual void print() ;
        
        /*
         * @param type 帧类型
         * @return     帧对象
         */
        static Frame* CreateFrame(eFrameType type,const Mat &img,const GeoPos &pt, FeatureDetector *pFeature = 0);
    protected:
        GeoPos          mPos;//坐标(经纬度）
        Mat             mImg;
        Mat             mTcw;//变换矩阵
        KeyPointVector  mKeyPoints;
        TargetItems     mTargets;
    };
    
    //特征点帧对象
    class FeatureFrame : public Frame
    {
    public:
        FeatureFrame(const Mat &img,const GeoPos &pt,FeatureDetector *pDetector);
        ~FeatureFrame();
        
        /*
         * 获取描述子
         */
        virtual Mat getDescriptor()const {return mDescriptor;}
        
    protected:
        Mat            mDescriptor;
    };
    
    //光流帧对象
    class OpticalFlowFrame : public Frame
    {
    public:
        OpticalFlowFrame(const Mat &img,const GeoPos &pt,FeatureDetector *pDetector);
        
        /*
         * 获取描述子
         */
		virtual Mat getDescriptor()const { return Mat(); }
    };
}
#endif /*__FRAME_H_H__*/

