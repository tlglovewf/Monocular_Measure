//
//  FeatureDetector.h
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/18.
//
#ifndef __FEATURE_DETECTOR_H_H__
#define __FEATURE_DETECTOR_H_H__
#include "mtypes.h"
#include "BaseObject.h"
namespace Monocular {
    //特征提取类
    class FeatureDetector : public BaseObject
    {
    public:

        /*
         * 特征关键点提取
         * @param img       图像
         * @param keypoints 特征点集
         */
        virtual void detect(const Mat &img, KeyPointVector &keypoints) = 0;
        
        /*
         * 特征描述子计算
         * @param img     图像
         * @keypoints     关键点帧
         * @descriptor    特征描述子
         */
        virtual void compute(const Mat &img, KeyPointVector &keypoints, Mat &descriptor) = 0;
        
        
        /*
         * 创建特征提取对象
         */
        static FeatureDetector* CreateDetector(eFeatureDetectorType type);
    };
    
    //orb 特征提取类
    class ORBFeatureDetector : public FeatureDetector
    {
    public:
        ORBFeatureDetector();
        ~ORBFeatureDetector();
        /*
         * 特征关键点提取
         * @param img       图像
         * @param keypoints 特征点集
         */
        virtual void detect(const Mat &img, KeyPointVector &keypoints) ;
        
        /*
         * 特征描述子计算
         * @param img     图像
         * @keypoints     关键点帧
         * @descriptor    特征描述子
         */
        virtual void compute(const Mat &img, KeyPointVector &keypoints, Mat &descriptor) ;
    protected:
        Ptr<ORB> mDetector;
    };
    
    //角点检测
    class FASTFeatureDetector : public FeatureDetector
    {
    public:
        /*
         * 特征关键点提取
         * @param img       图像
         * @param keypoints 特征点集
         */
        virtual void detect(const Mat &img, KeyPointVector &keypoints)
        {
            int fast_threshold = 20;
            bool nonmaxSuppression = true;
            FAST(img, keypoints, fast_threshold, nonmaxSuppression);
        }
        
        /*
         * 特征描述子计算
         * @param img     图像
         * @keypoints     关键点帧
         * @descriptor    特征描述子
         */
        virtual void compute(const Mat &img,KeyPointVector &keypoints, Mat &descriptor) {return;}
    };
    
    //add more .. for ex: surf sift
}

#endif /*__FEATURE_DETECTOR_H_H__*/

