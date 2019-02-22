#include "FeatureDetector.h"
namespace Monocular {
    
    FeatureDetector* FeatureDetector::CreateDetector(Monocular::eFeatureDetectorType type)
    {
        switch (type) {
            case eORB_TYPE:
                return new ORBFeatureDetector;
            case eFAST_TYPE:
                return new FASTFeatureDetector;
            default:
                assert(NULL);
        }
    }
    
    ORBFeatureDetector::ORBFeatureDetector()
    {
        mDetector = ORB::create(500, 2.0f, 8 ,31, 0, 2, ORB::HARRIS_SCORE,31,20);
    }
    
    ORBFeatureDetector::~ORBFeatureDetector()
    {
        
    }
    
    void ORBFeatureDetector::detect(const cv::Mat &img, KeyPointVector &keypoints)
    {
        assert(NULL != mDetector.get());
        mDetector->detect(img, keypoints);//关键点提取
    }
    
    void ORBFeatureDetector::compute(const cv::Mat &img, KeyPointVector &keypoints, cv::Mat &descriptor)
    {
        assert(NULL != mDetector.get());
        
        mDetector->compute(img, keypoints, descriptor);//描述子计算
    }
}
