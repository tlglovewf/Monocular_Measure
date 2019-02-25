#include "Frame.h"
#include "FeatureDetector.h"
#include <iterator>
#include <opencv2/imgproc.hpp>
#include "Functions.h"
namespace Monocular {
    
    Frame* Frame::CreateFrame(Monocular::eFrameType type,const Mat &img,const GeoPos &pt,FeatureDetector *pFeature/* = NULL*/)
    {
        switch (type) {
            case eFeatureFrame:
                return new FeatureFrame(img,pt,pFeature);
            case eOpticalFlowFrame:
                return new OpticalFlowFrame(img,pt,pFeature);
            default:
                return 0;//error or non-define type
        }
    }
    
    
    void Frame::setTargetItems(const TargetItems &items)
    {
        mTargets = std::move(items);
    }
    
    void Frame::print()
    {
        static int index = 0;
        
        PRINTLABEL("Frame index : ", index);
        
        PRINTGEOSTR("Frame pos : ", mPos);
        
        PRINTLABEL("Target List : ", mTargets.size());
        
        for( auto it : mTargets )
        {
            PRINTLABEL("Type: ", it._id);
            PRINTLABEL("Pixel_x:", it._center.x);
            PRINTLABEL("Pixel_y:", it._center.y);
            PRINTGEOSTR("Result_pos:", it._pos);
            PRINTGEOSTR("Real_pos  :", it._realpos);
            PRINTLABEL("accuracy error : ", Functions::ComputeDistance(it._pos, it._realpos));
        }
    }
    
    FeatureFrame::FeatureFrame(const Mat &img,const GeoPos &pt, FeatureDetector *pDetector):Frame(img,pt)
    {
        assert(NULL != pDetector);
        pDetector->addRef();
        
        //提取特征点 并计算描述子
        pDetector->detect(img, mKeyPoints);
        pDetector->compute(img, mKeyPoints, mDescriptor);
        
        pDetector->release();
    }
    FeatureFrame::~FeatureFrame()
    {

    }
    
    
    
    OpticalFlowFrame::OpticalFlowFrame(const Mat &img,const GeoPos &pt,FeatureDetector *pDetector):Frame(img,pt)
    {
        assert(NULL != pDetector);
        pDetector->addRef();
        
        Mat imggray;
        
        cvtColor(img, imggray, CV_BGR2GRAY);
        
        pDetector->detect(imggray, mKeyPoints);
        
        pDetector->release();
    }
    
}
