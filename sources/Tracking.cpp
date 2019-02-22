//
//  Tracking.cpp
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/18.
//

#include "Tracking.h"
#include "FeatureDetector.h"
#include "Frame.h"
#include "Matcher.h"

namespace Monocular {
    
    
    Tracking::Tracking(eFrameType ftype,eMatcherType mtype /*= eHammingDistanceMatcher*/):mFmType(ftype),mState(eNo_Image_Yet),mpCurrentFrame(NULL),mpPreFrame(NULL)
    {
        switch (ftype) {
            case eFeatureFrame:
                {
                    mpMatcher = Matcher::CreateMatcher(mtype);
                    mpFeatureDetector = FeatureDetector::CreateDetector(eORB_TYPE);
                }
                break;
                
            case eOpticalFlowFrame:
            {
                mpMatcher = Matcher::CreateMatcher(eOpticalFlowMatcher);
                mpFeatureDetector = FeatureDetector::CreateDetector(eFAST_TYPE);
            }
                break;
        }
       
    }
    
    void Tracking::reset()
    {
        resetState();
        
        //release resources
        DESTROYOBJECT(mpPreFrame);
        DESTROYOBJECT(mpCurrentFrame);
        DESTROYOBJECT(mpMatcher);
        DESTROYOBJECT(mpFeatureDetector);
    }
    
    bool Tracking::grabImage(const Mat &img,const GeoPos &geopt,const TargetItems &items)
    {
        if(!img.empty())
        {
            if( eNo_Image_Yet == mState )
            {//没有需要跟踪的图片状态
                mpCurrentFrame = Frame::CreateFrame(mFmType, img, geopt,mpFeatureDetector);
                mpCurrentFrame->setTargetItems(items);
                mState = eOk;
                
#ifdef NEEDPRINTDEBUGFINO
                PRINTGEOSTR("pre frame pos : ",geopt);
#endif
            }
            else
            {//存在需要的跟踪
                assert(NULL != mpCurrentFrame);
                
                if(NULL != mpPreFrame)
                {//前帧存在时,先释放
                    mpPreFrame->release();
                }
                
                mpPreFrame = mpCurrentFrame;//赋值前帧
                
                mpCurrentFrame = Frame::CreateFrame(mFmType, img, geopt,mpFeatureDetector);
                mpCurrentFrame->setTargetItems(items);
                
#ifdef NEEDPRINTDEBUGFINO
                PRINTGEOSTR("cur frame pos : ",geopt);
#endif
            }
            return track();
        }
        else
        {
            return false;
        }
    }
    
    bool Tracking::track()
    {
        assert(mpMatcher);
        assert(mpCurrentFrame);
        
        if(NULL != mpPreFrame)
        {
            mpMatcher->match(mpPreFrame, mpCurrentFrame);

            return  mpMatcher->hasData();
        }
        else
        {
            return false;
        }
    }

    void Tracking::getMatchVector(PtVector &prepts, PtVector &curpts)const
    {
        mpMatcher->getMatchVector(prepts, curpts);
    }
}
