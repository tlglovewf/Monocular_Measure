#include "Matcher.h"
#include "Frame.h"
#include <opencv2/video/tracking.hpp>
namespace Monocular
{
  
    Matcher* Matcher::CreateMatcher(Monocular::eMatcherType type)
    {
        switch (type) {
            case eHammingDistanceMatcher:
                return new HammingDistanceMatcher;
            case eOpticalFlowMatcher:
                return new OpticalFlowMatcher;
            default:
                return NULL;
        }
    }
    
    HammingDistanceMatcher::HammingDistanceMatcher():mMatcher(NORM_HAMMING)
    {
        
    }
    HammingDistanceMatcher::~HammingDistanceMatcher()
    {
        
    }
    
    void HammingDistanceMatcher::match(const Frame *pPreFrame,const Frame *pCurFrame)
    {
        assert(pPreFrame);
        assert(pCurFrame);
        
        const Mat &descriptorLeft = pPreFrame->getDescriptor();
        const Mat &descriptRight = pCurFrame->getDescriptor();
        MatchVector tmpmatches;
        mMatcher.match(descriptorLeft, descriptRight, tmpmatches);
        
        double min_dist = 10000, max_dist = 0;
        
        for(size_t i = 0; i < descriptorLeft.rows;++i)
        {
            double dist = tmpmatches[i].distance;
            if(dist < min_dist)min_dist = dist;
            if(dist > max_dist)max_dist = dist;
        }
        
        //        std::cout << " mindist : " << min_dist << endl;
        //        std::cout << " maxdist : " << max_dist << endl;
        assert(mPrePts.empty());
        
        for(int i = 0; i < tmpmatches.size();++i)
        {
#if 0
            const int sz = 10;
            Rect2f rect(0,0,sz,sz);
            if( (keypoints_1[tmpmatches[i].queryIdx].pt - keypoints_2[tmpmatches[i].trainIdx].pt).inside(rect))
                continue;
#else
            //            Rect2f  rect(0,1600,4096,600);
            //            if(keypoints_1[tmpmatches[i].queryIdx].pt.inside(rect) )
            //                continue;
            
#endif
            const KeyPointVector &prekey = pPreFrame->getKeyPoints();
            const KeyPointVector &curkey = pCurFrame->getKeyPoints();
            if( tmpmatches[i].distance <= max(2 * min_dist,30.0) )
            {
                mPrePts.emplace_back( prekey[tmpmatches[i].queryIdx].pt);
                mCurPts.emplace_back( curkey[tmpmatches[i].trainIdx].pt);
            }
        }
    }
    
    
    void Matcher::getMatchVector(PtVector &prepts, PtVector &curpts)
    {
        prepts.swap(mPrePts);
        curpts.swap(mCurPts);
    }
    
    
    void  OpticalFlowMatcher::match(const Frame *pPreFrame,const Frame *pCurFrame)
    {
        //this function automatically gets rid of points for which tracking fails
        
        FloatVector err;
        Size winSize = Size(21,21);
        TermCriteria termcrit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

        assert(mPrePts.empty());
        
        std::vector<uchar> status;
        KeyPoint::convert(pPreFrame->getKeyPoints(), mPrePts);
        
        calcOpticalFlowPyrLK(pPreFrame->getImg(), pCurFrame->getImg(), mPrePts, mCurPts, status, err, winSize, 3, termcrit, 0, 0.001);
        
        //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
        int indexCorrection = 0;
        for( int i=0; i < status.size(); i++)
        {
            Point2f pt = mCurPts.at(i- indexCorrection);
            if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))    {
                if((pt.x<0)||(pt.y<0))    {
                    status.at(i) = 0;
                }
                mPrePts.erase (mPrePts.begin() + (i - indexCorrection));
                mCurPts.erase (mCurPts.begin() + (i - indexCorrection));
                indexCorrection++;
            }
        }
    }
}
