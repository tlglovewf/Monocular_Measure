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
    {//直接交换数据
        prepts.swap(mPrePts);
        curpts.swap(mCurPts);
    }
    
    
    void  OpticalFlowMatcher::match(const Frame *pPreFrame,const Frame *pCurFrame)
    {
        //this function automatically gets rid of points for which tracking fails
        assert(mPrePts.empty());
        
        std::vector<uchar> status;
        /*
         * Shi-Tomasi算子
         *
         */
        const Mat &preimg = pPreFrame->getImg();
        
        if(!pPreFrame->getKeyPoints().empty())
        {//有提取特征点就直接转换,无就进行追踪
            KeyPoint::convert(pPreFrame->getKeyPoints(), mPrePts);
        }
        else
        {
            int rows = (preimg.rows + preimg.cols) >> 2;
            goodFeaturesToTrack(preimg, mPrePts, rows, 0.01, 10, Mat(),10,false);
        }
    
        /// 角点位置精准化参数
        Size winSize = Size( 5, 5 );
        Size flWinSize = Size(18,18);
        Size zeroZone = Size( -1, -1 );
        TermCriteria criteria = TermCriteria(
                                             CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
                                             40, //maxCount=40
                                             0.001 );    //epsilon=0.001
        FloatVector err;
        TermCriteria termcrit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
        /// 计算亚像素级
        cornerSubPix( preimg, mPrePts, winSize, zeroZone, criteria );
        
        calcOpticalFlowPyrLK(preimg, pCurFrame->getImg(), mPrePts, mCurPts, status, err, flWinSize, 3, termcrit, 0, 0.001);
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
