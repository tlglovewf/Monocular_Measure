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
    
    
    void knn_match(const Mat &descriptor1,const Mat &descriptor2, const BFMatcher &match,MatchVector &matches)
    {
        const float minRatio = 1.f / 1.2f;
        const int k = 2;
        
        std::vector<std::vector<DMatch> > knnMatches;
        match.knnMatch(descriptor1, descriptor2, knnMatches, k);
        
        for (size_t i = 0; i < knnMatches.size(); i++) {
            const DMatch& bestMatch = knnMatches[i][0];
            const DMatch& betterMatch = knnMatches[i][1];
            
            float  distanceRatio = bestMatch.distance / betterMatch.distance;
            if (distanceRatio < minRatio)
                matches.push_back(bestMatch);
        }
    }
    
    
    void HammingDistanceMatcher::match(const Frame *pPreFrame,const Frame *pCurFrame)
    {
        assert(pPreFrame);
        assert(pCurFrame);
        
        const Mat &descriptorLeft = pPreFrame->getDescriptor();
        const Mat &descriptorRight= pCurFrame->getDescriptor();
        MatchVector tmpmatches;
        mMatcher.match(descriptorLeft, descriptorRight, tmpmatches);
        double min_dist = 10000, max_dist = 0;
        
        for(size_t i = 0; i < descriptorLeft.rows;++i)
        {
            double dist = tmpmatches[i].distance;
            if(dist < min_dist)min_dist = dist;
            if(dist > max_dist)max_dist = dist;
        }
        assert(mPrePts.empty());
        
        
#if TESTOUTPUT
        MatchVector goods;
#endif
        const KeyPointVector &prekey = pPreFrame->getKeyPoints();
        const KeyPointVector &curkey = pCurFrame->getKeyPoints();
#if 0
        //效果不好 暂时舍弃
        for(int i = 0; i < tmpmatches.size();++i)
        {
            if( tmpmatches[i].distance <=  0.3 * (max_dist + min_dist ))
            {
                
                Point2f prept = prekey[tmpmatches[i].queryIdx].pt;
                Point2f curpt = curkey[tmpmatches[i].trainIdx].pt;
                const int len = 30;//像素距离
                if( fabs(curpt.x - prept.x) < len &&
                    fabs(curpt.y - prept.y) < len)
                {
                    mPrePts.emplace_back(prept);
                    mCurPts.emplace_back(curpt);
#if TESTOUTPUT
                    goods.emplace_back(tmpmatches[i]);
#endif
                }
            }
        }
#else
        
        knn_match(descriptorLeft, descriptorRight, mMatcher, goods);

        for(auto gd : goods)
        {
            mPrePts.emplace_back(prekey[gd.queryIdx].pt);
            mCurPts.emplace_back(curkey[gd.trainIdx].pt);
        }
        
#endif
#if TESTOUTPUT  //保存特征匹配图片
        Mat outimg;
        drawMatches(pPreFrame->getImg(),  pPreFrame->getKeyPoints(), pCurFrame->getImg(), pCurFrame->getKeyPoints(), goods, outimg);
         char outpath[1024] = {0};
         sprintf(outpath, "%s/result_match.png",SAVEPATH);
        imwrite(outpath, outimg);
        std::cout << "matching pt size :" << mPrePts.size() << std::endl;
#endif
    
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
            int rows = (preimg.rows + preimg.cols) >> 2 ;
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
