//
//  Matcher.h
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/18.
//
#ifndef __MATCHER_H_H__
#define __MATCHER_H_H__
#include "BaseObject.h"
#include "mtypes.h"
namespace Monocular {
    
    class Frame;
    //匹配类
    class Matcher : public BaseObject
    {
    public:
        
        /*
         * 获取匹配对
         * @param pPreFrame  前帧
         * @param pCurFrame  后帧
         * @param matches    匹配对
         */
        virtual void match(const Frame *pPreFrame,const Frame *pCurFrame) = 0;
        
        /*
         * 判定是否有匹配数据
         */
        virtual bool hasData()const = 0;
        
        /*
         * 获取匹配点集合
         * @param prepts 前帧匹配点
         * @param curpts 后帧匹配点
         */
        virtual void getMatchVector( PtVector &prepts, PtVector &curpts) ;
        
        /*
         * 创建匹配对象
         *
         */
        static Matcher* CreateMatcher(eMatcherType type);
        
        
    protected:
        PtVector mPrePts;
        PtVector mCurPts;
    };
    
    //基于汉明距离 筛选匹配
    class HammingDistanceMatcher : public Matcher
    {
    public:
        HammingDistanceMatcher();
        ~HammingDistanceMatcher();
        /*
         * 获取匹配对
         * @param pPreFrame  前帧
         * @param pCurFrame  后帧
         * @param matches    匹配对
         */
        virtual void match(const Frame *pPreFrame,const Frame *pCurFrame);
        
        /*
         * 判定是否有匹配数据
         */
        virtual bool hasData()const
        {
            return !mPrePts.empty();
        }
    protected:
        
        BFMatcher   mMatcher;

    };
    
    //光流匹配
    class OpticalFlowMatcher : public Matcher
    {
    public:
        /*
         * 获取匹配对
         * @param pPreFrame  前帧
         * @param pCurFrame  后帧
         * @param matches    匹配对
         */
        virtual void match(const Frame *pPreFrame,const Frame *pCurFrame) ;
        
        /*
         * 判定是否有匹配数据
         */
        virtual bool hasData()const
        {
            return true;
        }
    };
}


#endif /*__MATCHER_H_H__*/
