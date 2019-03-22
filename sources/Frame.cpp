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
        
        PRINTGEOSTR("Frame Pos : ", mPos);
        
        PRINTLABEL("Target List : ", mTargets.size());
        
        for( auto it : mTargets )
        {
            PRINTLABEL("Type: ", it._id);
            PRINTLABEL("Pixel_x:", it._center.x);
            PRINTLABEL("Pixel_y:", it._center.y);
            PRINTGEOSTR("Calc_Pos:", it._pos);
            
#if TESTOUTPUT
            PRINTGEOSTR("Real_Pos  :", it._realpos);
            PRINTLABEL("Distance : ", Functions::ComputeDistance(it._pos, it._realpos));
#endif
        }
    }
    
    void Frame::drawTargetItem(const TargetItem &item)
    {
        rectangle(mImg,item._box,Scalar(255,0,0),3,LINE_AA);
        circle(mImg, item._center, 5, Scalar(0,0,255),FILLED);
    }
    
    void Frame::drawMatch(const PtVector &cur,const PtVector &oth)
    {
        if( cur.empty() || oth.empty() || oth.size() != cur.size())
            return;
        
        for(size_t i = 0; i < cur.size();++i)
        {
            circle(mImg, cur[i], 3, Scalar(255,255,255),FILLED);
            line(mImg, cur[i], oth[i], Scalar(0,0,0,3),2,LINE_AA);
        }
    }
    void Frame::drawLine(float a, float b, float c)
    {
        double y1 = -c / b;
        double y2 = -(c + a * mImg.cols) / b;
        line(mImg, Point(0, y1), Point(mImg.cols, y2), Scalar(0,0,0),2,LINE_AA);
    }
    
    void Frame::drawLine(const Point2f &bg,const Point2f &ed)
    {
        line(mImg, bg, ed,Scalar(255,255,255),2,LINE_AA);
    }
    
    void Frame::display()
    {
        if(!mImg.empty())
        {
            char winNm[255] = {0};
            sprintf(winNm, "%ld",(long)(this));
            
            Mat outimg = mImg.clone();
            
            for( TargetItem item : mTargets)
            {
                rectangle(outimg, item._box, Scalar(0,0,255),3);
                circle(outimg, item._center, 5, Scalar(0,0,255),FILLED);
                
#if TESTOUTPUT
                Scalar color(100,100,0);
                //绘制出极线    上面的验证点应该正好在绘制出的极线上
                drawLine(item.a, item.b, item.c);
#endif
            }
           
            const float dt = 0.4;
            resize(outimg, outimg, Size(),dt,dt);
           
            //imshow(winNm, outimg);
            static int index = 0; //名称计数
            char outpath[1024] = {0};
            
            sprintf(outpath, "%s/result_%d.png",SAVEPATH,index++);
            
            imwrite(outpath, outimg);
//            waitKey(0);
        }
    }
    
    FeatureFrame::FeatureFrame(const Mat &img,const GeoPos &pt, FeatureDetector *pDetector):Frame(img,pt)
    {
        assert(NULL != pDetector);
        pDetector->addRef();
        TimeInterval t;
        t.start();
        //提取特征点 并计算描述子
        pDetector->detect(mImg, mKeyPoints);
        pDetector->compute(mImg, mKeyPoints, mDescriptor);
        
        pDetector->release();
        t.print("create frame");
    }
  
    FeatureFrame::~FeatureFrame()
    {
        
    }
    
    OpticalFlowFrame::OpticalFlowFrame(const Mat &img,const GeoPos &pt,FeatureDetector *pDetector):Frame(img,pt)
    {
        TimeInterval t;
        t.start();
        cvtColor(img, mImg, CV_BGR2GRAY);
        
//        assert(NULL != pDetector);
//        pDetector->addRef();
//        
//        pDetector->detect(mImg, mKeyPoints);
//        
//        pDetector->release();
        t.print("create frame");
    }
    
}
