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
    
    void Frame::display()
    {
        if(!mImg.empty())
        {
            char winNm[255] = {0};
            sprintf(winNm, "%ld",(long)(this));
            
            Mat outimg = mImg.clone();
            
            for( TargetItem item : mTargets)
            {
                rectangle(outimg, item._box, Scalar(0,255,0),3);
                circle(outimg, item._center, 5, Scalar(0,0,255),FILLED);
                
#if TESTOUTPUT
                Scalar color(100,100,0);
                //绘制出极线    上面的验证点应该正好在绘制出的极线上
                double y1 = -item.c / item.b;
                double y2 = -(item.c + item.a * outimg.cols) / item.b;
                line(outimg, Point(0, y1), Point(outimg.cols, y2), color,2,LINE_AA);
#endif
            }
           
            const float dt = 0.4;
            resize(outimg, outimg, Size(),dt,dt);
           
            //imshow(winNm, outimg);
            static int index = 0; //名称计数
            char outpath[1024] = {0};
            sprintf(outpath, "%s/%d.png",OUTPUTPATH,index++);
            imwrite(outpath, outimg);
            waitKey(0);
        }
    }
    
    FeatureFrame::FeatureFrame(const Mat &img,const GeoPos &pt, FeatureDetector *pDetector):Frame(img,pt)
    {
        assert(NULL != pDetector);
        pDetector->addRef();
        
        //提取特征点 并计算描述子
        pDetector->detect(mImg, mKeyPoints);
        pDetector->compute(mImg, mKeyPoints, mDescriptor);
        
        pDetector->release();
    }
  
    FeatureFrame::~FeatureFrame()
    {
        
    }
    
    OpticalFlowFrame::OpticalFlowFrame(const Mat &img,const GeoPos &pt,FeatureDetector *pDetector):Frame(img,pt)
    {
//        assert(NULL != pDetector);
//        pDetector->addRef();
        
//        Mat imggray;
        
        cvtColor(img, mImg, CV_BGR2GRAY);
        
        
        
//        pDetector->detect(imggray, mKeyPoints);
//
//        pDetector->release();
    }
    
}
