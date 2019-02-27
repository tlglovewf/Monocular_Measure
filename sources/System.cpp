#include "System.h"
#include "Tracking.h"
#include "Frame.h"
#include "PoseEstimation.h"
#include <fstream>
namespace Monocular {
    System::System(const Camera &cam,eTrackingMode mode,const std::string &outPath)
    {
        mpTracker       =  new Tracking(static_cast<eFrameType>((int)mode));
        mpEstimation    =  new PoseEstimation(cam);
        mpSerialization =  new FileSerialization(outPath);
    }
    
    System::~System()
    {
        resetList();
        DESTROYOBJECT(mpTracker);
        DESTROYOBJECT(mpEstimation);
        DESTROYOBJECT(mpSerialization);
    }
    
    //检测框大小
    inline bool checkRect(const Rect2f &rect)
    {
        return (rect.height > MINOBJECTSIZE) && (rect.width > MINOBJECTSIZE);
    }
    
    TargetItems System::objectDetect(const cv::Mat &img,int sample)
    {
        //add more ...
        static int index = 0;
        TargetItems items;
        const float sz = 50.0f;
        if( index++ % 2 == 0 )
        {
            Point2f pt;
            TargetItem item;
            if(sample)
            {
                pt = Point2f(2756,866);
               item = TargetItem {0,pt,Rect2f(pt.x - sz,pt.y - sz,sz * 2,sz * 2)};
#if TESTOUTPUT
              item._realpos = GeoPos(114.40350395, 30.60123760);
#endif
            }
            else
            {
                Rect2f rect = Rect2f(2307,635,93,123);
                pt = Point2f (rect.x + rect.width / 2,rect.y + rect.height / 2);
                item = TargetItem {0,pt,rect};
#if TESTOUTPUT
                item._realpos = GeoPos(121.46771743, 31.21026780);
#endif
            }

            if(checkRect(item._box))
                items.emplace_back(item);
            else
                assert(NULL);
        }
        else
        {
            Point2f pt;
            TargetItem item;
            if(sample)
            {
                pt = Point2f(2907,827);
                item = TargetItem {0,pt,Rect2f(pt.x - sz,pt.y - sz,sz * 2,sz * 2)};
            }
            else
            {
                Rect2f rect = Rect2f(2447,537,123,156);
                pt = Point2f (rect.x + rect.width / 2,rect.y + rect.height / 2);
                item = TargetItem{0,pt,rect};
            }
             if(checkRect(item._box))
                 items.emplace_back(item);
            else
                assert(NULL);
        }
        
        
        return items;
    }
    
    
    void System::handle(const cv::Mat &img,const GeoPos &geopt,const TargetItems &items)
    {
        if( NULL != mpTracker )
        {
            //add more ..
            if(mpTracker->grabImage(img,geopt,items))
            {//跟踪成功,即匹配对有值
                assert(mpEstimation);
                PtVector prepts,curpts;
                mpTracker->getMatchVector(prepts,curpts);
                Frame *pPreFrame = mpTracker->getFrame(ePreFrame);
                Mat tcw =  mpEstimation->estimate( pPreFrame, mpTracker->getFrame(eCurFrame), prepts, curpts,mpSerialization);
                pPreFrame->setWordTransform(std::move(tcw));
                
                //将有计算结果的帧添加到帧列表中
                addList(pPreFrame);
            }
            
            //add more ..
        }
    }
    
    void System::addList(Frame *frame)
    {
        assert(frame);
        frame->addRef();
        mFrameList.emplace_back(frame);
    }
    
    void System::resetList()
    {
        for( auto it : mFrameList )
        {
            assert(it);
            it->release();
        }
    }
    
    void System::printResult()
    {
        
#if TESTOUTPUT
        for(auto it : mFrameList)
        {
            assert(it);
//            it->print();//打印每帧的数据
            it->display();
        }
#endif
        
#ifdef NEEDWRITEFILE
        assert(mpSerialization);
        
        std::ifstream file;
        
        file.open(mpSerialization->getPath());
        
        while(!file.eof())
        {
            char temp[1024] = {0};
            file.getline(temp, 1024);
            std::cout << temp << std::endl;
        }
        
        file.close();
#endif
    }
    
    void System::reset()
    {
        if(NULL != mpTracker)mpTracker->reset();
    }
    

}
