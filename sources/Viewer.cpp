//
//  Viewer.cpp
//  Monocular_Measure
//
//  Created by TuLigen on 2019/3/12.
//

#include "Viewer.h"
#include <opencv2/imgproc.hpp>
namespace Monocular {
    
    
#define WIN_SIZE      600
#define HALF_WIN_SIZE 300
#define BOTTOM_POS    (WIN_SIZE - 100)
    DefaultViewer::DefaultViewer()
    {
        mCanvas = Mat::zeros(WIN_SIZE, WIN_SIZE, CV_8UC3);
    }
    
    DefaultViewer::~DefaultViewer()
    {
        
    }
    
    void Parallel_Pic(const Mat &first, const Mat &second,Mat &out)
    {
#if 1
        out = Mat( max(first.rows,second.rows) ,first.cols + second.cols,first.type());
        
        first.copyTo(out(Rect(0,0,first.cols,first.rows)));
        second.copyTo(out(Rect(first.cols,0,second.cols,second.rows)));
#else
        out = Mat(first.rows + second.rows,max(first.cols,second.cols),first.type());
        
        first.copyTo(out(Rect(0,0,first.cols,first.rows)));
        second.copyTo(out(Rect(first.cols,0,second.cols,second.rows)));

#endif
    }
    
    void DefaultViewer::render()
    {
        rectangle( mCanvas, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
        char text[256] = {0};
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", mIndicatorT.at<double>(0), mIndicatorT.at<double>(1), mIndicatorT.at<double>(2));
        int fontFace = FONT_HERSHEY_PLAIN;
        double fontScale = 1;
        int thickness = 1;
        cv::Point textOrg(10, 50);
        putText(mCanvas, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
        
        int x = int(mIndicatorT.at<double>(0)) + HALF_WIN_SIZE;
        int y = int(mIndicatorT.at<double>(2)) + BOTTOM_POS   ;
        circle(mCanvas, Point(x, y) ,1, CV_RGB(255,0,0), 2);//红色计算轨迹
        
//        x = x + mTarget.x;
//        y = y - mTarget.z;
//
//        circle(mCanvas, Point(x, y) ,1, CV_RGB(0,255,0), 2);//红色计算轨迹
        
        resize(mCurImg, mCurImg, Size(mCanvas.rows,mCanvas.cols));
        
        Mat out;
        
        if(mCurImg.channels()<3)
        {
            Mat grayCanvas;
            cvtColor(mCanvas, grayCanvas, CV_BGR2GRAY);
            
            Parallel_Pic(grayCanvas,mCurImg,out);
        }
        else
        {
            Parallel_Pic(mCanvas,mCurImg,out);
        }
        
        imshow("mg", out);
        
        
        waitKey(1);
    }
    
    void DefaultViewer::setCurrentPose(const Mat &img,const Mat &R, const Mat &t, double scale)
    {
        mIndicatorT = mIndicatorT + scale * (mIndicatorR * t);
        mIndicatorR = R * mIndicatorR ;
        mCurImg = img;
    }
    
    void DefaultViewer::setTargetPose(const Point3d &pt)
    {
//        mTarget = pt;
//        Mat temp = (Mat_<double>(3,1) << pt.z,pt.y,pt.x);
////
//        temp = mIndicatorR * temp;
//        double x = temp.at<double>(0,0);
//        double y = temp.at<double>(0,1);
//        double z = temp.at<double>(0,2);
//        mTarget = Point3d( x / z,y / z,0);
    }
}
