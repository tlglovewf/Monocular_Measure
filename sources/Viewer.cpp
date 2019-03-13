//
//  Viewer.cpp
//  Monocular_Measure
//
//  Created by TuLigen on 2019/3/12.
//

#include "Viewer.h"
#include <opencv2/imgproc.hpp>
namespace Monocular {
    
    
#define WIN_SIZE      1000
#define HALF_WIN_SIZE 500
    DefaultViewer::DefaultViewer()
    {
        mCanvas = Mat::zeros(WIN_SIZE, WIN_SIZE, CV_8UC3);
    }
    
    DefaultViewer::~DefaultViewer()
    {
        
    }
    
    void merge(const Mat &first, const Mat &second,Mat &out)
    {
        out = Mat(max(first.rows,second.rows),first.cols + second.cols,first.type());
        
        first.colRange(0, first.cols).copyTo(out.colRange(0, first.cols));
        
        second.colRange(0, second.cols).copyTo(out.colRange(first.cols, out.cols));
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
        int y = int(mIndicatorT.at<double>(2)) + HALF_WIN_SIZE ;
        circle(mCanvas, Point(x, y) ,1, CV_RGB(255,0,0), 2);//红色计算轨迹
        
//        imshow("traj", mCanvas);
        
        resize(mCurImg, mCurImg, Size(mCurImg.cols >> 2,mCurImg.rows >> 2));
        
        Mat out;
        merge(mCanvas,mCurImg);
        
//        imshow("img", mCurImg);
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
        
    }
}
