//
//  PoseEstimation.h
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/18.
//
#ifndef __POSEESTIMATION_H_H__
#define __POSEESTIMATION_H_H__

#include "BaseObject.h"
#include "mtypes.h"

namespace Monocular {
    class Frame;
    /*
     * 块匹配算法
     */
    enum eBlockMatchingType
    {
      eNCC,
      eSAD,
      eSSD
    };
    
    //状态恢复对象
    class PoseEstimation : public BaseObject
    {
    public:
        PoseEstimation(const Camera &cam):mCamera(cam)
        {
            
        }
        
        
        /*
         *  恢复目标点坐标
         *
         * @param preFrame      前帧
         * @param curFrame      后帧
         * @param prepts        前帧中匹配对中的像素点集
         * @param curpts        后帧中匹配对中的像素点集
         * @param scale         真实尺度(默认表示没有真实尺度,则计算两帧经纬度距离)
         * @return              恢复点坐标(世界坐标 经纬度)
         */
        void estimate(const Frame *preFrame,const Frame *curFrame,const PtVector &prepts,const PtVector &curpts,float realscale = -1.0);
        
        /*
         * 基于基础矩阵的2d-2d状态恢复
         *
         * @param pt1       前一帧特征点集合
         * @param pt2       当前帧特征点集合
         * @param matches   匹配对
         * @param R         相机旋转矩阵
         * @param t         相机平移矩阵
         */
        void estimateF_2d2d(const KeyPointVector &kypt1, const KeyPointVector &kypt2,const MatchVector &matches, Mat &R, Mat &t);
        
        /*
         * 基于基础矩阵的2d-2d状态恢复
         *
         * @param pt1       前一帧特征点集合
         * @param pt2       当前帧特征点集合
         * @param matches   匹配对
         * @param R         相机旋转矩阵
         * @param t         相机平移矩阵
         */
        void estimateF_2d2d(const PtVector &pt1, const PtVector &pt2, Mat &R, Mat &t);
        
        
        //基于单应矩阵
        void estimateH_2d2d();
        
        
        /*
         * 基于2d-3d状态恢复(待添加)
         *
         */
        void estimate_2d3d(...)
        {
            //add more..
        }
        
        /*
         * 三角测量
         * @param pixel_1   前帧特征点坐标
         * @param pixel_2   后帧特征点坐标
         * @param scale     两帧实际距离
         * @param R         两帧间旋转矩阵
         * @param t         两帧平移矩阵
         * @output          相机坐标系坐标
         */
        void triangulation(const Point2f &pixel_1,const Point2f &pixel_2,float scale,const Mat &R, const Mat &t, Point3d &output);
        
        
        /*
         * 计算极线
         * @fd      基础矩阵
         * @piexl   图1中像素点
         * @a,b,c   piexl计算出的 直线方程  ax+by+c = 0
         */
        void calcEpiline(const Mat &fdMat,const Point2f &piexl, float &a, float &b, float &c);
        void calcEpiline(const PtVector &pt1, const PtVector &pt2,const PtVector &piexls, FloatVector &a, FloatVector &b, FloatVector &c);
        
        
        /*
         * 极线搜索
         * @img1    前帧图像
         * @img2    当前帧图像
         * @piexl   需要匹配的像素点坐标
         * @a,b,c   极线 ax+by+c = 0
         * @type    块匹配类型
         * @return  匹配的点
         */
        Point2f epilineSearch(const Mat &img1,const Mat &img2,const Point2f &piexl, float a,float b, float c,eBlockMatchingType type = eNCC);
        
        /*
         * 极线搜索
         * @targets 匹配帧中所有对象
         * @item    前帧目标对象
         * @a,b,c   极线 ax+by+c = 0
         * @return  匹配的点
         */
        Point2f epilineSearch(const TargetItems &targets,const TargetItem &item,float a,float b, float c);
        
    protected:
        //根据orb-slam 内容 应该是要根据计算基础和单应矩阵的模型评分,然后最后进行选择最优进行状态恢复,此处暂留,待后续优化
        //这里可参考 尝试orb-slam里的内容
        /*
         * 计算基础矩阵
         */
        Mat findFundamentalMat(const PtVector &pt1,const PtVector &pt2,float &score);
        
        /*
         * 计算单应矩阵
         */
        Mat findHomography(const PtVector &pt1,const PtVector &pt2,float &score)
        {
            //add more..
            return Mat();
        }
        
        /*
         * opencv 三角测量函数实现
         */
        void triangulationCvImpl(const Point2d &pt1,const Point2d &pt2,const Mat &camPrj1,const Mat &camPrj2,Mat &pt_4d);
        
        /*
         *
         */
        void triangulationLinearLSTImpl(const Point2d &pt1,const Point2d &pt2,const Mat &camPrj1,const Mat &camPrj2,Mat &pt_4d);
        
        
        
        GeoPos calcWorldPos(const GeoPos &preGps, const GeoPos &curGps,const Point3d &target);
        
        //add more
        
    protected:
        Camera mCamera;
    };
    
    //块匹配
    class BlockMatch : public BaseObject
    {
    public:
        /*
         * 块评分
         * @param cur 当前帧
         * @param pt  像素点
         * @return    评分
         */
        virtual double score(const Mat &cur,const Point2f &pt) = 0;
        
        /*
         * 创建对象
         */
        static BlockMatch* CreateMethod(eBlockMatchingType type,const Mat &img, const Point2f &pt);
    };
    //NCC 算法
    class NCC_BlockMatch : public BlockMatch
    {
        const int ncc_window_size = 8;    // NCC 取的窗口半宽度
        const int ncc_area = (2*ncc_window_size+1)*(2*ncc_window_size+1); // NCC窗口面积
    public:
        /*
         * 构造函数
         */
        NCC_BlockMatch(const Mat &mat,const Point2f &pt);
        
        /*
         * 块评分
         * @param cur 当前帧
         * @param pt  像素点
         * @return    评分
         */
        virtual double score(const Mat &cur,const Point2f &pt);
    protected:
        // 双线性灰度插值
        inline double getBilinearInterpolatedValue( const Mat& img, const Point2d& pt ) {
            uchar* d = & img.data[ int(pt.y)*img.step+int(pt.x) ];
            double xx = pt.x - floor(pt.x);
            double yy = pt.y - floor(pt.y);
            return  (( 1-xx ) * ( 1-yy ) * double(d[0]) +
                     xx* ( 1-yy ) * double(d[1]) +
                     ( 1-xx ) *yy* double(d[img.step]) +
                     xx*yy*double(d[img.step+1]))/255.0;
        }
    protected:
        const Mat&           _mat;
        double               _mean;
        std::vector<double>  _values;
        double               _demoniator;
    };
    
    //add more   like  SSD   SAD
    
    class SSD_BlockMatch : public BlockMatch
    {
    public:
        /*
         * 块评分
         * @param cur 当前帧
         * @param pt  像素点
         * @return    评分
         */
        virtual double score(const Mat &cur,const Point2f &pt) {return 0.0;}
    };
    
    class SAD_BlockMatch : public BlockMatch
    {
    public:
        /*
         * 块评分
         * @param cur 当前帧
         * @param pt  像素点
         * @return    评分
         */
        virtual double score(const Mat &cur,const Point2f &pt) {return 0.0;}
    };
}
#endif /*__POSEESTIMATION_H_H__*/

