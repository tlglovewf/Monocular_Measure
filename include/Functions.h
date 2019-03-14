//
//  Functions.h
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/18.
//
#ifndef __FUNCTIONS_H_H_
#define __FUNCTIONS_H_H_
#include <cmath>
#include "mtypes.h"
namespace Monocular {
    const double PI64        = 3.1415926535897932384626433832795028841971693993751;
    const double Half_PI64   = 1.5707963267948965579989817342720925807952880859375;
    const double Earth_Radius= 6378137.0f;
    
#define D2R(X)   (X) * 0.01745329251994329547437168059786927187815308570862
#define R2D(X)   (X) / 0.01745329251994329547437168059786927187815308570862
#define FLATRATE  0.081819190782782381 // sqrt(EarthLongR * EarthLongR - EarthShortR * EarthShortR) / EarthLongR;
    
    class Functions
    {
    public:
        /**
         * 经纬度转墨卡托投影坐标
         @param lon 经度(degree）
         @param lat 维度
         @return    墨卡托投影坐标
         **/
        static cv::Point3d ComputeMerctorPosFromGPS( double lon, double lat)
        {
            cv::Point3d point;
            point.x = D2R(lon) * Earth_Radius  ;
            point.y = log(tan( D2R(lat) * 0.5 + 0.25 * PI64 )) * Earth_Radius;
            point.z = 0.0;
            return point;
        }
        
        /**
         * 墨卡托投影坐标转经纬度
         @param gps 墨卡托投影坐标
         @return    经纬度pos(lon,lat)
         **/
        static GeoPos ComputeGPSFromMerctorPos( const cv::Point3d &gps)
        {
            GeoPos gPos;
            gPos.x = R2D(gps.x / Earth_Radius) ;
            gPos.y = R2D(2 * ( atan( exp(gps.y / Earth_Radius))) - Half_PI64);
            return gPos;
        }
        
        /**
         * 计算距离
         @param gps 经纬度坐标
         @return    距离(m）
         **/
        static double ComputeDistance(double lng1, double lat1, double lng2, double lat2)
        {
            double radLat1 = D2R(lat1);
            double radLat2 = D2R(lat2);
            double a = radLat1 - radLat2;
            double b = D2R(lng1) - D2R(lng2);
            
            double s = 2 * asin(sqrt(pow(sin(a/2),2) +
                                     cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
            s = s * Earth_Radius;
            s = round(s * 10000) / 10000;
            return s;
        }
        /**
         * 计算距离
         @param geo1,geo2 经纬度坐标
         @return    距离(m）
         **/
        static double ComputeDistance(const GeoPos &geo1, const GeoPos &geo2)
        {
            return ComputeDistance(geo1.x,geo1.y,geo2.x,geo2.y);
        }
        
        
        /**
         * 像素坐标到像素物理坐标
         * @param   p 像素坐标
         * @param   K 内参矩阵
         & @return  像素物理坐标
         */
        static cv::Point2d Pixel2Cam(const cv::Point2d &p, const cv::Mat &K)
        {
            return cv::Point2d
            (
             (p.x-K.at<double>(0,2))/K.at<double>(0,0),
             (p.y-K.at<double>(1,2))/K.at<double>(1,1)
             );
        }
        
        /**
         * 获取相机->世界 转换矩阵
         * @param   xAxis x   direct   must normalizee
         * @param   zAxis eye direct   must normalize
         * @param   yAxis up  direct   must normalize
         & @return  world transmatrix
         */
        static cv::Mat ComputeWorldTransMatrix(const cv::Point3d &xAxis,
                                               const cv::Point3d &yAxis,
                                               const cv::Point3d &zAxis,
                                               const cv::Point3d &pt)
        {
            //构建世界变换矩阵  相机 -> 世界
            cv::Mat R = (cv::Mat_<double>(4,4) <<
                         xAxis.x , yAxis.x , zAxis.x  ,pt.x,
                         xAxis.y , yAxis.y,  zAxis.y  ,pt.y,
                         xAxis.z , yAxis.z , zAxis.z  ,pt.z,
                         0       ,0        ,0        ,1);
            return R;
        }
        
        /**
         * 经纬度转空间坐标系
         * @param   lon 经度
         * @param   lat 维度
         & @return  空间坐标点
         */
        static cv::Point3d ComputeXYZFromGPS(double lon, double lat,double H1 = 0)
        {
            double a = Earth_Radius;
            double e = FLATRATE;//sqrt(a * a - b * b) / a;
            double N = a / sqrt(1 - e * e * sin(D2R(lat)) * sin(D2R(lat)));
            double WGS84_X = (N + H1) * cos(D2R(lat)) * cos(D2R(lon));
            double WGS84_Y = (N + H1) * cos(D2R(lat)) * sin(D2R(lon));
            double WGS84_Z = (N * (1 - (e * e)) + H1) * sin(D2R(lat));
            
            return cv::Point3d(WGS84_X,WGS84_Y,WGS84_Z);
        }
        
        /**
         * 空间直角坐标系到经纬度坐标
         * @param   pt  空间坐标点
         & @return  经纬度
         */
        static GeoPos ComputeGPSFromXYZ(const cv::Point3d &pt)
        {
            double f, f1, f2;
            double p,zw, nnq;
            double b, l, h;
            
            double a = Earth_Radius;
            double eq = FLATRATE * FLATRATE;
            f = PI64 * 50 / 180;
            double x, y, z;
            x = pt.x;
            y = pt.y;
            z = pt.z;
            p = z / sqrt(x * x + y * y);
            do
            {
                zw = a / sqrt(1 - eq * sin(f) * sin(f));
                nnq = 1 - eq * zw / (sqrt(x * x + y * y) / cos(f));
                f1 = atan(p / nnq);
                f2 = f;
                f = f1;
            } while (!(abs(f2 - f1) < 10E-10));
            b = R2D(f);
            l = R2D( atan(y / x) );
            if (l < 0)
                l += 180.0;
            h = sqrt(x * x + y * y) / cos(f1) - a / sqrt(1 - eq * sin(f1) * sin(f1));
            return cv::Point2d{l, b};// h};
        }
        
        /**
         *  单位化
         **/
        static cv::Point3d Normalize( cv::Point3d &v)
        {
            double result;
            result = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            assert(0 != result);
            v = v / result;
            return v;
        }
        
        /**
         * 获取向量长度
         * @param   v 向量
         & @return  长度
         */
        static double GetLength(const cv::Point3d &v)
        {
            return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        }
        
        /*
         * 叉乘
         */
        inline static float  segmentCross
        (
         const Point2f&    v,
         const Point2f&    vStart,
         const Point2f&    vEnd
         )
        {
            return (vStart.x-v.x)*(vEnd.y-v.y) - (vEnd.x-v.x)*(vStart.y-v.y);
        }
#define YD_EPS_REAL32 1e-6
        /**
         * 判断线段与线段是否相交(跨立试验）
         * @param vStart1,vEnd1 线段1起终点
         * @param vStart2,vEnd2 线段2起终点
         * @return 是否相交
         */
        static bool  IsIntersect(const Point2f&    vStart1,const Point2f&    vEnd1,
                                 const Point2f&    vStart2,const Point2f&    vEnd2)
        {
            float leftS, leftE;
            leftS = segmentCross(vStart1, vStart2, vEnd2);
            leftE = segmentCross(vEnd1, vStart2, vEnd2);
            if ( leftS * leftE > YD_EPS_REAL32 )
            {
                return false;       // vStart1, vEnd1在另一条直线的同侧
            }
            
            leftS = segmentCross(vStart2, vStart1, vEnd1);
            leftE = segmentCross(vEnd2, vStart1, vEnd1);
            if ( leftS * leftE > YD_EPS_REAL32 )
            {
                return false;       // vStart2, vEnd2在另一条直线的同侧
            }
            
            return true;
        }
        
        
        /**
         * 判断直线是否与矩形相交
         * @param bg,ed 线段起终点
         * @param rect  矩形
         */
        static bool IsIntersect(const Point2f &bg,const Point2f &ed, const Rect2f &rect)
        {
            if(rect.contains(bg) || rect.contains(ed))
            {//先判断直线起终点是否在矩形内
                return true;
            }
            else
            {//再判断直线是否与矩形对角线相交
                return IsIntersect(bg, ed, Point2f(rect.x,rect.y),
                                   Point2f(rect.x+rect.width,rect.y+rect.height))||
                IsIntersect(bg, ed, Point2f(rect.x + rect.width,rect.y),
                            Point2f(rect.x,rect.y+rect.height));
            }
        }
        
        class Datum
        {
        public:
            double E2;
            double r_major;
            double r_minjor;
            Datum(double major,double minjor,double e2):r_major(major),r_minjor(minjor),E2(e2){}

            static  Datum WGS84Datum   ;
            static  Datum bj54Datum    ;
            static  Datum xian80Datum  ;
        };
        
        
        
        
        /*
         * 经纬度转高斯投影坐标（B 维度  L 精度   H 高程)
         */
        Point3d GaussProjCal(GeoPos BLH, Datum datum)//,double lon)
        {
            int ProjNo, ZoneWide; ////带宽
            double longitude0, X0, xval, yval;
            double a, e2, ee, NN, T, C, A, M, b, l;//, h;
            b = BLH.y;
            l = BLH.x;
            //    h = BLH.z;
            ZoneWide = 3; //3度带宽
            a = datum.r_major;
            ProjNo = (int)((l - 1.5) / ZoneWide + 1);
            longitude0 = ProjNo * ZoneWide; //中央经线
            
            longitude0 = longitude0 * PI64 / 180;
            l = l * PI64 / 180; //经度转换为弧度
            b = b * PI64 / 180; //纬度转换为弧度
            e2 = datum.E2;
            ee = e2 * (1.0 - e2);
            NN = a / sqrt(1.0 - e2 * sin(b) * sin(b));
            T = tan(b) * tan(b);
            C = ee * cos(b) * cos(b);
            A = (l - longitude0) * cos(b);
            
            M = a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * b - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * sin(2 * b) + (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * sin(4 * b) - (35 * e2 * e2 * e2 / 3072) * sin(6 * b));
            xval = NN * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * ee) * A * A * A * A * A / 120);
            yval = M + NN * tan(b) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * ee) * A * A * A * A * A * A / 720);
            X0 =  500000L;
            xval = xval + X0;
            return Point3d(xval, yval, 0);
        }
        
        
        //高斯投影由大地平面坐标(Unit:Metres)反算经纬度(Unit:DD)
        GeoPos GaussProjInvCal(Point3d XYZ, Datum datum, double lon)
        {
            int ProjNo, ZoneWide; ////带宽
            double l, b, longitude0, X0, xval, yval;
            double e1, e2, a, ee, NN, T, C, M, D, R, u, fai;
            a = datum.r_major; //54年北京坐标系参数
            ZoneWide = 3; //3度带宽
            
            ProjNo = (int)(XYZ.x / 1000000L); //查找带号
            longitude0 = (int)((lon - 1.5) / ZoneWide + 1) * ZoneWide; //中央经线
            
            longitude0 = longitude0 * PI64 / 180; //中央经线
            X0 = ProjNo * 1000000L + 500000L;
            xval = XYZ.x - X0; //带内大地坐标
            yval = XYZ.y;
            e2 = datum.E2;
            e1 = (1.0 - sqrt(1 - e2)) / (1.0 + sqrt(1 - e2));
            ee = e2 / (1 - e2);
            M = yval;
            u = M / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
            fai = u + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * u) + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * u) + (151 * e1 * e1 * e1 / 96) * sin(6 * u) + (1097 * e1 * e1 * e1 * e1 / 512) * sin(8 * u);
            C = ee * cos(fai) * cos(fai);
            T = tan(fai) * tan(fai);
            NN = a / sqrt(1.0 - e2 * sin(fai) * sin(fai));
            
            R = a * (1 - e2) / sqrt((1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)) * (1 - e2 * sin(fai) * sin(fai)));
            D = xval / NN;
            //计算经度(Longitude) 纬度(Latitude)
            l = longitude0 + (D - (1 + 2 * T + C) * D * D * D / 6 + (5 - 2 * C + 28 * T - 3 * C * C + 8 * ee + 24 * T * T) * D
                              * D * D * D * D / 120) / cos(fai);
            b = fai - (NN * tan(fai) / R) * (D * D / 2 - (5 + 3 * T + 10 * C - 4 * C * C - 9 * ee) * D * D * D * D / 24
                                             + (61 + 90 * T + 298 * C + 45 * T * T - 256 * ee - 3 * C * C) * D * D * D * D * D * D / 720);
            //转换为度 DD
            l = l * 180 / PI64;
            b = b * 180 / PI64;
            return GeoPos(l, b);//, XYZ.Z);
        }
    };
}
#endif /*__FUNCTIONS_H_H_*/

