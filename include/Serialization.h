//
//  Serialization.h
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/18.
//
#ifndef __SERIALIZATION_H_H__
#define __SERIALIZATION_H_H__
#include "BaseObject.h"
#include "mtypes.h"
#include <string>
#include <fstream>
namespace Monocular {
    class Frame;
    class PoseEstimation;
    //序列化接口
    class Serialization  : public BaseObject
    {
    public:
        /*
         * 格式化写经纬度
         * @param prompt 标签
         * @param geops  经纬度
         */
        virtual void writeFormat(const std::string &prompt,const GeoPos &geopos) = 0;
        
        /*
         * 格式化写值
         * @param prompt 标签
         * @param value  float值
         */
        virtual void writeFormat(const std::string &prompt,float value) = 0;
        
        /*
         * 格式化写值
         * @param prompt 标签
         * @param mat    矩阵
         */
        virtual void writeFormat(const std::string &prompt,const Mat &mat) = 0;
        
        /*
         * 格式化写值
         * @param prompt 标签
         * @param rect   边框
         */
        virtual void writeFormat(const std::string &prompt,const Rect2f &rect) = 0;
        
        /*
         * 格式化写值
         * @param prompt 标签
         * @param point  点
         */
        virtual void writeFormat(const std::string &prompt,const Point2f &pt) = 0;
        
        /*
         * 格式化写值
         * @param prompt 标签
         * @param point  点
         */
        virtual void writeFormat(const std::string &prompt,const Point3d &pt) = 0;
        
        /*
         * 打印字符
         */
        virtual void prompt(const std::string &prompt) = 0;
        
        /*
         * 写入
         */
        virtual void writeFlush() = 0;
        
        /*
         * 路径
         */
        virtual std::string getPath()const = 0;
        
    };
    
    //帧文件序列化对象
    class FileSerialization : public Serialization,public std::ofstream
    {
    public:
        FileSerialization(const std::string &path);
        ~FileSerialization()
        {
            close();
        }
        /*
         * 序列化
         */
        virtual void serialize(const Frame &f);
        
        
        /*
         * 格式化写经纬度
         * @param prompt 标签
         * @param geops  经纬度
         */
        virtual void writeFormat(const std::string &prompt,const GeoPos &geopos);
        
        /*
         * 格式化写值
         * @param prompt 标签
         * @param value  float值
         */
        virtual void writeFormat(const std::string &prompt,float value);
        
        /*
         * 格式化写值
         * @param prompt 标签
         * @param mat    矩阵
         */
        virtual void writeFormat(const std::string &promt,const Mat &mat);
        
        /*
         * 格式化写值
         * @param prompt 标签
         * @param rect   边框
         */
        virtual void writeFormat(const std::string &prompt,const Rect2f &rect);
        
        /*
         * 格式化写值
         * @param prompt 标签
         * @param point  点
         */
        virtual void writeFormat(const std::string &prompt,const Point2f &pt);
        
        /*
         * 格式化写值
         * @param prompt 标签
         * @param point  点
         */
        virtual void writeFormat(const std::string &prompt,const Point3d &pt) ;
        
        /*
         * 打印字符
         */
        virtual void prompt(const std::string &prompt);
        
        /*
         * 写入
         */
        virtual void writeFlush()
        {
            this->flush();
        }
        
        virtual std::string getPath()const
        {
            return mPath;
        }
        
    protected:
        /*
         * 路径
         */
        std::string mPath;
    };
    //add more  ?
}
#endif //__SERIALIZATION_H_H__
