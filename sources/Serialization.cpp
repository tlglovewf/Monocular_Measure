//add more ...
#include "Serialization.h"
#include "assert.h"
#include "Frame.h"
#include "Functions.h"
#include <iomanip>
namespace Monocular {
    //格式输出
#define WRITE_INFO(os,prompt,x)  os << prompt << ":" << (x) << std::endl;
    
    FileSerialization::FileSerialization(const std::string &path):mPath(path)
    {
        assert(!path.empty());
        this->open(path);
        this->setf(std::ios::left);
        this->width(25);
        this->precision(25);
    }
    
    void FileSerialization::writeFormat(const std::string &prompt,const GeoPos &geopos)
    {
        assert(is_open());
        *this  << std::setprecision(20) << prompt.c_str()\
        << ":" << geopos.y << "," << geopos.x << std::endl;
    }
    
    /*
     * 写经纬度
     * @param prompt 标签
     * @param value  float值
     */
    void FileSerialization::writeFormat(const std::string &prompt,float value)
    {
        assert(is_open());
        WRITE_INFO(*this,prompt.c_str(),value);
    }
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param mat    矩阵
     */
    void FileSerialization::writeFormat(const std::string &prompt,const Mat &mat)
    {
        assert(is_open());
        *this << prompt.c_str() << std::endl << mat << std::endl;
    }
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param rect   边框
     */
    void FileSerialization::writeFormat(const std::string &prompt,const Rect2f &rect)
    {
        assert(is_open());
        WRITE_INFO(*this,prompt.c_str(),rect);
    }
    /*
     * 打印字符
     */
    void FileSerialization::prompt(const std::string &prompt)
    {
        assert(is_open());
        *this << prompt.c_str() << ":" << std::endl;
    }
    
    /*
     * 格式化写值
     * @param prompt 标签
     * @param point  点
     */
    void FileSerialization::writeFormat(const std::string &prompt,const Point2f &pt)
    {
        assert(is_open());
        *this << std::left << std::setw(15) << prompt.c_str() << " : " << pt << std::endl;
    }
    
    void FileSerialization::serialize(const Monocular::Frame &f)
    {
        if(is_open())
        {
            //输出基本信息
            const GeoPos &geo = f.getPosition();
            writeFormat("pos", geo);//当前帧经纬度
            
            const TargetItems &items = (const_cast<Monocular::Frame&>(f)).getTargetItems();
            

            //输出目标信息
            for(const TargetItem &item :items)
            {
                WRITE_INFO(*this,"id",item._id)
//                WRITE_INFO(item._box)
                WRITE_INFO(*this,"center",item._center);
                WRITE_INFO(*this,"world pos",item._pos);
                
#if TESTOUTPUT
                if(CHECKVALUE(item._realpos))
                {
                    WRITE_INFO(*this,"accuracy",Functions::ComputeDistance(item._pos, item._realpos));
                }
#endif
                
            }

        }
    }
}
