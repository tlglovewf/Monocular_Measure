#include <iostream>
#include <System.h>
using namespace std;

int main(void)
{
    
    Monocular::Camera cam;
    cam.K = (Mat_<double>(3,3) <<  1.8144486313396042e+03,0  ,2.0521093136805948e+03,
             0, 1.8144486313396042e+03, 1.0898609600712157e+03,
             0, 0, 1);
    
    cam.principal_point  =  Point2d( 2.0521093136805948e+03,1.0898609600712157e+03);//光心坐标
    cam.focal_length     =  1.8144486313396042e+03;                                 //焦距
    
    Monocular::System sys(cam,Monocular::eFeaturesMode);
    
    const size_t Len = 2;
    Mat imgs[Len] = {imread("../../data/4014.jpg"),imread("../../data/4015.jpg")};
    Monocular::GeoPos geopts[Len] = { Monocular::GeoPos(114.40327060991, 30.60129500574),
                                      Monocular::GeoPos(114.40331230623, 30.60129672252)};
    
    for(size_t i = 0 ;  i < Len; ++i)
    {
        Monocular::TargetItems items = sys.objectDetect(imgs[i]);
        
        if(!items.empty())
        {//检测到目标
            //处理
            sys.handle(imgs[i],geopts[i],items);
        }
        else
        {
            sys.reset();
        }
    }
    sys.printResult();
    return 0;
}

