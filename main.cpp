#include <iostream>
#include <System.h>
#include <ctime>
#include "Viewer.h"
#include "Functions.h"
#if defined(_WIN32) | defined(_WIN64)
#include <io.h>
#else
#include <dirent.h>
#endif
#include <string.h>
using namespace std;

#define  TEST_SAMPLE     0 //定位测试例子


#define  TEST_TRAJECTORY 1 // 测试轨迹

extern void measureTest();
extern void trackTest();

int main(void)
{
    
#if TEST_TRAJECTORY
    
    trackTest();
    
#else
    
    measureTest();
    
#endif
    
    return 0;
}


void measureTest()
{
    Monocular::Camera cam;
#if TEST_SAMPLE
    cam.K = (Mat_<double>(3,3) <<  1.8144486313396042e+03,0  ,2.0521093136805948e+03,
             0, 1.8144486313396042e+03, 1.0898609600712157e+03,
             0, 0, 1);
    
    cam.principal_point  =  Point2d( 2.0521093136805948e+03,1.0898609600712157e+03);//光心坐标
    cam.focal_length     =  1.8144486313396042e+03;                                 //焦距
    
    Monocular::System sys(cam,Monocular::eOpticalFlowMode,std::string(SAVEPATH) +  std::string("/result.txt") );
    
    const size_t Len = 2;
    
    Mat imgs[Len] = {imread("../../data/4014.jpg"),imread("../../data/4015.jpg")
    };
    
    Monocular::GeoPos geopts[Len] = { Monocular::GeoPos(114.40327060991, 30.60129500574),
        Monocular::GeoPos(114.40331230623, 30.60129672252)
    };
#else
    cam.K = (Mat_<double>(3,3) <<  1.8144486313396042e+03,0., 2.0521093136805948e+03,
             0., 1.8144486313396042e+03, 1.0898609600712157e+03,
             0. ,0. ,1.);
    
    cam.principal_point  =  Point2d( 2.0521093136805948e+03 ,1.0898609600712157e+03);//光心坐标
    cam.focal_length     =  1.8144486313396042e+03;                                 //焦距
    
    Monocular::System sys(cam,Monocular::eFeaturesMode,std::string(SAVEPATH) +  std::string("/result.txt") );
    
    const size_t Len = 2;
    
    Mat imgs[Len] = {imread("../../data/132_L.jpg"),imread("../../data/133_L.jpg"),
    };
    Monocular::GeoPos geopts[Len] = { Monocular::GeoPos(121.46786057377388, 31.210303875854922),
        Monocular::GeoPos(121.46782325637565, 31.210289138720274)};
    
#endif
    
    time_t cur = clock();
    
    for(size_t i = 0 ;  i < Len; ++i)
    {
        Monocular::TargetItems items = sys.objectDetect(imgs[i],TEST_SAMPLE);
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
    
    time_t dt = clock() - cur;
    
    std::cout << "spend time: " << dt/(float)CLOCKS_PER_SEC << std::endl;
}

#if defined(_WIN32) | defined(_WIN64)
void dir(string path) {
    long hFile = 0;
    struct _finddata_t fileInfo;
    string pathName, exdName;
    // \\* 代表要遍历所有的类型,如改成\\*.jpg表示遍历jpg类型文件
    if ((hFile = _findfirst(pathName.assign(path).append("\\*").c_str(), &fileInfo)) == -1)
    {
        return;
    }
    do { //判断文件的属性是文件夹还是文件
        cout << fileInfo.name << (fileInfo.attrib&_A_SUBDIR ? "[folder]" : "[file]") << endl; }
    while (_findnext(hFile, &fileInfo) == 0)
        ;
    _findclose(hFile);
}



#else

inline bool isPicSuffix(const char *pName,size_t len)
{
    const size_t suffix = 3;
    assert(pName);
    if(len < suffix)
    {
        return false;
    }
    const char *pSuffix =  &pName[len - suffix];
    
    return !strcasecmp(pSuffix, "jpg") | !strcasecmp(pSuffix, "png");
}
int loadFiles( const std::string &dirpath, std::vector<std::string> &files )
{
    DIR *dp;
    struct dirent *dirp;
    if((dp = opendir(dirpath.c_str())) == NULL)
    {
        assert(NULL);
    }
    int index = 0;
    while((dirp = readdir(dp)) != NULL)
    {
        if(isPicSuffix(dirp->d_name,dirp->d_namlen))
        {
            std::string filepath(dirpath);
            filepath.append("/");
            filepath.append(dirp->d_name);
            files.emplace_back(filepath);
            ++index;
        }
    }
    closedir(dp);
    
    return index;
}
#endif




void trackTest()
{
    std::vector<std::string>  files;
    //read files
    int fileSize = loadFiles("/Volumes/mac/Data/weiya",files);
    if( fileSize < 1 )
    {
        cout << " This path has no files!Please change other path." << endl;
        return ;
    }
    Monocular::Camera cam;
    
    cam.K = (Mat_<double>(3,3) <<  2.3695365586649123e+03,0  ,2.0443736115794320e+03,
             0, 2.3695365586649123e+03, 1.0750972331437883e+03,
             0, 0, 1);
    
    cam.principal_point  =  Point2d( 2.0443736115794320e+03, 1.0750972331437883e+03);//光心坐标
    cam.focal_length     =  2.3695365586649123e+03;                                 //焦距
    
    
    Monocular::Viewer *pViewer = new Monocular::DefaultViewer;
    
    Monocular::System sys(cam,Monocular::eFeaturesMode,pViewer );
    
    Monocular::TimeInterval timeClac;
   
    for(int i = 0;i < fileSize ;++i)
    {
        timeClac.start();
        cout << files[i].c_str() << endl;
        Mat img = imread(files[i].c_str());
        assert(!img.empty());
        Monocular::GeoPos pos;
        sys.handle(img, pos);
        timeClac.print("total");
    }
    waitKey(0);
}



