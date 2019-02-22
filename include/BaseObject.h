//
//  BaseObject.h
//  Monocular_Measure
//
//  Created by TuLigen on 2019/2/18.
//

#ifndef __BASEOBJECT_H_H__
#define __BASEOBJECT_H_H__
namespace Monocular{
    /*
     * 基础类
     */
    class BaseObject
    {
    public:
        BaseObject():usecnt(1){}
        virtual ~BaseObject(){}
        
        void addRef()
        {
            ++usecnt;
        }
        
        void release()
        {
            if(--usecnt == 0)
            {
                delete this;
            }
        }
    private:
        int usecnt;
    };
}

#endif /* __BASEOBJECT_H_H__ */
