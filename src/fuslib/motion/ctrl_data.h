#ifndef FUS_CTRL_DATA_H
#define FUS_CTRL_DATA_H
#include"common.h"

class ControlBase{
        u32 l{0};
        u32 r{0};
public:
        ControlBase(const u32 in_l,const u32 in_r):l(in_l),r(in_r){}
        virtual u32 Left_Tick() const
        {
                return l;
        }
        virtual u32 Right_Tick() const
        {
                return r;
        }
};
#endif
