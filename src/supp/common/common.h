#ifndef COMMON_H
#define COMMON_H
#include<vector>
using namespace std;

using u8=unsigned char;
using u16=unsigned short;
using u32=unsigned long;
using si8=char;
using si16=short;
using si32=long;
using f32=float;
using f64=double;
// use the default copy/move Ctors
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

class PoseBase{
	f32 x{0.0};
	f32 y{0.0};
	f32 yaw{0.0};
public:
	PoseBase():x(0.0),y(0.0),yaw(0.0){}
	PoseBase(f32 in_x,f32 in_y,f32 in_yaw):
		x(in_x),y(in_y),yaw(in_yaw){}
	virtual f32 X() const noexcept{
		return x;
	}
	virtual void set_x(const f32& in) noexcept{
		x=in;
	}
	virtual void set_y(const f32& in) noexcept{
		y=in;
	}
	virtual void set_yaw(const f32& in) noexcept{
		yaw=in;
	}
	virtual f32 Y() const noexcept{
		return y;
	}
	virtual f32 Yaw() const noexcept{
		return yaw;
	}
};

class FeatBase{
	f32 x{0};
	f32 y{0};
public:
	FeatBase():x(0),y(0){}
	FeatBase(const f32 x_mm,const f32 y_mm):x(x_mm),y(y_mm){}
	virtual f32 X() const{return x;}
	virtual f32 Y() const{return y;}
};
#endif
