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

class PoseBase{
protected:
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

#endif
