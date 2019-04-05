#ifndef COMMON_H
#define COMMON_H
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
class Tick{ 
        u32 l{0};
        u32 r{0};
public:
        Tick(const u32 in_l,const u32 in_r):l(in_l),r(in_r){}
        u32 Left() const
        {
                return l;
        }
        u32 Right() const
        {
                return r;
        }
};

class Pose{
	f32 x{0.0};
	f32 y{0.0};
	f32 yaw{0.0};
public:
	Pose():x(0.0),y(0.0),yaw(0.0){}
	Pose(f32 in_x,f32 in_y,f32 in_yaw):
		x(in_x),y(in_y),yaw(in_yaw){}
	f32 X() const noexcept{
		return x;
	}
	void set_x(const f32& in) noexcept{
		x=in;
	}
	void set_y(const f32& in) noexcept{
		y=in;
	}
	void set_yaw(const f32& in) noexcept{
		yaw=in;
	}
	f32 Y() const noexcept{
		return y;
	}
	f32 Yaw() const noexcept{
		return yaw;
	}
};

class Feature{
	f32 x{0};
	f32 y{0};
public:
	Feature():x(0),y(0){}
	Feature(const f32 x_mm,const f32 y_mm):x(x_mm),y(y_mm){}
	f32 X(){return x;}
	f32 Y(){return y;}
};

class robot_config{
	f32 width_mm{0.0};
	f32 sensor_offset_mm{0.0};
	f32 Ticks_to_mm{0.0};
public:
	robot_config():
		width_mm(0.0),sensor_offset_mm(0.0),Ticks_to_mm(0.0){}
	robot_config(const f32& w,const f32& o,const f32& t):
		width_mm(w),sensor_offset_mm(o),Ticks_to_mm(t){}
	f32 get_width() const noexcept{
		return width_mm;
	}
	f32 get_offset() const noexcept{
		return sensor_offset_mm;
	}
	f32 get_Ticks_to_mm() const noexcept{
		return Ticks_to_mm;
	}
};
#endif
