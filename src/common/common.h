#ifndef COMMON_H
#define COMMON_H
using namespace std;

using u8=unsigned short;
using u16=unsigned int;
using u32=unsigned long;
using si8=short;
using si16=int;
using si32=long;
// use the default copy/move Ctors
class tick{ 
        u32 l{0};
        u32 r{0};
public:
        tick(const u32 in_l,const u32 in_r):l(in_l),r(in_r){}
        u32 left() const
        {
                return l;
        }
        u32 right() const
        {
                return r;
        }
};

class pose{
	float x{0.0};
	float y{0.0};
	float yaw{0.0};
public:
	pose():x(0.0),y(0.0),yaw(0.0){}
	pose(float in_x,float in_y,float in_yaw):
		x(in_x),y(in_y),yaw(in_yaw){}
	float get_x() const noexcept{
		return x;
	}
	void set_x(const float& in) noexcept{
		x=in;
	}
	void set_y(const float& in) noexcept{
		y=in;
	}
	void set_yaw(const float& in) noexcept{
		yaw=in;
	}
	float get_y() const noexcept{
		return y;
	}
	float get_yaw() const noexcept{
		return yaw;
	}
};

class robot_config{
	float width_mm{0.0};
	float sensor_offset_mm{0.0};
	float ticks_to_mm{0.0};
public:
	robot_config():
		width_mm(0.0),sensor_offset_mm(0.0),ticks_to_mm(0.0){}
	robot_config(const float& w,const float& o,const float& t):
		width_mm(w),sensor_offset_mm(o),ticks_to_mm(t){}
	float get_width() const noexcept{
		return width_mm;
	}
	float get_offset() const noexcept{
		return sensor_offset_mm;
	}
	float get_ticks_to_mm() const noexcept{
		return ticks_to_mm;
	}
};
#endif
