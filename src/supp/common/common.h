#ifndef COMMON_H
#define COMMON_H
#include<vector>
#include<memory>
#include<cmath>
using namespace std;

using u8=unsigned char;
using u16=unsigned short;
using u32=unsigned long;
using si8=char;
using si16=short;
using si32=long;
using f32=float;
using f64=double;

template<typename T>
using SmrtPtrVec=vector<shared_ptr<T>>;

template<typename T>
using ConstUnqPtrVec=vector<unique_ptr<const T>>;

class PoseBase{
protected:
	f32 x{0.0};
	f32 y{0.0};
	f32 yaw{0.0};
public:
	PoseBase():x(0.0),y(0.0),yaw(0.0){}
	PoseBase(f32 in_x,f32 in_y,f32 in_yaw):
		x(in_x),y(in_y),yaw(in_yaw){}
	virtual ~PoseBase(){}
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

inline f32 normal_pdf(f32 x, f32 m, f32 s)
{
    static const f32 inv_sqrt_2pi = 0.3989422804014327;
    f32 a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
}
#endif
