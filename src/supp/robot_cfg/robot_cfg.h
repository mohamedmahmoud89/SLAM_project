#include "common.h"

using namespace std;

class RobotConfig{
        f32 width_mm{0.0};
        f32 sensor_offset_mm{0.0};
        f32 ticks_to_mm{0.0};
	static const f32 mount_angle;
	static const f32 offset; 
public:
        RobotConfig():
                width_mm(0.0),sensor_offset_mm(0.0),ticks_to_mm(0.0){}
        RobotConfig(const f32& w,const f32& o,const f32& t):
                width_mm(w),sensor_offset_mm(o),ticks_to_mm(t){}
        f32 Width() const noexcept{
                return width_mm;
        }
        f32 Sensor_Offset() const noexcept{
                return sensor_offset_mm;
        }
        f32 Ticks_ToMm() const noexcept{
                return ticks_to_mm;
        }
	static f32 Ray_IdxToAng(const u16 idx);
};
