#include "robot_cfg.h"
const f32 RobotConfig::mount_angle=-0.06981317007977318;
const f32 RobotConfig::offset=0.006135923151543;
f32 RobotConfig::Ray_IdxToAng(const u16 idx){
	f32 ret(((idx-330)*offset)+mount_angle);
	return ret;
}