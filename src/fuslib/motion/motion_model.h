#include "common.h"
#include "robot_cfg.h"

using namespace Robot;

namespace Motion{
	class MotionModel{
	public:
        	MotionModel(){}
        	MotionModel(const MotionModel &rhs)=delete;
        	MotionModel& operator=(const MotionModel& rhs)=delete;
        	MotionModel(MotionModel&& rhs)=delete;
        	MotionModel& operator=(MotionModel&& rhs)=delete;
        	virtual void UpdatePos(
				const ControlBase& c,
				const Config& cfg)=0;
	};
};
