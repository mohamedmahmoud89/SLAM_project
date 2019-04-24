#include "common.h"
#include "robot_cfg.h"
#include "ctrl_data.h"
#include<memory>
using namespace Robot;

namespace Motion{
	class MotionModel{
	public:
        	MotionModel(){}
        	MotionModel(const MotionModel &rhs)=delete;
        	MotionModel& operator=(const MotionModel& rhs)=delete;
        	MotionModel(MotionModel&& rhs)=delete;
        	MotionModel& operator=(MotionModel&& rhs)=delete;
        	virtual ~MotionModel(){}
		virtual void UpdatePos(
				unique_ptr<PoseBase>& pos,
				const ControlBase& c,
				const Config& cfg) const=0;
	};

	class MMSimple:public MotionModel{
	public:
		virtual ~MMSimple(){}
		virtual void UpdatePos(
				unique_ptr<PoseBase>& pos,
				const ControlBase& c,
				const Config& cfg) const;
	private:
		void UpdateS(
			unique_ptr<PoseBase>& pos,
			const ControlBase& t,
			const RobotConfig& cfg)const;
		void UpdateC(
			unique_ptr<PoseBase>& pos,
			const ControlBase& t,
			const RobotConfig& cfg)const;
	};
};
