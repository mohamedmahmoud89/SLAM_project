#include"common.h"
#include"robot_cfg.h"
#include "motion_model.h"
#include<memory>
using namespace std;
using namespace Motion;
class Odom : public MotionModel{
	unique_ptr<PoseBase>p_pos;
public:
	Odom():
		MotionModel(),p_pos(make_unique<PoseBase>()){}
	Odom(const PoseBase& p):
		MotionModel(),p_pos(make_unique<PoseBase>(p)){}
	void UpdatePos(const ControlBase& c,
			const Robot::Config& cfg) override;
	const PoseBase get_pos() const noexcept{
		return *p_pos;	
	}
private:
	void UpdateS(const ControlBase& t,const RobotConfig& cfg);
	void UpdateC(const ControlBase& t,const RobotConfig& cfg);
};
