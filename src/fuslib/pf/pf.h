#ifndef PF_H
#define PF_H
#include "common.h"
#include "motion_model.h"
#include "ctrl_data.h"
#include <vector>

using namespace std;
using namespace Motion;

class ParticleFilter{
	SmrtPtrVec<PoseBase> particles;
	f32 ctrl_motion;
	f32 ctrl_turn;
	f32 meas_dist_std;
	f32 meas_ang_std;
public:	
	ParticleFilter()=delete;
	ParticleFilter(
		const SmrtPtrVec<PoseBase>& ps,
		const f32 control_motion,
		const f32 control_turn,
		const f32 meas_dist_stddev,
		const f32 meas_ang_stddev):
		particles(ps),
		ctrl_motion(control_motion),
		ctrl_turn(control_turn),
		meas_dist_std(meas_dist_stddev),
		meas_ang_std(meas_ang_stddev){}
	void Predict(
		const ControlBase& ctrl,
		const Robot::Config& cfg);
	SmrtPtrVec<PoseBase>& Particles(){return particles;}
};

class PfOutput{
public:
	ConstUnqPtrVec<PoseBase> particles;
	unique_ptr<const Robot::RobotConfig> cfg;
	PfOutput()=delete;
	PfOutput(
		const SmrtPtrVec<PoseBase>& pars,
		const RobotConfig& cfg);
	PfOutput(const PfOutput& rhs);
	PfOutput(PfOutput&& rhs):
		particles(move(rhs.particles)),
		cfg(move(rhs.cfg)){}
	
};
#endif
