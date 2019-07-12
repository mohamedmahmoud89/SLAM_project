#ifndef EKF_H
#define EKF_H
#include "common.h"
#include "feat.h"
#include "gaus.h"
#include "motion_model.h"
#include<memory>
#include "ctrl_data.h"

using namespace std;
using namespace Motion;
using namespace Feature;
using namespace Eigen;
class Ekf{
	Gaussian<PoseBase>belief;
	u8 state_space;
	f32 ctrl_motion;
	f32 ctrl_turn;
public:
	Ekf()=delete;
	Ekf(
		const u8 st_space,
		const PoseBase& init_state,
		const MatrixXf& init_covar,
		const f32 control_motion,
		const f32 control_turn):
		state_space(st_space),
		belief(Gaussian<PoseBase>(init_state,init_covar)),
		ctrl_motion(control_motion),
		ctrl_turn(control_turn){}
	void Predict(
		const ControlBase& ctrl,
		const Robot::Config& cfg);
	void Update(const FeatAssoc& assocs);
	const Gaussian<PoseBase>& Belief() const
	{
		return belief;
	}
private:
	shared_ptr<MatrixXf> Compute_G(
			const PoseBase& pos,
			const ControlBase& ctrl,
			const Robot::Config& cfg);
	
	shared_ptr<MatrixXf> Compute_G_linear(
			const PoseBase& pos,
			const ControlBase& ctrl,
			const Robot::Config& cfg);
	
	shared_ptr<MatrixXf> Compute_G_non_linear(
			const PoseBase& pos,
			const ControlBase& ctrl,
			const Robot::Config& cfg);
	
	shared_ptr<MatrixXf> Compute_V(
			const PoseBase& pos,
			const ControlBase& ctrl,
			const Robot::Config& cfg);
	
	shared_ptr<MatrixXf> Compute_SigmaCtrl(
			const ControlBase& ctrl,
			const Robot::Config& cfg);
	
	//shared_ptr<MatrixXf> Compute_V();	
};

class EkfOutput{
        Gaussian<PoseBase> belief;
        Robot::RobotConfig cfg;
public:  
	EkfOutput()=delete;
	EkfOutput(
                        const Gaussian<PoseBase>&b,
                        const Robot::RobotConfig& c):
                belief(b),
                cfg(c){}
	tuple<f32,f32,f32,f32> Std() const;
	unique_ptr<PoseBase> Pos() const;
};
#endif
