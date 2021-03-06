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
	f32 meas_dist_std;
	f32 meas_ang_std;
public:
	Ekf()=delete;
	Ekf(
		const u8 st_space,
		const PoseBase& init_state,
		const MatrixXf& init_covar,
		const f32 control_motion,
		const f32 control_turn,
		const f32 meas_dist_stddev,
		const f32 meas_ang_stddev):
		state_space(st_space),
		belief(Gaussian<PoseBase>(init_state,init_covar)),
		ctrl_motion(control_motion),
		ctrl_turn(control_turn),
		meas_dist_std(meas_dist_stddev),
		meas_ang_std(meas_ang_stddev){}
	void Predict(
		const ControlBase& ctrl,
		const Robot::Config& cfg);
	void Update(
		const FeatAssoc& assocs,
		const Robot::Config& cfg);
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
	
	shared_ptr<MatrixXf> Compute_H(
			const PoseBase& pos,
			const FeatBase& ref,
			const Robot::Config& cfg);
	shared_ptr<MatrixXf> Compute_Q();	
};

class EkfOutput{
        Gaussian<PoseBase> belief;
        Robot::RobotConfig cfg;
	vector<FeatBase> refs;
public:  
	EkfOutput()=delete;
	EkfOutput(
                        const Gaussian<PoseBase>&b,
                        const Robot::RobotConfig& c,
			const vector<FeatBase>&r):
                belief(b),
                cfg(c),
		refs(r){}
	tuple<f32,f32,f32,f32> Std() const;
	unique_ptr<PoseBase> Pos() const;
	vector<FeatBase> Refs() const{return refs;}
};
#endif
