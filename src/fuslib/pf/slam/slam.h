#ifndef SLAM_H
#define SLAM_H
#include "common.h"
#include "motion_model.h"
#include "ctrl_data.h"
#include "feat.h"
#include "pf.h"
#include <vector>

using namespace std;
using namespace Motion;
using namespace Feature;

class FastSlamPF : public ParticleFilter{
	// A list of the landmarks associated to each particle
	vector<SmrtPtrVec<Gaussian<PoseBase>>> map;
	f32 min_likelihood;
	f32 meas_dst_std;
	f32 meas_ang_std;
public:
        FastSlamPF()=delete;
        FastSlamPF(
                const SmrtPtrVec<PoseBase>& ps,
                const f32 control_motion,
                const f32 control_turn,
                const f32 meas_dist_stddev,
                const f32 meas_ang_stddev,
		const f32 min_like):
                ParticleFilter(ps,control_motion,control_turn,meas_dist_stddev,meas_ang_stddev),
		map(ps.size(),SmrtPtrVec<Gaussian<PoseBase>>(0)),
		min_likelihood(min_like),
		meas_dst_std(meas_dist_stddev),
		meas_ang_std(meas_ang_stddev){}
        void Update(
                FeatList& feats,
                const Robot::Config& cfg);
private:
        vector<f32> Calc_ImpWeights(
                        FeatList& feats,
                        const Robot::Config& cfg);
	void Update_Landmark(
			const size_t particle_idx,
			const shared_ptr<FeatBase>& feature,
			int landmark_idx,
			const Robot::Config& cfg);
	void Insert_Landmark(
			const size_t particle_idx,
			const shared_ptr<FeatBase>& feature,
			const Robot::Config& cfg);
	MatrixXf Compute_H(
                        const PoseBase& pos,
                        const FeatBase& landmark,
                        const Robot::Config& cfg);
        MatrixXf Compute_Q();
};

class FastSlamOutput : public PfOutput{
public:
	FastSlamOutput()=delete;
	FastSlamOutput(
			const SmrtPtrVec<PoseBase>& pars,
			const RobotConfig& cfg):PfOutput(pars,cfg){}
	FastSlamOutput(const FastSlamOutput& rhs):PfOutput(rhs){}
	FastSlamOutput(FastSlamOutput&& rhs):PfOutput(move(rhs)){}
};
#endif

