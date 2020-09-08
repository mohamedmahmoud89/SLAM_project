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
	//vector<SmartPtrVec<PoseLandmark>> map;
public:
        FastSlamPF()=delete;
        FastSlamPF(
                const SmrtPtrVec<PoseBase>& ps,
                const f32 control_motion,
                const f32 control_turn,
                const f32 meas_dist_stddev,
                const f32 meas_ang_stddev):
                ParticleFilter(ps,control_motion,control_turn,meas_dist_stddev,meas_ang_stddev){}
        void Update(
                FeatList& feats,
                const Robot::Config& cfg);
private:
        vector<f32> Calc_ImpWeights(
                        FeatList& feats,
                        const Robot::Config& cfg);
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

