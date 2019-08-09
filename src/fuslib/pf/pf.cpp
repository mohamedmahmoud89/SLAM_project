#include "pf.h"
#include <random>
#include <cassert>

using namespace std;

PfOutput::PfOutput(
		const SmrtPtrVec<PoseBase>& pars,
                const RobotConfig& in_cfg){
	cfg=make_unique<RobotConfig>(in_cfg);
	for(auto& i:pars){
		particles.push_back(make_unique<PoseBase>(*i));
	}	
}

PfOutput::PfOutput(const PfOutput& rhs){
	cfg=make_unique<RobotConfig>(*rhs.cfg);
	for(auto& i:rhs.particles){
                particles.push_back(make_unique<PoseBase>(*i));
        }
}

pair<f32,f32> ParticleFilter::Compute_SigmaCtrl(
                        const ControlBase& ctrl,
                        const Robot::Config& cfg){
	f32 t2mm(cfg.Ticks_ToMm());
        f32 r(ctrl.Right_Tick()*t2mm);
        f32 l(ctrl.Left_Tick() *t2mm);
        f32 motion_var_l(pow(ctrl_motion*l,2)+
                         pow(ctrl_turn*(l-r),2));
        f32 motion_var_r(pow(ctrl_motion*r,2)+
                         pow(ctrl_turn*(l-r),2));
	return make_pair<f32,f32>(
			sqrt(motion_var_r),
			sqrt(motion_var_l));
}

vector<f32> ParticleFilter::Calc_ImpWeights(
                        FeatList& feats,
                        RefList& refs,
                        const Robot::Config& cfg){
	vector<f32>ret;
	const f32 max_ref_dist(numeric_limits<f32>::max());
	const u8 num_landmarks(6);
	for(auto& pp:particles){
		const PoseBase particle(*pp);
		for(auto&feat:feats.Data()){
                        // transform the features to world coords
                        // based on the robot pos
                        FeatureGlobalTransform(
                                        *feat,particle,cfg);
                }
		unique_ptr<FeatAssoc> assocs(
                        FeatAssociate(feats.Data(),refs,max_ref_dist));
		assert(assocs->assocs_t.size()==num_landmarks);
		f32 weight(1);
		for(auto&assoc:assocs->assocs_t){
			auto p_ref=assocs->stored_t.find(
					assoc.second)->second;
			auto p_meas=assocs->scanned_t.find(
					assoc.first)->second;

			FeaturePolarTransform(
					*p_ref,particle,cfg);

			// sample pdf using polar coords diff

		}
		ret.push_back(weight);
	}
	return ret;
}

void ParticleFilter::Resample(const vector<f32>& weights){

}

void ParticleFilter::Predict(
		const ControlBase& ctrl,
		const Robot::Config& cfg){
	unique_ptr<MMSimple>p_motion(make_unique<MMSimple>());
	pair<f32,f32> ctrl_std(
			Compute_SigmaCtrl(ctrl,cfg));
	random_device dev;
	default_random_engine gen(dev());
	normal_distribution<f32> dist_l(
			ctrl.Left_Tick(),ctrl_std.second);
	normal_distribution<f32> dist_r(
			ctrl.Right_Tick(),ctrl_std.first);
	
	for(auto&i:particles){
		f32 l(dist_l(gen));
		f32 r(dist_r(gen));
		ControlBase sampled_ctrl(l,r);
		PoseBase temp(i->X(),i->Y(),i->Yaw());
		p_motion->UpdatePos(temp,sampled_ctrl,cfg);
		i->set_x(temp.X());
		i->set_y(temp.Y());
		i->set_yaw(temp.Yaw());
	}
}

void ParticleFilter::Update(
		FeatList& feats,
		RefList& refs,
		const Robot::Config& cfg){
	auto weights(Calc_ImpWeights(feats,refs,cfg));
	Resample(weights);
}
