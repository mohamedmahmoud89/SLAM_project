#include "pf.h"
#include <random>

using namespace std;

void ParticleFilter::Predict(
		const ControlBase& ctrl,
		const Robot::Config& cfg){

}

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
