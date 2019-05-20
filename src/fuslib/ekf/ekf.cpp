#include "ekf.h"

using namespace std;

void Ekf::Predict(
		const ControlBase& ctrl,
		const Robot::Config& cfg){
	unique_ptr<MMSimple>p_motion(make_unique<MMSimple>());
	// g()
	shared_ptr<PoseBase>p_pos(belief.Mean());
	p_motion->UpdatePos(*p_pos,ctrl,cfg);
}

void Ekf::Update(const FeatAssoc& assocs){

}
