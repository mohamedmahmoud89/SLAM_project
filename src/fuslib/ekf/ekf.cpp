#include "ekf.h"

using namespace std;
using namespace Eigen;

shared_ptr<MatrixXf> Ekf::Compute_G(
		const ControlBase& ctrl,
		const Robot::Config& cfg){
	shared_ptr<MatrixXf> ret=
		make_shared<MatrixXf>(state_space,state_space);
	return ret;
}

void Ekf::Predict(
		const ControlBase& ctrl,
		const Robot::Config& cfg,
		const MatrixXf& motion_covar){
	unique_ptr<MMSimple>p_motion(make_unique<MMSimple>());
	// Mu=g(Mt,Ut)
	shared_ptr<PoseBase>p_pos(belief.Mean());
	p_motion->UpdatePos(*p_pos,ctrl,cfg);

	// Sigma=G*Sigma*GT+R
}

void Ekf::Update(const FeatAssoc& assocs){
	// K=Sigma*HT*(H*Sigma*HT+Q)⁻¹
	
	// Mu=Mu+K*(Z-h(Mu))
	
	// Sigma=(I-K*H)Sigma
}
