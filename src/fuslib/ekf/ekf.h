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
public:
	Ekf()=delete;
	Ekf(
		const u8 st_space,
		const PoseBase& init_state,
		const MatrixXf& init_covar):
		state_space(st_space),
		belief(Gaussian<PoseBase>(init_state,init_covar)){}
	void Predict(
		const ControlBase& ctrl,
		const Robot::Config& cfg,
		const MatrixXf& motion_covariance);
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
	
	//shared_ptr<MatrixXf> Compute_V();	
};
