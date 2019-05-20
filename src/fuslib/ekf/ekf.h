#include "common.h"
#include "feat.h"
#include "gaus.h"
#include "motion_model.h"
#include<memory>
#include "ctrl_data.h"

using namespace std;
using namespace Motion;
using namespace Feature;
class Ekf{
	Gaussian<PoseBase>belief;
public:
	Ekf()=delete;
	Ekf(const PoseBase& init_state,const Matrix<f32>& init_covar):
		belief(Gaussian<PoseBase>(init_state,init_covar)){}
	void Predict(
		const ControlBase& ctrl,
		const Robot::Config& cfg);
	void Update(const FeatAssoc& assocs);
	const Gaussian<PoseBase>& Belief() const
	{
		return belief;
	}
};
