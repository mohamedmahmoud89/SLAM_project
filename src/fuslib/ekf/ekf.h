#include "common.h"
#include "feat.h"
#include "gaus.h"
#include "motion_model.h"
#include<memory>
#include "ctrl_data.h"

using namespace std;
using namespace Motion;
using namespace Feature;
template<const u8 state_space>
class Ekf{
	unique_ptr<MotionModel>mm;
	Gaussian<PoseBase,state_space>belief;
public:
	void Predict(const ControlBase& ctrl);
	void Update(const FeatAssoc& assocs);
	const Gaussian<PoseBase,state_space>& Belief() const
	{
		return belief;
	}
};
