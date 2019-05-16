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
	unique_ptr<MMSimple>mm;
	Gaussian<PoseBase>belief;
public:
	Ekf()=delete;
	Ekf(const u8 state_space):
		belief(Gaussian<PoseBase>(state_space)),
		mm(make_unique<MMSimple>()){}
	void Predict(const ControlBase& ctrl);
	void Update(const FeatAssoc& assocs);
	const Gaussian<PoseBase>& Belief() const
	{
		return belief;
	}
};
