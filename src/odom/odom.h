#ifndef ODOM_H
#define ODOM_H
#include"common.h"
#include"robot_cfg.h"
#include "motion_model.h"
#include<memory>
using namespace std;
using namespace Motion;
class Odom : public MMSimple{
	unique_ptr<PoseBase>p_pos;
public:
	Odom():
		MMSimple(),p_pos(make_unique<PoseBase>()){}
	Odom(const PoseBase& p):
		MMSimple(),p_pos(make_unique<PoseBase>(p)){}
	~Odom(){}
	/*void UpdatePos(unique_ptr<PoseBase>& pos,
			const ControlBase& c,
			const Robot::Config& cfg) override{
		MMSimple::UpdatePos(p_pos,c,cfg);
	}*/
	void Update(
		const ControlBase& c,
		const Robot::Config& cfg)
	{
		MMSimple::UpdatePos(p_pos,c,cfg);
	}
	const PoseBase& get_pos() const noexcept{
		return *p_pos;	
	}
};
#endif
