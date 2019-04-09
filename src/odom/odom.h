#include"common.h"
#include"robot_cfg.h"
#include<memory>
using namespace std;

class Odom{
	unique_ptr<Pose>p_pos;
public:
	Odom():
		p_pos(make_unique<Pose>()){}
	Odom(const Pose& p):
		p_pos(make_unique<Pose>(p)){}
	Odom(const Odom& rhs)=delete;
	Odom& operator=(const Odom& rhs)=delete;
	Odom(Odom&& rhs)=delete;
	Odom& operator=(Odom&& rhs)=delete;
	void UpdatePos(const Tick& t,const RobotConfig& cfg);
	const Pose get_pos() const noexcept{
		return *p_pos;	
	}
private:
	void UpdateS(const Tick& t,const RobotConfig& cfg);
	void UpdateC(const Tick& t,const RobotConfig& cfg);
};
