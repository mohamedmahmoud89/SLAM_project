#include"common.h"
#include<memory>
using namespace std;

class odom{
	unique_ptr<Pose>p_pos;
public:
	odom():
		p_pos(make_unique<Pose>()){}
	odom(const Pose& p):
		p_pos(make_unique<Pose>(p)){}
	void UpdatePos(const Tick& t,const robot_config& cfg);
	const Pose get_pos() const noexcept{
		return *p_pos;	
	}
private:
	void UpdateS(const Tick& t,const robot_config& cfg);
	void UpdateC(const Tick& t,const robot_config& cfg);
};
