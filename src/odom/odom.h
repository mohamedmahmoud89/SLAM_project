#include"common.h"
#include<memory>
using namespace std;

class odom{
	unique_ptr<pose>p_pos;
public:
	odom():
		p_pos(make_unique<pose>()){}
	odom(const pose& p):
		p_pos(make_unique<pose>(p)){}
	void UpdatePos(const tick& t,const robot_config& cfg);
	const pose get_pos() const noexcept{
		return *p_pos;	
	}
private:
	void UpdateS(const tick& t,const robot_config& cfg);
	void UpdateC(const tick& t,const robot_config& cfg);
};
