#include"file_mgr.h"
#include<algorithm>
#include<iostream>
#include<memory>
#include"odom.h"
#include<cmath>
using namespace std;

int main(){
	MotorFileMgr* pmfm=MotorFileMgr::get_instance();
        unique_ptr<odom>odo=make_unique<odom>(
			pose(1850.0,1897.0,(213.0/180)*M_PI));
	unique_ptr<PosFileMgr> pfm=make_unique<PosFileMgr>();
	unique_ptr<robot_config> cfg=
		make_unique<robot_config>(150.0,30.0,0.349);

	pmfm->read("../data/robot4_motors.txt");
        vector<tick> v(pmfm->get_data());
	for(auto&tick:v){
		odo->UpdatePos(tick,*cfg);
		pfm->add_sample(odo->get_pos());
	}
		
	pfm->write("../data/poses_from_ticks.txt");
	return 0;
}

