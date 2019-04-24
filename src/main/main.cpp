#include"file_mgr.h"
#include<algorithm>
#include<iostream>
#include<memory>
#include"odom.h"
#include<cmath>
#include"lfx.h"
using namespace std;

int main(){
	//motor data definitions
	MotorFileMgr* pmfm=MotorFileMgr::get_instance();
        unique_ptr<Odom>odo=make_unique<Odom>(
			PoseBase(1850.0,1897.0,(213.0/180)*M_PI));
	unique_ptr<PosFileMgr> pfm=make_unique<PosFileMgr>();
	unique_ptr<RobotConfig> cfg=
		make_unique<RobotConfig>(150.0,30.0,0.349);
	//scan data definitions
	ScanFileMgr* psfm(ScanFileMgr::get_instance());
	unique_ptr<Scan::ScanConfig>s_cfg(
		make_unique<Scan::ScanConfig>(20,100,90));
	unique_ptr<LidarFeatExBase>plfx(
		make_unique<LidarFeatExBase>());
	unique_ptr<FeatFileMgr>ffm(make_unique<FeatFileMgr>());
	
	// motor data
	pmfm->read("../data/robot4_motors.txt");
        vector<ControlBase> v(pmfm->get_data());
	for(auto& t:v){
		odo->Update(t,*cfg);
		pfm->add_sample(odo->get_pos());
	}
		
	pfm->write("../data/poses_from_ticks.txt");
	
	//scan data
	psfm->read("../data/robot4_scan.txt");
	vector<Scan::Scan> scans(psfm->get_data());
	vector<Feature::FeatList>vfl;
	for(auto&i:scans){
		Feature::FeatList vf(plfx->Feature_Extract(
				i,*s_cfg));
		ffm->add_sample(vf);
	}	
	ffm->write("../data/cylinders.txt");
	return 0;
}

