#include"file_mgr.h"
#include<memory>
#include"odom.h"
#include<cmath>
#include"lfx.h"
#include"feat.h"
using namespace std;
//TODO:
//1-read the reference cylinders data and fill a FeatList of em
//2-transform the feature list vf to the world coords
//3-associate both together
void Test_Klmn(){
        //motor data definitions
        MotorFileMgr* pmfm=MotorFileMgr::get_instance();
        unique_ptr<Odom>odo=make_unique<Odom>(
                        PoseBase(1850.0,1897.0,(213.0/180)*M_PI));
        unique_ptr<RobotConfig> cfg=
                make_unique<RobotConfig>(150.0,30.0,0.349);
	//scan data definitions
        ScanFileMgr* psfm(ScanFileMgr::get_instance());
        unique_ptr<Scan::ScanConfig>s_cfg(
                make_unique<Scan::ScanConfig>(20,100,90));
        unique_ptr<LidarFeatExBase>plfx(
                make_unique<LidarFeatExBase>());
	//ref landmark data definitions
	RefLandmarkFileMgr* prfm(RefLandmarkFileMgr::get_instance());

		
        // read data
        pmfm->read("../data/robot4_motors.txt");
        psfm->read("../data/robot4_scan.txt");
        prfm->read("../data/robot_arena_landmarks.txt");
	vector<ControlBase> ticks(pmfm->get_data());
        vector<Scan::Scan> scans(psfm->get_data());
	vector<Feature::FeatBase> refs(prfm->get_data());
        for(int i=0;i<ticks.size();++i){
                //predict
		odo->Update(ticks[i],*cfg);

		//associate
                Feature::FeatList vf(plfx->Feature_Extract(
                                scans[i],*s_cfg));
		for(auto&i:vf.Data().List()){
			FeatureTransform(*i,odo->get_pos());
		}
        }
}

