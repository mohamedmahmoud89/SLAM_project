#include"file_mgr.h"
#include<memory>
#include"odom.h"
#include<cmath>
#include"lfx.h"
#include"feat.h"
#include"test.h"
using namespace std;

//TODO:
//implement the prediction step
void Test::Test_Klmn(){
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
        SmrtPtrVec<Feature::FeatBase> refs(prfm->get_data());
        for(int i=0;i<ticks.size();++i){
                //predict
                odo->Update(ticks[i],*cfg);

                //associate
                FeatList vf(plfx->Feature_Extract(
                                scans[i],*s_cfg));
                for(auto&i:vf.Data()){
                        // transform the features to world coords
			// based on the robot pos
			FeatureTransform(*i,odo->get_pos(),*cfg);
                }
		unique_ptr<FeatAssoc> assocs(
			FeatAssociate(vf.Data(),refs));
        }
}

