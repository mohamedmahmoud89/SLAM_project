#include"file_mgr.h"
#include<memory>
#include<cmath>
#include "lfx.h"
#include "feat.h"
#include "test.h"
#include "loc.h"
#include<random>

using namespace std;

void Test::Test_FastSlam(){
	//motor data definitions
        MotorFileMgr* pmfm=MotorFileMgr::get_instance();
        //scan data definitions
        ScanFileMgr* psfm(ScanFileMgr::get_instance());
        Scan::ScanConfig s_cfg(20,100,90);
        unique_ptr<LidarFeatExBase>plfx(
                make_unique<LidarFeatExBase>());

	// pf definitions
	RobotConfig cfg(155.0,30.0,0.349);
        f32 control_motion(0.35);
        f32 control_turn(0.6);
        f32 meas_dist_std(200);
        f32 meas_ang_std((15.0/180.0)*M_PI);
	u16 num_particles(25);
	SmrtPtrVec<PoseBase>vec;

	for(u16 i=0;i<num_particles;++i){
		vec.push_back(make_shared<PoseBase>(500.0,0.0,(45.0/180.0)*M_PI));
	}
	
	f32 min_likelihood(0.001);
	FastSlamPF pf(
			vec,
			control_motion,
			control_turn,
			meas_dist_std,
			meas_ang_std,
			min_likelihood);
	
	
	// output file Mgr
	unique_ptr<FastSlamFileMgr> pfm=make_unique<FastSlamFileMgr>(
			num_particles);
	
	// read data
        pmfm->read("../data/robot4_motors.txt");
        psfm->read("../data/robot4_scan.txt");
        vector<ControlBase> ticks(pmfm->get_data());
        vector<Scan::Scan> scans(psfm->get_data());

        // pf loop
	for(int i=0;i<ticks.size();++i){
		//if(ticks[i].Right_Tick()!=0&&ticks[i].Left_Tick()!=0)
		{
			// predict
			pf.Predict(ticks[i],cfg);
			
			// update
			/*FeatList feats(plfx->Feature_Extract(
					scans[i],s_cfg));

			pf.Update(feats,cfg);*/
		}
		//store outputs
		pfm->add_sample(FastSlamOutput(pf.Particles(),cfg));	
	
	}
	// write output file
        pfm->write("../data/slam.txt");
}
