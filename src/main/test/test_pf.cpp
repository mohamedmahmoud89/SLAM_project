#include"file_mgr.h"
#include<memory>
#include<cmath>
#include "lfx.h"
#include "feat.h"
#include "test.h"
#include "pf.h"
#include<random>

using namespace std;

void Test::Test_Pf(){
	//motor data definitions
        MotorFileMgr* pmfm=MotorFileMgr::get_instance();
        //scan data definitions
        ScanFileMgr* psfm(ScanFileMgr::get_instance());
        Scan::ScanConfig s_cfg(20,100,90);
        unique_ptr<LidarFeatExBase>plfx(
                make_unique<LidarFeatExBase>());
        //ref landmark data definitions
        RefLandmarkFileMgr* prfm(RefLandmarkFileMgr::get_instance());

	// pf definitions
	PoseBase pos_mean(1850.0,1897.0,(213.0/180)*M_PI);
        PoseBase pos_std(100,100,((10.0 / 180.0) * M_PI));
	RobotConfig cfg(155.0,30.0,0.349);
        f32 control_motion(0.05);
        f32 control_turn(0.1);
        f32 meas_dist_std(200);
        f32 meas_ang_std((15.0/180.0)*M_PI);
	u16 num_particles(500);
	SmrtPtrVec<PoseBase>vec;

	// random engine
	random_device rd;
	default_random_engine gen(rd());
	//normal_distribution<f32> dist_x(pos_mean.X(),pos_std.X());
	//normal_distribution<f32> dist_y(pos_mean.Y(),pos_std.Y());
	//normal_distribution<f32> dist_yaw(pos_mean.Yaw(),pos_std.Yaw());
	uniform_real_distribution<f32> dist_x(0,2000);
	uniform_real_distribution<f32> dist_y(0,2000);
	uniform_real_distribution<f32> dist_yaw(-M_PI,M_PI);
	for(u16 i=0;i<num_particles;++i){
		f32 x(dist_x(gen));
		f32 y(dist_y(gen));
		f32 yaw(dist_yaw(gen));
		vec.push_back(make_shared<PoseBase>(x,y,yaw));
	}
	ParticleFilter pf(
			vec,
			control_motion,
			control_turn,
			meas_dist_std,
			meas_ang_std);
	
	
	// output file Mgr
	unique_ptr<PfFileMgr> pfm=make_unique<PfFileMgr>(
			num_particles);
	
	// read data
        pmfm->read("../data/robot4_motors.txt");
        psfm->read("../data/robot4_scan.txt");
        prfm->read("../data/robot_arena_landmarks.txt");
        vector<ControlBase> ticks(pmfm->get_data());
        vector<Scan::Scan> scans(psfm->get_data());
        SmrtPtrVec<Feature::FeatBase> refs(prfm->get_data());

        // pf loop
	for(int i=0;i<ticks.size();++i){
		if(ticks[i].Right_Tick()!=0&&ticks[i].Left_Tick()!=0)
		{
			// predict
			pf.Predict(ticks[i],cfg);
			
			// update
			FeatList feats(plfx->Feature_Extract(
					scans[i],s_cfg));

			pf.Update(feats,refs,cfg);
		}
		//store outputs
		pfm->add_sample(PfOutput(pf.Particles(),cfg));	
	
	}
	// write output file
        pfm->write("../data/pf.txt");
}
