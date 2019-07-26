#include"file_mgr.h"
#include<memory>
#include"odom.h"
#include<cmath>
#include"lfx.h"
#include"feat.h"
#include"test.h"
#include"ekf.h"
using namespace std;
using namespace Eigen;
//TODO:
//implement the prediction step
void Test::Test_Klmn(){
        //motor data definitions
        MotorFileMgr* pmfm=MotorFileMgr::get_instance();
        //scan data definitions
        ScanFileMgr* psfm(ScanFileMgr::get_instance());
        unique_ptr<Scan::ScanConfig>s_cfg(
                make_unique<Scan::ScanConfig>(20,100,90));
        unique_ptr<LidarFeatExBase>plfx(
                make_unique<LidarFeatExBase>());
        //ref landmark data definitions
        RefLandmarkFileMgr* prfm(RefLandmarkFileMgr::get_instance());

	// output file Mgr
	unique_ptr<EkfFileMgr> efm=make_unique<EkfFileMgr>();
	
	//ekf definitions
	Matrix3f covariance;
        covariance << pow(100,2),0,0,
		     0,pow(100,2),0,
		     0,0,pow(((10.0 / 180.0) * M_PI),2);
        PoseBase pos(1850.0,1897.0,(213.0/180)*M_PI);
        RobotConfig cfg(155.0,30.0,0.349);
	Matrix2f motion_covar;
	f32 control_motion(0.35);
	f32 control_turn(0.6);
	f32 meas_dist_std(200);
	f32 meas_ang_std((15.0/180.0)*M_PI);
	Ekf ekf(3,pos,
		covariance,
		control_motion,
		control_turn,
		meas_dist_std,
		meas_ang_std);

        // read data
        pmfm->read("../data/robot4_motors.txt");
        psfm->read("../data/robot4_scan.txt");
        prfm->read("../data/robot_arena_landmarks.txt");
        vector<ControlBase> ticks(pmfm->get_data());
	vector<Scan::Scan> scans(psfm->get_data());
        SmrtPtrVec<Feature::FeatBase> refs(prfm->get_data());
	
	// Ekf loop
        for(int i=0;i<ticks.size();++i){
                //predict
                ekf.Predict(ticks[i],cfg);

                //associate
                FeatList vf(plfx->Feature_Extract(
                                scans[i],*s_cfg));
                for(auto&feat:vf.Data()){
                        // transform the features to world coords
			// based on the robot pos
			FeatureGlobalTransform(
					*feat,*ekf.Belief().Mean(),cfg);
                }
		unique_ptr<FeatAssoc> assocs(
			FeatAssociate(vf.Data(),refs));

		for(auto&ref:refs){
			// fill in the polar attributes of refs to
			// be used in the update step
			FeaturePolarTransform(
				*ref,*ekf.Belief().Mean(),cfg);
		}

		//update
		ekf.Update(*assocs,cfg);

		//store outputs
		EkfOutput log(ekf.Belief(),cfg);
		efm->add_sample(log);
        }
	// write output file
	efm->write("../data/klmn.txt");
}


