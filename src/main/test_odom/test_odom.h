#include"file_mgr.h"
#include<memory>
#include"odom.h"
#include<cmath>
using namespace std;

void Test_Odom(){
        //motor data definitions
        MotorFileMgr* pmfm=MotorFileMgr::get_instance();
        unique_ptr<Odom>odo=make_unique<Odom>(
                        PoseBase(1850.0,1897.0,(213.0/180)*M_PI));
        unique_ptr<PosFileMgr> pfm=make_unique<PosFileMgr>();
        unique_ptr<RobotConfig> cfg=
                make_unique<RobotConfig>(150.0,30.0,0.349);
        // motor data
        pmfm->read("../data/robot4_motors.txt");
        vector<ControlBase> v(pmfm->get_data());
        for(auto& t:v){
                odo->Update(t,*cfg);
                pfm->add_sample(odo->get_pos());
        }

        pfm->write("../data/poses_from_ticks.txt");
}
