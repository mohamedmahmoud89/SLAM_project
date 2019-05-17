#include"file_mgr.h"
#include<memory>
#include"lfx.h"
#include"test.h"
using namespace std;

void Test::Test_Scan(){
        //scan data definitions
	ScanFileMgr* psfm(ScanFileMgr::get_instance());
        unique_ptr<Scan::ScanConfig>s_cfg(
                make_unique<Scan::ScanConfig>(20,100,90));
        unique_ptr<LidarFeatExBase>plfx(
                make_unique<LidarFeatExBase>());
        unique_ptr<FeatFileMgr>ffm(make_unique<FeatFileMgr>());
        
        //scan data
        psfm->read("../data/robot4_scan.txt");
        vector<Scan::Scan> scans(psfm->get_data());
        for(auto&i:scans){
                Feature::FeatList vf(plfx->Feature_Extract(
                                i,*s_cfg));
                ffm->add_sample(vf);
        }
        ffm->write("../data/cylinders.txt");
}

