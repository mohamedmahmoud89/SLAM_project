#include "lfx.h"
#include "robot_cfg.h"
#include<cmath>

using namespace std;

vector<u16> LidarFeatEx::derive_scan(
		const vector<u16>& scan,
		const ScanConfig& cfg){
	vector<u16>ret(1,0);
	for(size_t i=1;i<scan.size()-1;++i){
		if(scan[i-1]>cfg.Min_ValidDepth()&&
		   scan[i+1]>cfg.Min_ValidDepth()){
			ret.push_back((scan[i+1]-scan[i-1])/2);
			continue;
		}
		ret.push_back(0);
	}
}

vector<Feature> LidarFeatEx::find_features(
		const vector<u16>& scan,
		const vector<u16>& derivative,
		const ScanConfig& cfg){
	vector<Feature>ret;
	bool is_feat_scanned(false);
	f32 sum_rays(0),sum_depths(0);
	u8 num_rays(0);

	for(size_t i=0;i<scan.size();++i){
		if(derivative[i]<-cfg.Min_DeltaDepth()){
			is_feat_scanned=true;
			sum_rays=0;
			sum_depths=0;
			num_rays=0;
		}
		if(is_feat_scanned&&
			derivative[i]>cfg.Min_DeltaDepth()){
			f32 avg_r(sum_rays/num_rays);
			f32 avg_d(sum_depths/num_rays);
			f32 theta(RobotConfig::Ray_IdxToAng(avg_r));
			f32 x(avg_d+cfg.Feat_Offset()*cos(theta));
			f32 y(avg_d+cfg.Feat_Offset()*sin(theta));
			ret.push_back(Feature(x,y));
		}
		else if(is_feat_scanned&&
			scan[i]>cfg.Min_ValidDepth()){
			num_rays++;
			sum_rays+=i;
			sum_depths+=scan[i];
		}
	}

	return ret;
}

vector<Feature> LidarFeatEx::Feature_Extract(
		const vector<u16>& scan,
		const ScanConfig& cfg){
	vector<u16>derived_scan(derive_scan(scan,cfg));
	return find_features(scan,derived_scan,cfg);
}
