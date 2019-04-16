#include "lfx.h"
#include "robot_cfg.h"
#include<cmath>

using namespace std;
using namespace Robot;
vector<si16> LidarFeatEx::derive_scan(
		const vector<u16>& scan,
		const ScanConfig& cfg){
	vector<si16>ret(1,0);
	for(size_t i=1;i<scan.size()-1;++i){
		if(scan[i-1]>cfg.Min_ValidDepth()&&
		   scan[i+1]>cfg.Min_ValidDepth()){
			ret.push_back((scan[i+1]-scan[i-1])/2);
			continue;
		}
		ret.push_back(0);
	}
	return ret;
}

vector<FeatBase> LidarFeatEx::find_features(
		const vector<u16>& scan,
		const vector<si16>& derivative,
		const ScanConfig& cfg){
	vector<FeatBase>ret;
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
			f32 x((avg_d+cfg.Feat_Offset())*cos(theta));
			f32 y((avg_d+cfg.Feat_Offset())*sin(theta));
			ret.push_back(FeatBase(x,y));
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

vector<FeatBase> LidarFeatEx::Feature_Extract(
		const Scan::ScanBase<u16>& scan,
		const Scan::Config& cfg){
	vector<si16>derived_scan(
		derive_scan(scan.Depth(),
		dynamic_cast<const ScanConfig&>(cfg)));
	return find_features(
		scan.Depth(),
		derived_scan,
		dynamic_cast<const ScanConfig&>(cfg));
}
