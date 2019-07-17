#include "lfx.h"
#include "robot_cfg.h"
#include<cmath>
#include "feat.h"
using namespace std;
using namespace Robot;
vector<si16> LidarFeatExBase::derive_scan(
		const Scan::Scan& scan,
		const Scan::ScanConfig& cfg){
	vector<si16>ret(1,0);
	for(size_t i=1;i<scan.Data().size()-1;++i){
		if(scan.Data()[i-1]->Depth()>cfg.Min_ValidDepth()&&
		   scan.Data()[i+1]->Depth()>cfg.Min_ValidDepth()){
			ret.push_back((
				scan.Data()[i+1]->Depth()-
				scan.Data()[i-1]->Depth())/2);
			continue;
		}
		ret.push_back(0);
	}
	return ret;
}

Feature::FeatList LidarFeatExBase::find_features(
		const Scan::Scan& scan,
		const vector<si16>& derivative,
		const Scan::ScanConfig& cfg){
	Feature::FeatList ret;
	bool is_feat_scanned(false);
	f32 sum_rays(0),sum_depths(0);
	u8 num_rays(0);
	size_t id(0);

	for(size_t i=0;i<derivative.size();++i){
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
			ret.Push_Back(
				make_shared<FeatBase>(
					x,y,avg_r,theta,0,0,id++));
		}
		else if(is_feat_scanned&&
			scan.Data()[i]->Depth()>cfg.Min_ValidDepth()){
			num_rays++;
			sum_rays+=i;
			sum_depths+=scan.Data()[i]->Depth();
		}
	}

	return ret;
}

Feature::FeatList LidarFeatExBase::Feature_Extract(
		const Scan::Scan& scan,
		const Scan::Config& cfg){
	vector<si16>derived_scan(
		derive_scan(scan,
		dynamic_cast<const Scan::ScanConfig&>(cfg)));
	return find_features(
		scan,
		derived_scan,
		dynamic_cast<const Scan::ScanConfig&>(cfg));
}
