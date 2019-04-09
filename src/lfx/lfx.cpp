#include "lfx.h"
#include "robot_cfg.h"

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

}

vector<Feature> LidarFeatEx::Feature_Extract(
		const vector<u16>& scan,
		const ScanConfig& cfg){
	vector<u16>derived_scan(derive_scan(scan,cfg));
	return find_features(scan,derived_scan,cfg);
}
