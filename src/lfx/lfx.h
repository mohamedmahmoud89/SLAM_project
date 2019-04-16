#include "common.h"
#include "lfx_cfg.h"
#include <vector>

using namespace std;

class LidarFeatEx{
	vector<si16> derive_scan(
			const vector<u16>& scan,
			const ScanConfig& cfg);
	vector<FeatBase> find_features(
			const vector<u16>& scan,
			const vector<si16>& derivative,
			const ScanConfig& cfg);
public:
	vector<FeatBase> Feature_Extract(
			const Scan::ScanBase<u16>& scan,
			const Scan::Config& cfg);
};
