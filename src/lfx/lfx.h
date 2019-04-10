#include "common.h"
#include "lfx_cfg.h"
#include <vector>

using namespace std;

class LidarFeatEx{
	vector<si16> derive_scan(
			const vector<u16>& scan,
			const ScanConfig& cfg);
	vector<Feature> find_features(
			const vector<u16>& scan,
			const vector<si16>& derivative,
			const ScanConfig& cfg);
public:
	vector<Feature> Feature_Extract(
			const vector<u16>&scan,
			const ScanConfig& cfg);
};
