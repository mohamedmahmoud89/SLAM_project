#ifndef LFX_H
#define LFX_H
#include "common.h"
#include "scan.h"
#include "feat.h"
#include <vector>

using namespace std;
using namespace Feature;
class LidarFeatExBase{
	vector<f32> derive_scan(
			const Scan::Scan& scan,
			const Scan::ScanConfig& cfg);
	FeatList find_features(
			const Scan::Scan& scan,
			const vector<f32>& derivative,
			const Scan::ScanConfig& cfg);
public:
	virtual ~LidarFeatExBase(){}
	virtual FeatList Feature_Extract(
			const Scan::Scan& scan,
			const Scan::Config& cfg);
};
#endif
