#include "common.h"
#include "scan.h"
#include "feat.h"
#include <vector>

using namespace std;
using namespace Feature;
class LidarFeatExBase{
	vector<si16> derive_scan(
			const Scan::Scan& scan,
			const Scan::ScanConfig& cfg);
	FeatList find_features(
			const Scan::Scan& scan,
			const vector<si16>& derivative,
			const Scan::ScanConfig& cfg);
public:
	virtual FeatList Feature_Extract(
			const Scan::Scan& scan,
			const Scan::Config& cfg);
};
