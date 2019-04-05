#include "common.h"

class ScanConfig{
public:
	ScanConfig():
		min_scan_dist(0),
		depth_jump(0),
		feat_offset(0){}
	ScanConfig(const u8 dist,const u8 jump,const u8 offset):
		min_scan_dist(dist),
		depth_jump(jump),
		feat_offset(offset){}

	u8 min_scan_dist{0};
	u8 depth_jump{0};
	u8 feat_offset{0};
};
