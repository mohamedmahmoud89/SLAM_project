#include "common.h"

class ScanConfig{
public:
	ScanConfig():
		min_valid_depth(0),
		min_delta_depth(0),
		feat_center_offset(0){}
	ScanConfig(const u8 dist,const u8 jump,const u8 offset):
		min_valid_depth(dist),
		min_delta_depth(jump),
		feat_center_offset(offset){}
	u8 Min_ValidDepth() const noexcept{return min_valid_depth;}
	u8 Min_DeltaDepth() const noexcept{return min_delta_depth;}
	u8 Feat_Offset() const noexcept{return feat_center_offset;}
private:
	u8 min_valid_depth{0}; // min valid depth info
	u8 min_delta_depth{0}; // min delta depth to consider a feat.
	// offset between the feat surface and the feat center
	u8 feat_center_offset{0};
};
