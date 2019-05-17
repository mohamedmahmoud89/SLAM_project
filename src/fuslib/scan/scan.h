#ifndef FUS_SCAN_H
#define FUS_SCAN_H
#include "common.h"
using namespace std;

namespace Scan{
	class Config{
	public:
		virtual ~Config(){}
		virtual u8 Min_ValidDepth() const noexcept=0;	
	};
	class ScanConfig : public Config{
	public:
		ScanConfig():
			min_valid_depth(0),
			min_delta_depth(0),
			feat_center_offset(0){}
		ScanConfig(const u8 dist,
				const u8 jump,const u8 offset):
                min_valid_depth(dist),
                min_delta_depth(jump),
                feat_center_offset(offset){}
		virtual ~ScanConfig(){}
		u8 Min_ValidDepth() const noexcept override
		{
			return min_valid_depth;
		}
		u8 Min_DeltaDepth() const noexcept{
			return min_delta_depth;}
		u8 Feat_Offset() const noexcept{
			return feat_center_offset;}
	protected:
		u8 min_valid_depth{0}; // min valid depth info
		// min delta depth to consider a feat.
		u8 min_delta_depth{0};
		// offset between the feat surface and the feat center
		u8 feat_center_offset{0};
	};

	class RayInfoBase{
	protected:
		u16 depth;
	public:
		RayInfoBase(const u16 d):depth(d){}
		virtual ~RayInfoBase(){}
		virtual int Depth() const noexcept{return depth;}
	};
	class Scan{
		SmrtPtrVec<RayInfoBase> scan_data;
	public:
		const SmrtPtrVec<RayInfoBase>& Data() const noexcept
		{
			return scan_data;
		}
		void Push_Back(const shared_ptr<RayInfoBase>& p_ray)
		{
			scan_data.push_back(p_ray);
		}
	};
};
#endif
