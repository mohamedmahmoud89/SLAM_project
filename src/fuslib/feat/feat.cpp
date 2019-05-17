#include"feat.h"
#include<cmath>
#include<memory>
#include<algorithm>
using namespace std;
using namespace Feature;
using namespace Robot;
f32 max_ref_dist=300;
void Feature::FeatureTransform(
		FeatBase& feat,
		const PoseBase& coord,
		const RobotConfig& cfg){
	f32 scanner_x(
		coord.X()+(cfg.Sensor_Offset()*cos(coord.Yaw())));
	f32 scanner_y(
		coord.Y()+(cfg.Sensor_Offset()*sin(coord.Yaw())));
	f32 x(scanner_x+feat.X()*cos(coord.Yaw()));
	x-=(feat.Y()*sin(coord.Yaw()));
	feat.Set_X(x);
	f32 y(scanner_y+feat.X()*sin(coord.Yaw()));
	y+=(feat.Y()*cos(coord.Yaw()));
	feat.Set_Y(y);
}

unique_ptr<FeatAssoc> Feature::FeatAssociate(
		const SmrtPtrVec<FeatBase>& scanned,
		const SmrtPtrVec<FeatBase>& stored){
	unique_ptr<FeatAssoc> p_assoc(make_unique<FeatAssoc>());
	auto lambda_scan=[&p_assoc](const shared_ptr<FeatBase>& pf){
		p_assoc->scanned_t.insert(
			make_pair(pf->Id(),shared_ptr<FeatBase>(pf)));
	};
	auto lambda_store=[&p_assoc](const shared_ptr<FeatBase>& pf){
		p_assoc->stored_t.insert(
			make_pair(pf->Id(),shared_ptr<FeatBase>(pf)));
	};
	for_each(scanned.begin(),scanned.end(),lambda_scan);
	for_each(stored.begin() ,stored.end(), lambda_store);
	f32 min_squared(pow(max_ref_dist,2));
	for(auto&sc_ft:scanned){
		si32 id(-1);
		for(auto&st_ft:stored){
			f32 dist_squared(
				pow(sc_ft->X()-st_ft->X(),2)+
				pow(sc_ft->Y()-st_ft->Y(),2));
			if(dist_squared<min_squared){
				min_squared=dist_squared;
				id=st_ft->Id();
			}		
		}
		if(id!=-1)
			p_assoc->assocs_t.insert(
				make_pair(sc_ft->Id(),id));
	}
	return p_assoc;
}
