#include"feat.h"
#include<cmath>
#include<memory>
#include<algorithm>
using namespace std;
using namespace Feature;
using namespace Robot;
f32 max_ref_dist=300;
void Feature::FeatureGlobalTransform(
		FeatBase& feat,
		const PoseBase& coord,
		const Config& cfg){
	f32 scanner_x(
		coord.X()+(cfg.Sensor_Offset()*cos(coord.Yaw())));
	f32 scanner_y(
		coord.Y()+(cfg.Sensor_Offset()*sin(coord.Yaw())));
	f32 x(scanner_x+feat.X()*cos(coord.Yaw()));
	x-=(feat.Y()*sin(coord.Yaw()));
	feat.Set_GX(x);
	f32 y(scanner_y+feat.X()*sin(coord.Yaw()));
	y+=(feat.Y()*cos(coord.Yaw()));
	feat.Set_GY(y);
}

void Feature::FeaturePolarTransform(
		FeatBase& feat,
		const PoseBase& coord,
		const Config& cfg){	
	f32 scanner_x(
		coord.X()+(cfg.Sensor_Offset()*cos(coord.Yaw())));
	f32 scanner_y(
		coord.Y()+(cfg.Sensor_Offset()*sin(coord.Yaw())));
	f32 dx(feat.GX()-scanner_x);
	f32 dy(feat.GY()-scanner_y);
	f32 r(sqrt(pow(dx,2)+pow(dy,2)));
	f32 theta(atan2(dy,dx)-coord.Yaw()+M_PI);
	//while(theta>=2*M_PI)theta-=(2*M_PI);
	theta=fmod(theta,2*M_PI);
	theta-=M_PI;
	feat.Set_R(r);
	feat.Set_Theta(theta);
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
	for(auto&sc_ft:scanned){
		si32 id(-1);
		f32 min_squared(pow(max_ref_dist,2));
		for(auto&st_ft:stored){
			f32 dist_squared(
				pow(sc_ft->GX()-st_ft->GX(),2)+
				pow(sc_ft->GY()-st_ft->GY(),2));
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

vector<FeatBase> Feature::ExtractRefFeatures(
		const FeatAssoc& assocs){
	vector<FeatBase>ret;
	for(auto& assoc:assocs.assocs_t){
		auto p_ref=assocs.stored_t.find(
                                assoc.second)->second;
		FeatBase temp(*p_ref);
		ret.push_back(temp);
	}
	return ret;
}
