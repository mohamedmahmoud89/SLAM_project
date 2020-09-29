#include"feat.h"
#include<cmath>
#include<memory>
#include<algorithm>
using namespace std;
using namespace Feature;
using namespace Robot;
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
	f32 theta(atan2(dy,dx)-coord.Yaw());
	f32 sign(theta/fabs(theta));
	theta+=(sign*M_PI);
	//while(theta>=2*M_PI)theta-=(2*M_PI);
	theta=fmod(theta,2*M_PI);
	theta-=(sign*M_PI);
	feat.Set_R(r);
	feat.Set_Theta(theta);
}

unique_ptr<FeatAssoc> Feature::FeatAssociate(
		const SmrtPtrVec<FeatBase>& scanned,
		const SmrtPtrVec<FeatBase>& stored,
		const f32 max_ref_dist){
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
		f64 min_squared(pow(max_ref_dist,2));
		//f64 min_squared(numeric_limits<long double>::max());
		for(auto&st_ft:stored){
			f64 dist_squared(
				pow(sc_ft->GX()-st_ft->GX(),2)+
				pow(sc_ft->GY()-st_ft->GY(),2));
			//cout<<"dist_sq = "<<dist_squared<<endl;
			//cout<<"min_sq = "<<min_squared<<endl;	
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

f32 Feature::MaxLikeFeatAssociate(
                const shared_ptr<PoseBase>& particle,
                const shared_ptr<FeatBase>& feature,
                const shared_ptr<Gaussian<PoseBase>>& landmark,
		const Config& cfg,
		const f32 meas_dst_std,
		const f32 meas_ang_std){
	float ret(0);
	// the likelihood is the pdf function N(z-z_proposed, 0, SIGMA_SQ)
	// implement the reverse measurement function h_rev() to calc z_proposed
	FeatBase proposed_meas;
	proposed_meas.Set_GX(landmark->Mean()->X());	
	proposed_meas.Set_GY(landmark->Mean()->Y());
	FeaturePolarTransform(proposed_meas,*particle,cfg);
	
	// H is the jacobian of the inverse measurement function h_rev() with respect to the landmark
	MatrixXf H(2,2);
	f32 d(cfg.Sensor_Offset());
        f32 r(proposed_meas.R());
        f32 theta(particle->Yaw());
        f32 scanner_x(
                particle->X()+(d*cos(theta)));
        f32 scanner_y(
                particle->Y()+(d*sin(theta)));
        f32 dx(proposed_meas.GX()-scanner_x);
        f32 dy(proposed_meas.GY()-scanner_y);
        H(0,0)=dx/r;
        H(0,1)=dy/r;
        H(1,0)=(-dy)/pow(r,2);
        H(1,1)=dx/pow(r,2);
	
	// Q is the measurement variance
	MatrixXf Q(2,2);
	Q(0,0)=pow(meas_dst_std,2);
	Q(0,1)=0;
	Q(1,0)=0;
	Q(1,1)=pow(meas_ang_std,2);

	// implement the variance propagation to calc the SIGMA_SQ = H*SIGMA*H_transpose + Q
	MatrixXf Cov(2,2);
	Cov = H*(*landmark->Covariance())*H.transpose();
	Cov+=Q;

	// calculate z-z_proposed
	VectorXf delta_z(2);
	delta_z(0)=feature->R()-proposed_meas.R();
	delta_z(1)=feature->Theta()-proposed_meas.Theta();
	delta_z(1)+=M_PI;
        delta_z(1)=fmod(delta_z(1),2*M_PI);
        delta_z(1)-=M_PI;

	// calc the pdf N(delta_z,0,Cov)
	ret=(1.0f/(sqrt(2*M_PI*Cov.determinant())))*
		exp(-0.5f*delta_z.transpose()*Cov.inverse()*delta_z);

	return ret;
}
