#include "pf.h"
#include <random>
#include <cassert>
#include "common.h"
using namespace std;

pair<f32,f32> ParticleFilter::Compute_SigmaCtrl(
                        const ControlBase& ctrl,
                        const Robot::Config& cfg){
	f32 t2mm(cfg.Ticks_ToMm());
        f32 r(ctrl.Right_Tick()*t2mm);
        f32 l(ctrl.Left_Tick()*t2mm);
        f32 motion_var_l(pow(ctrl_motion*l,2)+
                         pow(ctrl_turn*(l-r),2));
        f32 motion_var_r(pow(ctrl_motion*r,2)+
                         pow(ctrl_turn*(l-r),2));
	return make_pair<f32,f32>(
			sqrt(motion_var_r),
			sqrt(motion_var_l));
}

void ParticleFilter::Resample(const vector<f32>& weights){
	SmrtPtrVec<PoseBase>resampled;
	u16 N(particles.size());
	u16 idx(rand()%N);
	f32 beta(0);
	f32 mw(*max_element(weights.begin(),weights.end()));

	for(u16 i=0;i<N;++i){
		f32 random_num(static_cast<f32>(rand())/RAND_MAX);
		beta=(random_num*2*mw);
		while(beta>weights[idx]){
			beta-=weights[idx];
			idx=(idx+1)%N;
		}
		// sample new particles around the one of chosen index
		random_device rd;
		default_random_engine gen(rd());
		normal_distribution<f32> dist_x(particles[idx]->X(),20.0);
		normal_distribution<f32> dist_y(particles[idx]->Y(),20.0);
		normal_distribution<f32> dist_yaw(particles[idx]->Yaw(),((2.0 / 180.0) * M_PI));
		f32 x(dist_x(gen));
                f32 y(dist_y(gen));
                f32 yaw(dist_yaw(gen));
		resampled.push_back(make_shared<PoseBase>(x,y,yaw));
	}
	particles.clear();
	particles=move(resampled);
}

void ParticleFilter::Predict(
		const ControlBase& ctrl,
		const Robot::Config& cfg){
	unique_ptr<MMSimple>p_motion(make_unique<MMSimple>());
	pair<f32,f32> ctrl_std(
			Compute_SigmaCtrl(ctrl,cfg));
	random_device dev;
	default_random_engine gen(dev());
	normal_distribution<f32> dist_l(
			ctrl.Left_Tick(),ctrl_std.second);
	normal_distribution<f32> dist_r(
			ctrl.Right_Tick(),ctrl_std.first);
	
	for(auto&i:particles){
		f32 l(dist_l(gen));
		f32 r(dist_r(gen));
		ControlBase sampled_ctrl(l,r);
		PoseBase temp(i->X(),i->Y(),i->Yaw());
		p_motion->UpdatePos(temp,sampled_ctrl,cfg);
		i->set_x(temp.X());
		i->set_y(temp.Y());
		i->set_yaw(temp.Yaw());
	}
}

PfOutput::PfOutput(
                const SmrtPtrVec<PoseBase>& pars,
                const RobotConfig& in_cfg){
        // cfg
        cfg=make_unique<RobotConfig>(in_cfg);
        f32 mean_x(0),mean_y(0),mean_cos(0),mean_sin(0);

        // particles
        for(auto& i:pars){
                particles.push_back(make_unique<PoseBase>(*i));
                mean_x+=i->X();
                mean_y+=i->Y();
                mean_cos=cos(i->Yaw());
                mean_sin=sin(i->Yaw());
        }

        // mean
        auto sz(pars.size());
        mean_sin/=sz;
        mean_cos/=sz;
        mean=make_unique<const PoseBase>(mean_x/sz,mean_y/sz,atan2(mean_sin,mean_cos));

        // std
        Matrix2f std_xy;
        f32 sxx(0),sxy(0),syy(0),stheta(0);
        for(auto&i:pars){
                f32 dx(i->X()-mean->X());
                f32 dy(i->Y()-mean->Y());
                sxx+= pow(dx,2);
                sxy+= dx*dy;
                syy+= pow(dy,2);

                f32 dh(i->Yaw()-mean->Yaw()+M_PI);
                dh=fmod(dh,2*M_PI);
                dh-=M_PI;
                stheta+= pow(dh,2);
        }
        sxx/=(sz-1);
        syy/=(sz-1);
        sxy/=(sz-1);
        stheta/=(sz-1);
        std_xy << sxx,sxy,sxy,syy;
        SelfAdjointEigenSolver<MatrixXf> eigensolver(std_xy);
        auto eigenvals=eigensolver.eigenvalues();
        auto eigenvecs=eigensolver.eigenvectors();
        f32 x(sqrt(eigenvals(1)));
        f32 y(sqrt(eigenvals(0)));
        f32 xy_angle(atan2(eigenvecs(1,1),eigenvecs(0,1)));
        f32 theta(sqrt(stheta));
        std=make_tuple(xy_angle,x,y,theta);
}

PfOutput::PfOutput(const PfOutput& rhs){
        cfg=make_unique<RobotConfig>(*rhs.cfg);
        for(auto& i:rhs.particles){
                particles.push_back(make_unique<PoseBase>(*i));
        }
        mean=make_unique<const PoseBase>(*rhs.mean);
        std=rhs.std;
}

