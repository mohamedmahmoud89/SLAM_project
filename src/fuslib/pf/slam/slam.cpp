#include "slam.h"

void FastSlamPF::Update(
                FeatList& feats,
                const Robot::Config& cfg){
        auto weights(Calc_ImpWeights(feats,cfg));
        Resample(weights);
}

vector<f32> FastSlamPF::Calc_ImpWeights(
		FeatList& feats,
		const Robot::Config& cfg){
        vector<f32>ret;
        for(int i=0;i<particles.size();++i){
                f32 weight(1);
                for(auto&feat:feats.Data()){
			vector<f32> likelihood;
			for(int j=0;j<map[i].size();++j){
				likelihood.push_back(
					MaxLikeFeatAssociate(
						particles[i],
						feat,
						map[i][j],
						cfg,
						meas_dst_std,
						meas_ang_std));
			}

			// obtain the max likelihood idx
			auto it=max_element(likelihood.begin(),likelihood.end());
			
			// update weight
			weight*= *it;

			// check to either add a new landmark or update an existing
			if(*it>min_likelihood){
				Update_Landmark(i,feat,it-likelihood.begin(),cfg);
			}
			else{
				Insert_Landmark(i,feat,cfg);
			}
                }
                ret.push_back(weight);
        }
        return ret;
}

MatrixXf FastSlamPF::Compute_H(
               const PoseBase& pos,
               const FeatBase& landmark,
               const Robot::Config& cfg){
	MatrixXf H(2,2);
        f32 d(cfg.Sensor_Offset());
        f32 r(landmark.R());
        f32 theta(pos.Yaw());
        f32 scanner_x(
                pos.X()+(d*cos(theta)));
        f32 scanner_y(
                pos.Y()+(d*sin(theta)));
        f32 dx(landmark.GX()-scanner_x);
        f32 dy(landmark.GY()-scanner_y);
        H(0,0)=dx/r;
        H(0,1)=dy/r;
        H(1,0)=(-dy)/pow(r,2);
        H(1,1)=dx/pow(r,2);

	return H;
}

MatrixXf FastSlamPF::Compute_Q(){
	MatrixXf Q(2,2);
        Q(0,0)=pow(meas_dst_std,2);
        Q(0,1)=0;
        Q(1,0)=0;
        Q(1,1)=pow(meas_ang_std,2);

	return Q;
}


void FastSlamPF::Update_Landmark(
		const size_t particle_idx,
		const shared_ptr<FeatBase>& feature,
		int landmark_idx,
		const Config& cfg){
	// get old Mu and Var
	VectorXf Mu(2);
	Mu(0)=map[particle_idx][landmark_idx]->Mean()->X();
	Mu(1)=map[particle_idx][landmark_idx]->Mean()->Y();
	MatrixXf Var(*map[particle_idx][landmark_idx]->Covariance());
	
	// Q is the measurement variance
        MatrixXf Q(Compute_Q());

        // H is the jaccobian
        MatrixXf H(Compute_H(*particles[particle_idx],*feature,cfg));	
	
	// calculate kalman gain
	MatrixXf K(Var*H.transpose());
	K*=(H*Var*H.transpose()+Q).inverse();

	// innovation
	FeatBase proposed_meas;
	proposed_meas.Set_GX(map[particle_idx][landmark_idx]->Mean()->X());
        proposed_meas.Set_GY(map[particle_idx][landmark_idx]->Mean()->Y());
        FeaturePolarTransform(proposed_meas,*particles[particle_idx],cfg);
	
	VectorXf dz(2);
        dz(0)=feature->R()-proposed_meas.R();
        dz(1)=feature->Theta()-proposed_meas.Theta();
        dz(1)+=M_PI;
        dz(1)=fmod(dz(1),2*M_PI);
        dz(1)-=M_PI;

	Mu=Mu+K*dz;

	MatrixXf I(MatrixXf::Identity(2,2));
	Var=(I-(K*H))*Var;

	map[particle_idx][landmark_idx]->Mean()->set_x(Mu(0));
	map[particle_idx][landmark_idx]->Mean()->set_y(Mu(1));
	*map[particle_idx][landmark_idx]->Covariance()=Var;
}

void FastSlamPF::Insert_Landmark(
		const size_t particle_idx,
		const shared_ptr<FeatBase>& feature,
		const Config& cfg){
	FeatureGlobalTransform(*feature,*particles[particle_idx],cfg);

        // Mean = feature->GX(), feature->GY()
        PoseBase mean(feature->GX(), feature->GY(),0);

	// Q is the measurement variance
        MatrixXf Q(Compute_Q());

	// H is the jaccobian
	MatrixXf H(Compute_H(*particles[particle_idx],*feature,cfg));

	// Cov = H.inverse()*Q*H.inverse().transpose()
	MatrixXf Cov(H.inverse()*Q*H.inverse().transpose());

	// add a gaussian to the particle's landmarks vector
	map[particle_idx].push_back(make_shared<Gaussian<PoseBase>>(mean,Cov));
}
