#include "ekf.h"
#include<cmath>
using namespace std;
using namespace Eigen;

shared_ptr<MatrixXf> Ekf::Compute_G_linear(
		const PoseBase& pos,
		const ControlBase& ctrl,
		const Robot::Config& cfg){
	shared_ptr<MatrixXf> ret=
		make_shared<MatrixXf>(state_space,state_space);
	f32 t2mm(cfg.Ticks_ToMm());
        f32 dist_l(ctrl.Left_Tick() *t2mm);
	f32 theta(pos.Yaw());
	MatrixXf m(2,2);
	m(0,0)=0;
	
	(*ret)(0,0)=1;
	(*ret)(0,1)=0;
	(*ret)(0,2)=-dist_l*sin(theta);
	(*ret)(1,0)=0;
	(*ret)(1,1)=1;
	(*ret)(1,2)=dist_l*cos(theta);
	(*ret)(2,0)=0;
	(*ret)(2,1)=0;
	(*ret)(2,2)=1;
	return ret;
}

shared_ptr<MatrixXf> Ekf::Compute_G_non_linear(
		const PoseBase& pos,
		const ControlBase& ctrl,
		const Robot::Config& cfg){
	shared_ptr<MatrixXf> ret=
		make_shared<MatrixXf>(state_space,state_space);
	f32 t2mm(cfg.Ticks_ToMm());
        f32 dist_r(ctrl.Right_Tick()*t2mm);
        f32 dist_l(ctrl.Left_Tick() *t2mm);
        f32 width(cfg.Width());
	f32 theta(pos.Yaw());
        f32 alpha((dist_r-dist_l)/width);
        f32 radius(dist_l/alpha);
	(*ret)(0,0)=1;
	(*ret)(0,1)=0;
	(*ret)(0,2)=(radius+(width/2.0)*(cos(theta+alpha)-cos(theta)));
	(*ret)(1,0)=0;
	(*ret)(1,1)=1;
	(*ret)(1,2)=(radius+(width/2.0)*(sin(theta+alpha)-sin(theta)));
	(*ret)(2,0)=0;
	(*ret)(2,1)=0;
	(*ret)(2,2)=1;
	return ret;
}

shared_ptr<MatrixXf> Ekf::Compute_G(
		const PoseBase& pos,
		const ControlBase& ctrl,
		const Robot::Config& cfg){
	if(ctrl.Right_Tick()==ctrl.Left_Tick()){
		return Compute_G_linear(pos,ctrl,cfg);
	}

	return Compute_G_non_linear(pos,ctrl,cfg);
}

shared_ptr<MatrixXf> Ekf::Compute_V(
		const PoseBase& pos,
		const ControlBase& ctrl,
		const Robot::Config& cfg){
	
	shared_ptr<MatrixXf> ret=
		make_shared<MatrixXf>(state_space,2);
	f32 t2mm(cfg.Ticks_ToMm());
        f32 r(ctrl.Right_Tick()*t2mm);
        f32 l(ctrl.Left_Tick() *t2mm);
        f32 width(cfg.Width());
	f32 theta(pos.Yaw());
        f32 alpha((r-l)/width);
        f32 radius(l/alpha);
	(*ret)(2,0)=-1.0/width;
	(*ret)(2,1)=1.0/width;
	if(ctrl.Right_Tick()!=ctrl.Left_Tick()){
		(*ret)(0,0)=((width*r)/pow(r-l,2))*
			(sin(theta+alpha)-sin(theta));
		(*ret)(0,0)-=(((r+l)/(2*(r-l)))*cos(theta+alpha));
		(*ret)(1,0)=((width*r)/pow(r-l,2))*
			(-cos(theta+alpha)+cos(theta));
		(*ret)(1,0)-=(((r+l)/(2*(r-l)))*sin(theta+alpha));
		
		(*ret)(0,1)=((-width*l)/pow(r-l,2))*
			(sin(theta+alpha)-sin(theta));
		(*ret)(0,1)+=(((r+l)/(2*(r-l)))*cos(theta+alpha));
		(*ret)(1,1)=((-width*l)/pow(r-l,2))*
			(-cos(theta+alpha)+cos(theta));
		(*ret)(1,1)-=(((r+l)/(2*(r-l)))*sin(theta+alpha));
		return ret;
	}
	(*ret)(0,0)=0.5*(cos(theta)+((l/width)*sin(theta)));
	(*ret)(0,1)=0.5*(((-l/width)*sin(theta))+cos(theta));
	(*ret)(1,0)=0.5*(sin(theta)-((l/width)*cos(theta)));
	(*ret)(1,1)=0.5*(((l/width)*cos(theta))+sin(theta));
	return ret;
}

shared_ptr<MatrixXf> Ekf::Compute_SigmaCtrl(
                        const ControlBase& ctrl,
			const Robot::Config& cfg){
	shared_ptr<MatrixXf>ret(make_shared<MatrixXf>(2,2));
	f32 t2mm(cfg.Ticks_ToMm());
        f32 r(ctrl.Right_Tick()*t2mm);
        f32 l(ctrl.Left_Tick() *t2mm);
	f32 motion_var_l(pow(ctrl_motion*l,2)+
			 pow(ctrl_turn*(l-r),2));
	f32 motion_var_r(pow(ctrl_motion*r,2)+
			 pow(ctrl_turn*(l-r),2));
	(*ret) << motion_var_l,0,
		  0,motion_var_r;
	return ret;
}

void Ekf::Predict(
		const ControlBase& ctrl,
		const Robot::Config& cfg){
	unique_ptr<MMSimple>p_motion(make_unique<MMSimple>());
	shared_ptr<MatrixXf> sigma_ctrl(Compute_SigmaCtrl(ctrl,cfg));
	shared_ptr<MatrixXf> G(Compute_G(*belief.Mean(),ctrl,cfg));
	shared_ptr<MatrixXf> V(Compute_V(*belief.Mean(),ctrl,cfg));
	// R=V*Sigma_ctrl*VT   V=dg/d ctrl
	MatrixXf R((*V)*(*sigma_ctrl)*((*V).transpose()));
	// Sigma=G*Sigma*GT+R  G=dg/d state
	*(belief.Covariance())=
		(*G)*(*belief.Covariance())*((*G).transpose())+R;
	// Mu=g(Mt-1,Ut)
	shared_ptr<PoseBase>p_pos(belief.Mean());
	p_motion->UpdatePos(*p_pos,ctrl,cfg);
}

void Ekf::Update(const FeatAssoc& assocs){
	// K=Sigma*HT*(H*Sigma*HT+Q)⁻¹
	
	// Mu=Mu+K*(Z-h(Mu))
	
	// Sigma=(I-K*H)Sigma
}

tuple<f32,f32,f32,f32> EkfOutput::Std()const{
	Matrix2f std_xy;
	std_xy << (*belief.Covariance())(0,0),
	      (*belief.Covariance())(0,1),
	      (*belief.Covariance())(1,0),
	      (*belief.Covariance())(1,1);
	SelfAdjointEigenSolver<MatrixXf> eigensolver(std_xy);
	auto eigenvals=eigensolver.eigenvalues();
	auto eigenvecs=eigensolver.eigenvectors();
	f32 x(sqrt(eigenvals(0)));
	f32 y(sqrt(eigenvals(1)));
	f32 xy_angle(atan2(eigenvecs(1,0),eigenvecs(0,0)));
	f32 theta(sqrt((*belief.Covariance())(2,2)));
	return make_tuple(xy_angle,x,y,theta);
}

unique_ptr<PoseBase> EkfOutput::Pos() const{
	PoseBase pos(*belief.Mean());
	f32 offset(cfg.Sensor_Offset());
	f32 x(pos.X()+offset*cos(pos.Yaw()));
	f32 y(pos.Y()+offset*sin(pos.Yaw()));
	return make_unique<PoseBase>(x,y,pos.Yaw());
}
