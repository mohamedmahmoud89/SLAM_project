#include "odom.h"
#include<cmath>
//#include<iostream>
using namespace std;
void Odom::UpdateS(const Tick& t,const OdomConfig& cfg){
	f32 dist(t.Right()*cfg.Ticks_ToMm());
	f32 x(p_pos->X());
	f32 y(p_pos->Y());
	f32 yaw(p_pos->Yaw());
	p_pos->set_x(x+(dist*cos(yaw)));
	p_pos->set_y(y+(dist*sin(yaw)));
}

void Odom::UpdateC(const Tick& t,const OdomConfig& cfg){
	f32 t2mm(cfg.Ticks_ToMm());
	f32 dist_r(t.Right()*t2mm);
	f32 dist_l(t.Left() *t2mm);
        f32 width(cfg.Width());
	f32 offset(cfg.Sensor_Offset());
	f32 x(p_pos->X());
        f32 y(p_pos->Y());
        f32 yaw(p_pos->Yaw());

	f32 alpha((dist_r-dist_l)/width);
	f32 radius(dist_l/alpha);
	f32 r_x(x-(offset*cos(yaw)));
	f32 r_y(y-(offset*sin(yaw)));
	f32 rot_rad(radius+(width/2.0));
	f32 c_x(r_x-(rot_rad*sin(yaw)));
	f32 c_y(r_y+(rot_rad*cos(yaw)));
	yaw+=alpha;
	while(yaw>=2*M_PI)
		yaw-=2*M_PI;
	p_pos->set_yaw(yaw);
	c_x+=rot_rad*sin(yaw);
	c_y-=rot_rad*cos(yaw);
	p_pos->set_x(c_x+(offset*cos(yaw)));
	p_pos->set_y(c_y+(offset*sin(yaw)));
}

void Odom::UpdatePos(const Tick& t,const OdomConfig& cfg){
	if(t.Right()==t.Left()){
		UpdateS(t,cfg);
		return;
	}
	UpdateC(t,cfg);
} 
