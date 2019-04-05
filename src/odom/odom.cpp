#include "odom.h"
#include<cmath>
//#include<iostream>
using namespace std;
void odom::UpdateS(const Tick& t,const robot_config& cfg){
	f32 dist(t.Right()*cfg.get_Ticks_to_mm());
	f32 x(p_pos->X());
	f32 y(p_pos->Y());
	f32 yaw(p_pos->Yaw());
	p_pos->set_x(x+(dist*cos(yaw)));
	p_pos->set_y(y+(dist*sin(yaw)));
}

void odom::UpdateC(const Tick& t,const robot_config& cfg){
	f32 t2mm(cfg.get_Ticks_to_mm());
	f32 dist_r(t.Right()*t2mm);
	f32 dist_l(t.Left() *t2mm);
        f32 width(cfg.get_width());
	f32 offset(cfg.get_offset());
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

void odom::UpdatePos(const Tick& t,const robot_config& cfg){
	if(t.Right()==t.Left()){
		UpdateS(t,cfg);
		return;
	}
	UpdateC(t,cfg);
} 
