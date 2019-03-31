#include "odom.h"
#include<cmath>
//#include<iostream>
using namespace std;
void odom::UpdateS(const tick& t,const robot_config& cfg){
	float dist(t.right()*cfg.get_ticks_to_mm());
	float x(p_pos->get_x());
	float y(p_pos->get_y());
	float yaw(p_pos->get_yaw());
	p_pos->set_x(x+(dist*cos(yaw)));
	p_pos->set_y(y+(dist*sin(yaw)));
}

void odom::UpdateC(const tick& t,const robot_config& cfg){
	float t2mm(cfg.get_ticks_to_mm());
	float dist_r(t.right()*t2mm);
	float dist_l(t.left() *t2mm);
        float width(cfg.get_width());
	float offset(cfg.get_offset());
	float x(p_pos->get_x());
        float y(p_pos->get_y());
        float yaw(p_pos->get_yaw());

	float alpha((dist_r-dist_l)/width);
	float radius(dist_l/alpha);
	float r_x(x-(offset*cos(yaw)));
	float r_y(y-(offset*sin(yaw)));
	float rot_rad(radius+(width/2.0));
	float c_x(r_x-(rot_rad*sin(yaw)));
	float c_y(r_y+(rot_rad*cos(yaw)));
	yaw+=alpha;
	while(yaw>=2*M_PI)
		yaw-=2*M_PI;
	p_pos->set_yaw(yaw);
	c_x+=rot_rad*sin(yaw);
	c_y-=rot_rad*cos(yaw);
	p_pos->set_x(c_x+(offset*cos(yaw)));
	p_pos->set_y(c_y+(offset*sin(yaw)));
}

void odom::UpdatePos(const tick& t,const robot_config& cfg){
	if(t.right()==t.left()){
		UpdateS(t,cfg);
		return;
	}
	UpdateC(t,cfg);
} 
