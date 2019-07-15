#include"motion_model.h"
#include<cmath>
#include"common.h"
using namespace std;
using namespace Robot;
using namespace Motion;
void MMSimple::UpdateS(
		PoseBase& pos,
		const ControlBase& t,
		const RobotConfig& cfg)const{
        f32 dist(t.Right_Tick()*cfg.Ticks_ToMm());
        f32 x(pos.X());
        f32 y(pos.Y());
        f32 yaw(pos.Yaw());
        pos.set_x(x+(dist*cos(yaw)));
        pos.set_y(y+(dist*sin(yaw)));
}

void MMSimple::UpdateC(
		PoseBase& pos,
		const ControlBase& t,
		const RobotConfig& cfg)const{
        f32 t2mm(cfg.Ticks_ToMm());
        f32 dist_r(t.Right_Tick()*t2mm);
        f32 dist_l(t.Left_Tick() *t2mm);
        f32 width(cfg.Width());
        f32 offset(cfg.Sensor_Offset());
        f32 x(pos.X());
        f32 y(pos.Y());
        f32 yaw(pos.Yaw());

        f32 alpha((dist_r-dist_l)/width);
        f32 radius(dist_l/alpha);
        // this lines are commented because the original 
	// implementation for Unit A was tracking the scanner pos
	// while the rest of the course is tracking the robot pos
	//
	//f32 r_x(x-(offset*cos(yaw)));
        //f32 r_y(y-(offset*sin(yaw)));
        f32 r_x(pos.X());
	f32 r_y(pos.Y());
	f32 rot_rad(radius+(width/2.0));
        f32 c_x(r_x-(rot_rad*sin(yaw)));
        f32 c_y(r_y+(rot_rad*cos(yaw)));
        yaw+=alpha;
        while(yaw>=2*M_PI)
                yaw-=2*M_PI;
        pos.set_yaw(yaw);
        c_x+=rot_rad*sin(yaw);
        c_y-=rot_rad*cos(yaw);
        // this lines are commented because the original 
	// implementation for Unit A was tracking the scanner pos
	// while the rest of the course is tracking the robot pos
	//
        //pos.set_x(c_x+(offset*cos(yaw)));
        //pos.set_y(c_y+(offset*sin(yaw)));
	pos.set_x(c_x);
	pos.set_y(c_y);
}

void MMSimple::UpdatePos(
		PoseBase& pos,
		const ControlBase& c,
		const Config& cfg)const{
	if(c.Right_Tick()==c.Left_Tick()){
                UpdateS(
			pos,
			c,
			dynamic_cast<const RobotConfig&>(cfg));
                return;
        }
        UpdateC(
		pos,
		c,
		dynamic_cast<const RobotConfig&>(cfg));
}
