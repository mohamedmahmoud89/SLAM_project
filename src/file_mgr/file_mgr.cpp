#include"file_mgr.h"
#include<fstream>
#include<sstream>
//#include<algorithm>
//#include<iostream>
//#include<memory>
#include"file_open.h"
using namespace std;

// Motor

void MotorFileMgr::read_line(const string& line){
	stringstream ss(line);
        string left,right;
        for(int i=0;i<3;++i)
             ss>>left;
        for(int i=0;i<4;++i)
             ss>>right;
	tick current(stoul(left),stoul(right));
        if(is_first_tick){
		is_first_tick=false;
		last_tick=current;
		return;
	}
	tick temp(
			current.left()-last_tick.left(),
			current.right()-last_tick.right());
	ticks.push_back(temp);
	//cout<<temp.left()<<" "<<temp.right()<<endl;
	last_tick=current;
}

void MotorFileMgr::read(const string& fn){
	ifstream file;
	string line;
	{
		file_open fl(&file,fn);
		// needs lock in case of running async
		if(file.is_open())
			while(getline(file,line)){
				read_line(line);
			}
	}
}

vector<tick> MotorFileMgr::get_ticks() const noexcept{
	return ticks;
}

MotorFileMgr* MotorFileMgr::get_instance() noexcept{
	static MotorFileMgr g_mfm;
	return &g_mfm;
}

// Pose
void PosFileMgr::add_pos(const pose& pos){
	poses.push_back(pos);
}

void PosFileMgr::write(const string& filename) const{
	ofstream fs;
	{
		file_open fo(&fs,filename);
		if(fs.is_open()){
			for(const pose&p:poses){
				fs<<"F"<<" ";
				fs<<p.get_x()<<" ";
				fs<<p.get_y()<<" ";
				fs<<p.get_yaw()<<endl;
			}
		}
	}
}
