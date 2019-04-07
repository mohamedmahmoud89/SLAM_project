#include"file_mgr.h"
//#include<fstream>
#include<sstream>
//#include<algorithm>
//#include<iostream>
//#include<memory>
#include"file_open.h"
using namespace std;

// general Read 
template<typename T>void ReadFileMgr<T>::read(const string& fn){
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

template<typename T>vector<T> ReadFileMgr<T>::get_data() const noexcept{
        return data;
}

// Motor
void MotorFileMgr::read_line(const string& line){
	stringstream ss(line);
        string Left,Right;
        for(int i=0;i<3;++i)
             ss>>Left;
        for(int i=0;i<4;++i)
             ss>>Right;
	Tick current(stoul(Left),stoul(Right));
        if(is_first_Tick){
		is_first_Tick=false;
		last_Tick=current;
		return;
	}
	Tick temp(
			current.Left()-last_Tick.Left(),
			current.Right()-last_Tick.Right());
	data.push_back(temp);
	last_Tick=current;
}

MotorFileMgr* MotorFileMgr::get_instance() noexcept{
	static MotorFileMgr g_mfm;
	return &g_mfm;
}

//Scan
void ScanFileMgr::read_line(const string& line){

}

ScanFileMgr* ScanFileMgr::get_instance() noexcept{
        static ScanFileMgr g_sfm;
        return &g_sfm;
}

// general Write
template<typename T>void WriteFileMgr<T>::add_sample(const T& sample){
	data.push_back(sample);
}

template<typename T>void WriteFileMgr<T>::write(
		const string& filename) const
{
	ofstream fs;
	{
		file_open fo(&fs,filename);
		if(fs.is_open()){
			for(const T& sample:data){
				write_line(fs,sample);
			}
		}
	}
}

// Pos
void PosFileMgr::write_line(ofstream& ofs,const Pose& pos) const{
	ofs<<"F"<<" ";
        ofs<<pos.X()<<" ";
        ofs<<pos.Y()<<" ";
        ofs<<pos.Yaw()<<endl;
}

// Feature
void FeatFileMgr::write_line(
		ofstream& ofs,
		const Feature& feat) const{
	
	ofs<<"D C ";
	ofs<<feat.X()<<" ";
	ofs<<feat.Y()<<" "<<endl;
}
