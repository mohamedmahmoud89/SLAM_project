#ifndef FILE_MGR_H
#define FILE_MGR_H
#include<vector>
#include<string>
//#include<iostream>
#include<fstream>
#include "common.h"
#include"ctrl_data.h"
#include"scan.h"
#include"feat.h"
using namespace std;
using namespace Feature;
template<typename T>
class ReadFileMgr{
public:
	ReadFileMgr()=default;
	ReadFileMgr(const ReadFileMgr& rhs)=delete;
	ReadFileMgr& operator=(const ReadFileMgr& rhs)=delete;
	virtual void read(const string& filename);
	virtual vector<T> get_data() const noexcept;
	virtual ~ReadFileMgr(){}
protected:
	virtual void read_line(const string& line)=0;
	vector<T> data;
};

// in case of running on mutli threaded system
// a mutex needs to be added to the class definition for locking
// the file
// singleton design ppattern is good in case only one file in the
// system which is the case for "robot_motor.txt"
class MotorFileMgr :public ReadFileMgr<ControlBase>{
	MotorFileMgr():
		is_first_Tick(true),
		last_Tick(ControlBase(0,0)){};
public:
	static MotorFileMgr* get_instance() noexcept;
private:
	void read_line(const string& line) override;
	ControlBase last_Tick;
	bool is_first_Tick{true};
};

class ScanFileMgr : public ReadFileMgr<Scan::Scan>{
	ScanFileMgr()=default;
public:
	static ScanFileMgr* get_instance() noexcept;
private:
	void read_line(const string& line) override;
};

template<typename T>
class WriteFileMgr{
public:
	virtual void add_sample(const T& sample);
	// here we can move write() to a base class and privately
	// inhirint it to avoid code bloat
	virtual void write(const string& filename) const;
protected:
	virtual void write_line(ofstream& ofs,const T& sample)const=0;
	vector<T>data;
};

class PosFileMgr : public WriteFileMgr<PoseBase>{
	void write_line(
			ofstream& ofs,
			const PoseBase& pos) const override;
};

class FeatFileMgr :public WriteFileMgr<Feature::FeatList>{
	void write_line(
		ofstream& ofs,
		const Feature::FeatList& feat) const override;
};
#endif
