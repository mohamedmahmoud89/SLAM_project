#include<vector>
#include<string>
//#include<iostream>
#include<fstream>
#include "common.h"
using namespace std;

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
class MotorFileMgr :public ReadFileMgr<tick>{
	MotorFileMgr():is_first_tick(true),last_tick(tick(0,0)){};
public:
	static MotorFileMgr* get_instance() noexcept;
private:
	void read_line(const string& line) override;
	tick last_tick;
	bool is_first_tick{true};
};

class ScanFileMgr : public ReadFileMgr<vector<u16>>{
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
	virtual void write(const string& filename) const;
protected:
	virtual void write_line(ofstream& ofs,const T& sample)const=0;
	vector<T>data;
};

class PosFileMgr : public WriteFileMgr<pose>{
	void write_line(ofstream& ofs,const pose& pos) const override;
};
