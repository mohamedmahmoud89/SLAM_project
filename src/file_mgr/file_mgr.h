#include<vector>
#include<string>
#include<iostream>
#include "common.h"
using namespace std;

//design is not good. why do we need class hirarchy here?
//isn't MotorFileMgr class (and other classes for scans and so on) enough?

/*class GenFileMgr{
public:
	virtual void read(const string& filename)=0;
	virtual vector<tick> get_ticks() const=0;
	virtual ~GenFileMgr(){}
};*/


// in case of running on mutli threaded system
// a mutex needs to be added to the class definition for locking
// the file
// singleton design ppattern is good in case only one file in the
// system which is the case for "robot_motor.txt"
class MotorFileMgr{
private:
	MotorFileMgr():is_first_tick(true),last_tick(tick(0,0)){};

public:
	MotorFileMgr(const MotorFileMgr& rhs)=delete;
	MotorFileMgr& operator=(const MotorFileMgr& rhs)=delete;
	void read(const string& filename);
	vector<tick> get_ticks() const noexcept;
	~MotorFileMgr(){}
	static MotorFileMgr* get_instance() noexcept;
private:
	void read_line(const string& line);
	vector<tick> ticks;
	tick last_tick;
	bool is_first_tick{true};
};

class PosFileMgr{
public:
	void add_pos(const pose& pos);
	void write(const string& filename) const;
private:
	vector<pose> poses;
};
