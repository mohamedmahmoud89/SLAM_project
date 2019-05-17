#ifndef FUS_FEAT_H
#define FUS_FEAT_H
#include "common.h"
#include "robot_cfg.h"
#include<unordered_map>
using namespace std;

namespace Feature{
class FeatBase{
protected: 
	f32 x{0};
        f32 y{0};
        size_t id{0};
public:
        FeatBase():x(0),y(0),id(0){}
        FeatBase(
                const f32 x_mm,
                const f32 y_mm,
                const size_t uid):x(x_mm),y(y_mm),id(uid){}
        virtual ~FeatBase(){}
	virtual f32 X() const noexcept{return x;}
        virtual f32 Y() const noexcept{return y;}
        virtual size_t Id() const noexcept{return id;}
	void Set_X(f32 in){x=in;}
	void Set_Y(f32 in){y=in;}
};

class FeatList{
	SmrtPtrVec<FeatBase> features;
public:
	const SmrtPtrVec<FeatBase>& Data() const noexcept{
		return features;}
	void Push_Back(const shared_ptr<FeatBase>& p_ft)
	{
		features.push_back(p_ft);
	}
};

class FeatAssoc{
        public:
                unordered_map<size_t,shared_ptr<FeatBase>>scanned_t;
                unordered_map<size_t,shared_ptr<FeatBase>>stored_t;
                unordered_map<size_t,size_t> assocs_t;
		/*AssocTables(const AssocTables& rhs):
			scanned(move(rhs.scanned)),
			stored(move(rhs.stored)),
			assocs(move(rhs.assocs)){}
		AssocTables& operator=(const AssocTables& rhs){
			scanned=move(rhs.scanned);
			stored=move(rhs.stored);
			assocs=move(rhs.assocs);
			return *this;
		}*/
};

unique_ptr<FeatAssoc> FeatAssociate(
                const SmrtPtrVec<FeatBase>& scanned,
                const SmrtPtrVec<FeatBase>& stored);


void FeatureTransform(
		FeatBase& feat,
		const PoseBase& coord,
		const Robot::RobotConfig& cfg);
};
#endif
