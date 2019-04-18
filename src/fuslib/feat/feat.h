#ifndef FUS_FEAT_H
#define FUS_FEAT_H
#include "common.h"
#include"ptr_vec.h"
#include<unordered_map>
using namespace std;

namespace Feature{
class FeatBase{
        f32 x{0};
        f32 y{0};
        size_t id{0};
public:
        FeatBase():x(0),y(0),id(0){}
        FeatBase(
                const f32 x_mm,
                const f32 y_mm,
                const size_t uid):x(x_mm),y(y_mm),id(uid){}
        virtual f32 X() const noexcept{return x;}
        virtual f32 Y() const noexcept{return y;}
        virtual size_t Id() const noexcept{return id;}
};

class FeatList{
	SmrtPtrVec<FeatBase> features;
public:
	/*FeatList()=default;
	FeatList(const FeatList& rhs):features(rhs.features){}
	FeatList& operator=(const FeatList& rhs)
	{
		features=rhs.features;
		return *this;
	}
	FeatList(FeatList&& rhs):features(move(rhs.features)){}
	FeatList& operator=(FeatList&& rhs){
		features=move(rhs.features);
		return *this;
	}*/
	const SmrtPtrVec<FeatBase>& Data() const noexcept{
		return features;}
	void Push_Back(const shared_ptr<FeatBase>& p_ft)
	{
		features.Push_Back(p_ft);
	}
};

class FeatAssoc{
public:
        class AssocTables{
        public:
                unordered_map<size_t,shared_ptr<FeatBase>>scanned;
                unordered_map<size_t,shared_ptr<FeatBase>>stored;
                unordered_map<size_t,size_t> assocs;
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
        const AssocTables& Update_Assocs(
                const FeatList& scanned,
                const FeatList& stored);
private:
	AssocTables t;
};
};
#endif
