#ifndef PTR_VEC_H
#define PTR_VEC_H
#include<vector>
#include<memory>
using namespace std;
template<typename T>
class SmrtPtrVec{
	vector<shared_ptr<T>> list;
public:
	/*SmrtPtrVec()=default;
	SmrtPtrVec(const vector<shared_ptr<T>>&in_list):
		list(in_list){}
	SmrtPtrVec(const SmrtPtrVec& rhs):
		list(rhs.list){}
	SmrtPtrVec& operator=(const SmrtPtrVec& rhs){
		list=rhs.list;
		return *this;
	}
	SmrtPtrVec(SmrtPtrVec&& rhs):list(move(rhs.list)){}
	SmrtPtrVec& operator=(SmrtPtrVec&& rhs){
		list=move(rhs.list);
		return *this;
	}*/
	const vector<shared_ptr<T>>& List() const noexcept
	{
		return list;
	}
	void Push_Back(const shared_ptr<T>& ptr)
	{
		list.push_back(ptr);
	}
	const shared_ptr<T>& operator[](const size_t idx)const
	{
		return list[idx];
	}
};
#endif
