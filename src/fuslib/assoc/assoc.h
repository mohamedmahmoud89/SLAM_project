#include"common.h"
#include<unordered_map>
using namespace std;
class FeatAssocBase{
	unordered_map<size_t,size_t>assocs;
public:
	virtual void Update_Assocs(
		const unordered_map<size_t,FeatBase>&scanned,
		const unordered_map<size_t,FeatBase>&stored);
};
