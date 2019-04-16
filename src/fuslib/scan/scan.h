#include<vector>

using namespace std;

namespace Scan{
	class Config{
	public:
		virtual u8 Min_ValidDepth() const noexcept=0;	
	};
	template<typename T>
	class ScanBase{
        	vector<T> depth;
	public:
	        ScanBase(const vector<T>&scan):depth(scan){}
        	virtual vector<T> Depth() const noexcept
        	{
                	return depth;
        	}
	};
};
