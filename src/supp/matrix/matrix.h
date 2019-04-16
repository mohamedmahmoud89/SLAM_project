#include<vector>
#include "common.h"
using namespace std;

template<typename T,const u8 rows,const u8 cols>
class Matrix{
	vector<vector<T>> m;
public:
	Matrix():m(rows,vector<T>(cols,0)){}
};
