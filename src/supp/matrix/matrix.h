#include<vector>
#include "common.h"
using namespace std;

template<typename T>
class Matrix{
	vector<vector<T>> m;
public:
	Matrix()=delete;
	Matrix(const u8 rows,const u8 cols):
		m(rows,vector<T>(cols,0)){}
};
