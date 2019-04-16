#include "common.h"
#include "matrix.h"
#include<memory>
using namespace std;

template<typename T,const u8 state_space>
class Gaussian{
        unique_ptr<T> mean;
        Matrix<f32,state_space,state_space> covariance;
public:
        Gaussian(){}
};

