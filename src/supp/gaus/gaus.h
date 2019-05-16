#include "common.h"
#include "matrix.h"
#include<memory>
using namespace std;

template<typename T>
class Gaussian{
        unique_ptr<T> mean;
        Matrix<f32> covariance;
public:
        Gaussian(const u8 state_space):
		mean(make_unique<T>()),
		covariance(state_space,state_space){};
	Gaussian()=delete;
};

