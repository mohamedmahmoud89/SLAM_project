#include "common.h"
#include "matrix.h"
#include<memory>
using namespace std;

template<typename T>
class Gaussian{
        shared_ptr<T> mean;
        Matrix<f32> covariance;
public:
        Gaussian(
		const T& init_state,
		const Matrix<f32>& init_covar):
		mean(make_unique<T>(init_state)),
		covariance(init_covar){};
	Gaussian()=delete;
	shared_ptr<T> Mean()const noexcept{return mean;}
};

