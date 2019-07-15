#ifndef GAUS_H
#define GAUS_H
#include "common.h"
#include <Eigen/Dense>
#include<memory>
using namespace std;
using namespace Eigen;
template<typename T>
class Gaussian{
        shared_ptr<T> mean;
        shared_ptr<MatrixXf> covariance;
public:
        Gaussian(
		const T& init_state,
		const MatrixXf& init_covar):
		mean(make_shared<T>(init_state)),
		covariance(make_shared<MatrixXf>(init_covar)){};
	Gaussian()=delete;
	Gaussian(const Gaussian& rhs):
		mean(make_shared<T>(*rhs.mean)),
		covariance(make_shared<MatrixXf>(*rhs.covariance)){};
	shared_ptr<T> Mean()const noexcept{return mean;}
	shared_ptr<MatrixXf> Covariance() const {return covariance;}
};
#endif
