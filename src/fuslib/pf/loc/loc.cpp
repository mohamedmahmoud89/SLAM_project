#include "loc.h"
#include <random>
#include <cassert>
#include "common.h"
using namespace std;

vector<f32> LocPF::Calc_ImpWeights(
                        FeatList& feats,
                        RefList& refs,
                        const Robot::Config& cfg){
	vector<f32>ret;
	const f32 max_ref_dist(numeric_limits<f32>::max());
	const u8 num_landmarks(6);
	for(auto& pp:particles){
		const PoseBase particle(*pp);
		for(auto&feat:feats.Data()){
                        // transform the features to world coords
                        // based on the particle pos
                        FeatureGlobalTransform(
                                        *feat,particle,cfg);
                }
		unique_ptr<FeatAssoc> assocs(
                        FeatAssociate(
				feats.Data(),
				refs,
				max_ref_dist));
	//	assert(assocs->assocs_t.size()==num_landmarks);
		f32 weight(1);
		for(auto&assoc:assocs->assocs_t){
			auto p_ref=assocs->stored_t.find(
					assoc.second)->second;
			auto p_meas=assocs->scanned_t.find(
					assoc.first)->second;

			FeaturePolarTransform(
					*p_ref,particle,cfg);

			// sample pdf using polar coords diff
			f32 delta_dst(p_ref->R()-p_meas->R());
			f32 delta_ang(p_ref->Theta()-p_meas->Theta());
			
			//ang normalization
			f32 sign(delta_ang/fabs(delta_ang));
			delta_ang+=sign*M_PI;
			delta_ang=fmod(delta_ang,2*M_PI);
			delta_ang-=sign*M_PI;
			
			f32 nd1(normal_pdf(delta_dst,0,meas_dist_std));
			f32 nd2(normal_pdf(delta_ang,0,meas_ang_std));
			weight*=(nd1*nd2);
		}
		ret.push_back(weight);
	}
	return ret;
}

void LocPF::Update(
		FeatList& feats,
		RefList& refs,
		const Robot::Config& cfg){
	auto weights(Calc_ImpWeights(feats,refs,cfg));
	Resample(weights);
}
