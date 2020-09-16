#include "slam.h"

void FastSlamPF::Update(
                FeatList& feats,
                const Robot::Config& cfg){
        auto weights(Calc_ImpWeights(feats,cfg));
        Resample(weights);
}

vector<f32> FastSlamPF::Calc_ImpWeights(
		FeatList& feats,
		const Robot::Config& cfg){
        vector<f32>ret;
        for(int i=0;i<particles.size();++i){
                f32 weight(1);
                for(auto&feat:feats.Data()){
			vector<f32> likelihood;
			for(int j=0;j<map[i].size();++j){
				likelihood.push_back(
					MaxLikeFeatAssociate(
						particles[i],
						feat,
						map[i][j]));
			}

			// obtain the max likelihood idx
			auto it=max_element(likelihood.begin(),likelihood.end());
			
			// update weight
			weight*= *it;

			// check to either add a new landmark or update an existing
			if(*it>min_likelihood){
				Update_Landmark(feat,it-likelihood.begin());
			}
			else{
				Insert_Landmark(feat);
			}
                }
                ret.push_back(weight);
        }
        return ret;
}

void FastSlamPF::Update_Landmark(const shared_ptr<FeatBase>& feature,int idx){

}
void FastSlamPF::Insert_Landmark(const shared_ptr<FeatBase>& feature){

}
