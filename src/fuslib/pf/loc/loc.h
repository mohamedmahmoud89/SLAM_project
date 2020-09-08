#ifndef LOC_H
#define LOC_H
#include "common.h"
#include "motion_model.h"
#include "ctrl_data.h"
#include "feat.h"
#include "pf.h"
#include <vector>

using namespace std;
using namespace Motion;
using namespace Feature;

class LocPF : public ParticleFilter{
public:
        LocPF()=delete;
        LocPF(
                const SmrtPtrVec<PoseBase>& ps,
                const f32 control_motion,
                const f32 control_turn,
                const f32 meas_dist_stddev,
                const f32 meas_ang_stddev):
                ParticleFilter(ps,control_motion,control_turn,meas_dist_stddev,meas_ang_stddev){}
        typedef SmrtPtrVec<FeatBase> RefList;
        void Update(
                FeatList& feats,
                RefList& refs,
                const Robot::Config& cfg);
private:
        vector<f32> Calc_ImpWeights(
                        FeatList& feats,
                        RefList& refs,
                        const Robot::Config& cfg);
};

#endif

