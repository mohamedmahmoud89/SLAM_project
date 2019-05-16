#include"feat.h"
#include<cmath>
using namespace std;
using namespace Feature;

void FeatureTransform(
		FeatBase& feat,
		const PoseBase& coord){
	f32 x(coord.X()+feat.X()*cos(coord.Yaw()));
	x-=(feat.Y()*sin(coord.Yaw()));
	feat.Set_X(x);
	f32 y(coord.Y()+feat.X()*sin(coord.Yaw()));
	y+=(feat.Y()*cos(coord.Yaw()));
	feat.Set_Y(y);
}
const FeatAssoc::AssocTables& FeatAssoc::Update_Assocs(
		const FeatList& scanned,
		const FeatList& stored){
	// TODO: do the assoc magic

	return t;
}
