#include "ibex.h"
#include "vibes.h"


ibex::Interval left(const ibex::Interval& x);

ibex::Interval right(const ibex::Interval& x);

ibex::IntervalVector left(const ibex::IntervalVector& X);

ibex::IntervalVector right(const ibex::IntervalVector& X);

void paving(ibex::IntervalVector X, std::vector<ibex::SepInter*> listSep, std::vector<ibex::IntervalVector>& listBoxes);

void createSepBorder(std::vector<std::vector<double>> border, std::vector <ibex::SepInter*> &listSep, ibex::IntervalVector boatInitPos, ibex::Interval T);

bool collisionCondition(ibex::Interval v, ibex::Interval x0, ibex::Interval y0, double th, ibex::Interval vi, ibex::Interval xi, ibex::Interval yi, ibex::Interval thi, ibex::Interval t);
/*
bool crossBroder(ibex::Interval v, ibex::interval x0, ibex::interval y0, double th, ibex::Interval t, std::vector<double> border1, std::vector<double> border2);
*/
double midPointDistance(ibex::IntervalVector X, ibex::IntervalVector Y);

ibex::IntervalVector findClosest(std::vector<ibex::IntervalVector> listBoxes, ibex::IntervalVector boatSpeed);