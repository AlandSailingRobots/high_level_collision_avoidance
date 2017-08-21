#include "ibex.h"
#include "vibes.h"
#include <memory>


ibex::Interval left(const ibex::Interval& x);

ibex::Interval right(const ibex::Interval& x);

ibex::IntervalVector left(const ibex::IntervalVector& X);

ibex::IntervalVector right(const ibex::IntervalVector& X);

void paving(ibex::IntervalVector X, std::vector<std::shared_ptr<ibex::SepInter>> listSep, std::vector<ibex::IntervalVector>& listBoxes, double epsilon);

void createSepBorder(std::vector<std::vector<double>> border, std::vector<std::shared_ptr<ibex::SepInter>> &listSep, ibex::IntervalVector boatInitPos, ibex::Interval T, std::vector<std::shared_ptr<ibex::Function>> &deleteFunc, std::vector<std::shared_ptr<ibex::SepFwdBwd>> &deleteInitialSep);

bool collisionCondition(ibex::Interval v, ibex::Interval x0, ibex::Interval y0, double th, ibex::Interval vi, ibex::Interval xi, ibex::Interval yi, ibex::Interval thi, ibex::Interval t);

bool crossBorder(ibex::Interval v, ibex::Interval x0, ibex::Interval y0, double th, double t, std::vector<double> border1, std::vector<double> border2);

double midPointDistance(ibex::IntervalVector X, ibex::IntervalVector Y);

ibex::IntervalVector findClosest(std::vector<ibex::IntervalVector> listBoxes, ibex::IntervalVector boatSpeed);