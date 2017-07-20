#include "ibex.h"
#include "vibes.h"


ibex::Interval left(const ibex::Interval& x);

ibex::Interval right(const ibex::Interval& x);

ibex::IntervalVector left(const ibex::IntervalVector& X);

ibex::IntervalVector right(const ibex::IntervalVector& X);

void paving(ibex::IntervalVector X, std::vector<ibex::SepInter*> listSep, std::vector<ibex::IntervalVector>& listBoxes);

bool collisionCondition(ibex::Interval v, ibex::Interval x0, ibex::Interval y0, ibex::Interval th, ibex::Interval vi, ibex::Interval xi, ibex::Interval yi, ibex::Interval thi, ibex::Interval t);

ibex::IntervalVector findClosest(std::vector<ibex::IntervalVector> listBoxes, ibex::IntervalVector boatSpeed);