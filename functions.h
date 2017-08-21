#include "ibex.h"
#include "vibes.h"
#include "interval_tools.h"

namespace functions{
void manageCollision(std::vector<std::vector<double>>& waypoints, ibex::IntervalVector boatState, std::vector<ibex::Interval>& boatSpeed, std::vector<ibex::IntervalVector> obstacles, std::vector<std::vector<std::vector<double>>> borderList);

void pathReplanning(double& boatHead, ibex::Interval& speed, ibex::IntervalVector boatState, ibex::Interval timeInterval, std::vector<ibex::IntervalVector> obstacles, std::vector<std::vector<std::vector<double>>> borderList);

void waypointManagement(double boatHead, std::vector<ibex::Interval>& boatSpeed, ibex::IntervalVector boatState, double endTime, std::vector<std::vector<double>>& waypoints, int currentSegmentIndex);

void drawTrajectory(std::vector<std::vector<double>> waypoints, std::vector<ibex::Interval> boatSpeed, ibex::IntervalVector boatState, std::vector<ibex::IntervalVector> obstacles);

double computeHeading(ibex::Interval XspeedComponent, ibex::Interval YspeedComponent);
}