#include "ibex.h"
#include "vibes.h"
#include "interval_tools.h"
#include "functions.h"

using namespace ibex;
using namespace std;

void manageCollision(vector<double[2]>& waypoints, IntervalVector boatState, vector<IntervalVector> obstacles){
    Interval boatHead;
    boatHead = acos(waypoints[1][0]/(sqrt(pow(waypoints[1][0],2)+pow(waypoints[1][1],2))));

    Interval T(0,0);

    for ( int i = 1; i< waypoints.size(); i++){
        boatState[1] = boatState[0]*cos(boatHead)*T.ub() + boatState[1];
        boatState[2] = boatState[0]*sin(boatHead)*T.ub() + boatState[2];

        for (int i = 0; i<2; i++){
            obstacles[i][1] = obstacles[i][0]*cos(obstacles[i][3])*T.ub() + obstacles[i][1];
            obstacles[i][2] = obstacles[i][0]*sin(obstacles[i][3])*T.ub() + obstacles[i][2];
        }

        T = Interval(0, (sqrt(pow(waypoints[i][0] - waypoints[i-1][0],2)+pow(waypoints[i][1] - waypoints[i-1][1],2)))/boatState[0].mid());

        boatHead = acos((waypoints[i][0] - waypoints[i-1][0])/(sqrt(pow(waypoints[i][0] - waypoints[i-1][0],2)+pow(waypoints[i][1] - waypoints[i-1][1],2))));

        for (int i = 0; i < obstacles.size(); i++){
            if (collisionCondition(boatState[0], boatState[1], boatState[2], boatHead, obstacles[i][0], obstacles[i][1], obstacles[i][2], obstacles[i][3], T)){
                //pathReplanning(/*TODO*/);
            }
        }
    }
}