#include "vibes.h"
#include "ibex.h"
#include "functions.h"

using namespace ibex;
using namespace std;




int main(int argc, char** argv){
    vector<vector<double>> waypoints = {{0,0},{40,50},{-10,70},{-5,120}};
    vector<Interval> boatSpeed(waypoints.size()-1,Interval(2,2.5));

    cout << "before collision avoidance" << endl;
    cout << "waypoints :" << endl;
    for ( int i = 0; i < waypoints.size(); i++){
        cout << waypoints[i][0] << "," << waypoints[i][1] << endl;
    }
    cout << "speed :" << endl;
    for (int i = 0; i < boatSpeed.size(); i++){
        cout << boatSpeed[i] << endl;
    }


    vector<IntervalVector> obstacles;
    double pos1[4][2] = {{1.5, 1.7}, {30, 35}, {-10, -8}, {2, 2.1}}; // {speed, posInitx, posInity, heading}
    double pos2[4][2] = {{3, 3.5}, {10, 12}, {0, 3}, {0.4, 0.5}};
    IntervalVector obs1(4, pos1);
    IntervalVector obs2(4, pos2);
    obstacles.push_back(obs1);
    obstacles.push_back(obs2);

    IntervalVector boatState(2);
    boatState[0] = Interval(-1,1);
    boatState[1] = Interval(-1,1);


    functions::manageCollision(waypoints, boatState, boatSpeed, obstacles);

    cout << "after collision avoidance" << endl;
    cout << "waypoints :" << endl;
    for (int i = 0; i < waypoints.size(); i++){
        cout << waypoints[i][0] << "," << waypoints[i][1] << endl;
    }


    cout << "speed :" << endl;
    for (int i = 0; i< boatSpeed.size(); i++){
        cout << boatSpeed[i] << endl;
    }

    return 0;
}