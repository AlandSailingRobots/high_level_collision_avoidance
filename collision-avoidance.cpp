#include "vibes.h"
#include "ibex.h"
#include "functions.h"

using namespace ibex;
using namespace std;




int main(int argc, char** argv){
    //coordinate {x,y} of the waypoints
    vector<vector<double>> waypoints = {{0,0},{40,50},{-10,60},{-5,100},{10,120}};
    //initial boat speed for each segment
    vector<Interval> boatSpeed(waypoints.size()-1, Interval(2,2.5));

    cout << "before collision avoidance" << endl;
    cout << "waypoints :" << endl;
    for ( int i = 0; i < waypoints.size(); i++){
        cout << waypoints[i][0] << "," << waypoints[i][1] << endl;
    }
    cout << "speed :" << endl;
    for (int i = 0; i < boatSpeed.size(); i++){
        cout << boatSpeed[i] << endl;
    }

    //informations about obstacles
    vector<IntervalVector> obstacles;
    double pos1[4][2] = {{1.5, 1.7}, {30, 35}, {-10, -8}, {2, 2.1}}; // {speed, posInitx, posInity, heading}
    double pos2[4][2] = {{3, 3.5}, {10, 12}, {0, 3}, {1.5, 1.6}};
    IntervalVector obs1(4, pos1);
    IntervalVector obs2(4, pos2);
    obstacles.push_back(obs1);
    obstacles.push_back(obs2);

    /*initial position of the boat, /!\ it must be somewhere in the segment between the 2 firsts waypoints /!\ */
    double _boatState[2][2] = {{-1,1}, {-1,1}};
    IntervalVector boatState(2, _boatState);

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