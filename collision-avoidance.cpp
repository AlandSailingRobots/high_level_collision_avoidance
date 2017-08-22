#include "vibes.h"
#include "ibex.h"
#include "functions.h"
#include "json.hpp"
#include <fstream>

using namespace ibex;
using namespace std;
using json = nlohmann::json;




int main(int argc, char** argv){
    ifstream input("config.json");
    json config;
    input >> config;

    //navigation zone :
    vector<vector<vector<double>>> borderList = config["borderList"];

    //coordinate {x,y} of the waypoints
    vector<vector<double>> waypoints = config["waypoints"];
    vector<vector<double>> formerWpts = waypoints;
    //initial boat speed for each segment
    vector<Interval> boatSpeed(waypoints.size()-1, Interval(config["defaultSpeedInterval"][0],config["defaultSpeedInterval"][1]));

    vibes::beginDrawing();
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
    IntervalVector obsi(4);
    vector<IntervalVector> obstacles;
    for (int i = 0; i < config["obstaclesInfos"].size(); i++){
        obsi[0] = Interval(config["obstaclesInfos"][i][0][0], config["obstaclesInfos"][i][0][1]);
        obsi[1] = Interval(config["obstaclesInfos"][i][1][0], config["obstaclesInfos"][i][1][1]);
        obsi[2] = Interval(config["obstaclesInfos"][i][2][0], config["obstaclesInfos"][i][2][1]);
        obsi[3] = Interval(config["obstaclesInfos"][i][3][0], config["obstaclesInfos"][i][3][1]);
        obstacles.push_back(obsi);
        
    }

    /*initial position of the boat, by default on the 1st waypoint */
    double boatInitPosUncertaintySize = config["boatInitPosUncertaintySize"];
    double boatStateBounds[2][2] = {{waypoints[0][0] - boatInitPosUncertaintySize/2., waypoints[0][0] + boatInitPosUncertaintySize/2.}, {waypoints[0][1] - boatInitPosUncertaintySize/2., waypoints[0][1] + boatInitPosUncertaintySize/2.}};
    IntervalVector boatState(2, boatStateBounds);

    functions::manageCollision(waypoints, boatState, boatSpeed, obstacles, borderList);

    cout << "after collision avoidance" << endl;
    cout << "waypoints :" << endl;
    for (int i = 0; i < waypoints.size(); i++){
        cout << waypoints[i][0] << "," << waypoints[i][1] << endl;
    }


    cout << "speed :" << endl;
    for (int i = 0; i< boatSpeed.size(); i++){
        cout << boatSpeed[i] << endl;
    }

    functions::drawTrajectory(formerWpts, waypoints, boatSpeed, boatState, obstacles, borderList);

    return 0;
}