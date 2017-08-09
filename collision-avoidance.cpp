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
    //initial boat speed for each segment
    vector<Interval> boatSpeed(waypoints.size()-1, Interval(2,2.5));

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

    vector<double> drawx, drawy;

    for (int i=0; i< waypoints.size(); i++){
        drawx.push_back(waypoints[i][0]);
        drawy.push_back(waypoints[i][1]);
    }
    vibes::newFigure("path");
    vibes::setFigureProperties(vibesParams("x",0 , "y", 0, "width", 800, "height", 800));
    vibes::drawLine(drawx, drawy, "yellow");

    
    for (int i=0; i<borderList.size(); i++){
        drawx.resize(0);
        drawy.resize(0);
        for (int j=0; j<=borderList[i].size(); j++){
            drawx.push_back(borderList[i][j%borderList[i].size()][0]);
            drawy.push_back(borderList[i][j%borderList[i].size()][1]);
        }
        vibes::drawLine(drawx, drawy, "red");
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
    double _boatState[2][2] = {{waypoints[0][0]-1, waypoints[0][0]+1}, {waypoints[0][1]-1, waypoints[0][1]+1}};
    IntervalVector boatState(2, _boatState);

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
    vibes::selectFigure("path");

    drawx.resize(0);
    drawy.resize(0);
    for (int i=0; i< waypoints.size(); i++){
        drawx.push_back(waypoints[i][0]);
        drawy.push_back(waypoints[i][1]);
    }
    vibes::drawLine(drawx, drawy, "blue");

    functions::drawTrajectory(waypoints, boatSpeed, boatState, obstacles);

    return 0;
}