#include "vibes.h"
#include "ibex.h"
#include "functions.h"

using namespace ibex;
using namespace std;




int main(int argc, char** argv){

    //navigation zone :
    vector<vector<vector<double>>> borderList = {{{-200, 0}, {-300, 0},{0, 200},{200, 0}, {300, 0}, {0, -200}}, {{-60, 0}, {-60, 50}, {-110, 50}, {-110, 0}}};

    //coordinate {x,y} of the waypoints
    vector<vector<double>> waypoints = {{0,0},{40,50},{-10,60},{-5,100},{10,120}};
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
    vector<IntervalVector> obstacles;
    double pos1[4][2] = {{1.5, 1.7}, {30, 32}, {-10, -8}, {2, 2.1}}; // {speed, posInitx, posInity, heading}
    double pos2[4][2] = {{2, 2.3}, {10, 12}, {0, 2}, {1.6, 1.7}};
    IntervalVector obs1(4, pos1);
    IntervalVector obs2(4, pos2);
    obstacles.push_back(obs1);
    obstacles.push_back(obs2);

    /*initial position of the boat, /!\ it must be somewhere in the segment between the 2 firsts waypoints /!\ */
    double _boatState[2][2] = {{-1,1}, {-1,1}};
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