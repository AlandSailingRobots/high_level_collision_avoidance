#include "ibex.h"
#include "vibes.h"
#include "interval_tools.h"
#include "functions.h"

using namespace ibex;
using namespace std;

namespace functions{
void manageCollision(vector<vector<double>>& waypoints, IntervalVector boatState, vector<Interval>& boatSpeed, vector<IntervalVector> obstacles,vector<vector<vector<double>>> borderList){
    double boatHead;
    //assume that the boat heading will be aligned with the 2 waypoints
    boatHead = acos(waypoints[1][0]/(sqrt(pow(waypoints[1][0],2)+pow(waypoints[1][1],2))));

    Interval T;

    vector<double> drawx, drawy;

    //for each segment of the trajectory 
    for ( int i = 1; i< waypoints.size(); i++){     
        
        if (i != 1){ //not needed at the first iteration, as everything is already initialized
            //update the initial position of the boat in the current segment
            boatState[0] = boatSpeed[i-2]*cos(boatHead)*T.ub() + boatState[0];
            boatState[1] = boatSpeed[i-2]*sin(boatHead)*T.ub() + boatState[1];
            
            //update the initial position of the obstacle in the current segment
            for (int j = 0; j < obstacles.size(); j++){
                obstacles[j][1] = obstacles[j][0]*cos(obstacles[j][3])*T.ub() + obstacles[j][1];
                obstacles[j][2] = obstacles[j][0]*sin(obstacles[j][3])*T.ub() + obstacles[j][2];
            }
        }        

        //update the duration of the current segment
        T = Interval(0, (sqrt(pow(waypoints[i][0] - waypoints[i-1][0],2)+pow(waypoints[i][1] - waypoints[i-1][1],2)))/boatSpeed[i-1].mid());

        //update the boat heading
        boatHead = acos((waypoints[i][0] - waypoints[i-1][0])/(sqrt(pow(waypoints[i][0] - waypoints[i-1][0],2)+pow(waypoints[i][1] - waypoints[i-1][1],2))));
        
        //check for each obstacles if there is a collision, if yes, call the pathReplanning method
        for (int j = 0; j < obstacles.size(); j++){
            drawx.resize(0);
            drawy.resize(0);
            drawx.push_back(obstacles[j][1].mid());
            drawy.push_back(obstacles[j][2].mid());
            drawx.push_back((obstacles[j][0]*cos(obstacles[j][3])*T.ub() + obstacles[j][1]).mid());
            drawy.push_back((obstacles[j][0]*sin(obstacles[j][3])*T.ub() + obstacles[j][2]).mid());
            vibes::selectFigure("path");
            vibes::drawLine(drawx, drawy, "black");
            if (collisionCondition(boatSpeed[i-1], boatState[0], boatState[1], boatHead, obstacles[j][0], obstacles[j][1], obstacles[j][2], obstacles[j][3], T*1.05)){
                cout << "collision detected in the " << i <<" segment, with obstacles " << j << endl;
                //in the computation, every obstacles are taken into account, so we won't enter here 2 times, even if there are 2 different obstacles in this segment
                
                pathReplanning(boatHead, boatSpeed[i-1], boatState, T*1.05, obstacles, borderList);


                if (i == waypoints.size() - 1){
                    boatSpeed.push_back(Interval(2,2.5));
                    waypoints.push_back(waypoints[i]);
                    waypoints[i][0] = boatSpeed[i-1].mid()*cos(boatHead)*T.ub() + boatState[0].mid();
                    waypoints[i][1] = boatSpeed[i-1].mid()*sin(boatHead)*T.ub() + boatState[1].mid();
                }
                else{
                    waypoints[i][0] = boatSpeed[i-1].mid()*cos(boatHead)*T.ub() + boatState[0].mid();
                    waypoints[i][1] = boatSpeed[i-1].mid()*sin(boatHead)*T.ub() + boatState[1].mid();
                }
                cout << "check collision risk after path replanning : " << collisionCondition(boatSpeed[i-1], boatState[0], boatState[1], boatHead, obstacles[j][0], obstacles[j][1], obstacles[j][2], obstacles[j][3], T) << endl;
            }
        }
    }
}

void pathReplanning(double& boatHead, Interval& speed, IntervalVector boatState, Interval T, vector<IntervalVector> obstacles, vector<vector<vector<double>>> borderList){
    IntervalVector speedComponents(2);
    speedComponents[0] = speed*cos(boatHead);
    speedComponents[1] = speed*sin(boatHead);

    string name = "path replanning from [" + to_string(boatState[0].lb()) + ", " + to_string(boatState[0].ub()) + "], [" + to_string(boatState[1].lb()) + ", " + to_string(boatState[1].ub()) + "]";
    
    vibes::newFigure(name);
    vibes::setFigureProperties(vibesParams("x", 100, "y", 100, "width", 800, "height", 800));

    Variable vx, vy;
    vector<SepInter*> listSep;
    Function* pf1;
    Function* pf2;
    Function* pf3;
    SepFwdBwd* pSep1;
    SepFwdBwd* pSep2;
    SepFwdBwd* pSep3;
    SepInter* pSep;    

    //compute separators for the borders (exterior borders and islands borders)
    for (int i = 0; i<borderList.size(); i++){
        createSepBorder(borderList[i], listSep, boatState, T);
    }

    //build one separator per obstacles, the union is made in the paving method
    for ( int i = 0; i < obstacles.size(); i++){
        pf1 = new Function(vx, vy, (vx - obstacles[i][0]*cos(obstacles[i][3]))*T  +boatState[0] - obstacles[i][1]);
        pf2 = new Function(vx, vy, (vy - obstacles[i][0]*sin(obstacles[i][3]))*T  + boatState[1] - obstacles[i][2]);
        pf3 = new Function(vx, vy, (vy - obstacles[i][0]*sin(obstacles[i][3]))*(boatState[0] - obstacles[i][1]) - (vx - obstacles[i][0]*cos(obstacles[i][3]))*(boatState[1] - obstacles[i][2]));

        pSep1 = new SepFwdBwd(*pf1, Interval(0,0));
        pSep2 = new SepFwdBwd(*pf2, Interval(0,0));
        pSep3 = new SepFwdBwd(*pf3, Interval(0,0));
        pSep = new SepInter(*pSep1, *pSep2, *pSep3);

        listSep.push_back(pSep);
    }

    double _speedInterval[2][2] = {{-10,10},{-10,10}};
    IntervalVector speedInterval(2, _speedInterval);

    vector<IntervalVector> listBoxes;

    paving(speedInterval, listSep, listBoxes);

    

    vibes::drawBoxes({{speedComponents[0].lb(), speedComponents[0].ub(), speedComponents[1].lb(), speedComponents[1].ub()}}, "[blue]");

    IntervalVector newSpeed = findClosest(listBoxes, speedComponents);
    vibes::drawBoxes({{newSpeed[0].lb(), newSpeed[0].ub(), newSpeed[1].lb(), newSpeed[1].ub()}}, "[green]");

    //compute the heading corresponding to the midpoint of the speed components box.
    if (newSpeed[0].mid()>= 0 && newSpeed[1].mid() >= 0){
        boatHead = atan(newSpeed[1].mid()/newSpeed[0].mid());
    }
    else if (newSpeed[0].mid() < 0 && newSpeed[1].mid() >= 0 ){
        boatHead = 3.14159 + atan(newSpeed[1].mid()/newSpeed[0].mid());
    }
    else if (newSpeed[0].mid() < 0 && newSpeed[1].mid() < 0){
        boatHead = -3.14159 + atan(newSpeed[1].mid()/newSpeed[0].mid());
    }
    else if (newSpeed[0].mid() >= 0 && newSpeed[1].mid() < 0){
        boatHead = atan(newSpeed[1].mid()/newSpeed[0].mid());
    }

    //compute the speed interval corresponding to this heading, and allowing the speed to stay inside the speed components box.
    double speedDiam = min(abs((newSpeed[0].ub() - newSpeed[0].lb())/cos(boatHead)), abs((newSpeed[1].ub() - newSpeed[1].lb())/sin(boatHead)));
    speed = Interval((newSpeed.mid()[0]/cos(boatHead)) - speedDiam/2, (newSpeed.mid()[0]/cos(boatHead)) + speedDiam/2);

    //for testing
    speedComponents[0] = speed*cos(boatHead);
    speedComponents[1] = speed*sin(boatHead);
    vibes::drawBoxes({{speedComponents[0].lb(), speedComponents[0].ub(), speedComponents[1].lb(), speedComponents[1].ub()}}, "[orange]");
    
    }
}