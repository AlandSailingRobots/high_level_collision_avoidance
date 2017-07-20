#include "ibex.h"
#include "vibes.h"
#include "interval_tools.h"
#include "functions.h"

using namespace ibex;
using namespace std;

namespace functions{
void manageCollision(vector<vector<double>>& waypoints, IntervalVector boatState, vector<Interval>& boatSpeed, vector<IntervalVector> obstacles){
    double boatHead;
    boatHead = acos(waypoints[1][0]/(sqrt(pow(waypoints[1][0],2)+pow(waypoints[1][1],2))));

    Interval T;

    for ( int i = 1; i< waypoints.size(); i++){
        //cout << "heading " << boatHead << endl; 
        
        if (i != 1){
            boatState[0] = boatSpeed[i-2]*cos(boatHead)*T.ub() + boatState[0];
            boatState[1] = boatSpeed[i-2]*sin(boatHead)*T.ub() + boatState[1];
            //cout << "speed " << i-2 << " " << boatSpeed[i-2] << endl;

            for (int j = 0; j < obstacles.size(); j++){
                obstacles[j][1] = obstacles[j][0]*cos(obstacles[j][3])*T.ub() + obstacles[j][1];
                obstacles[j][2] = obstacles[j][0]*sin(obstacles[j][3])*T.ub() + obstacles[j][2];
            }
        }        

        T = Interval(0, (sqrt(pow(waypoints[i][0] - waypoints[i-1][0],2)+pow(waypoints[i][1] - waypoints[i-1][1],2)))/boatSpeed[i-1].mid());

        boatHead = acos((waypoints[i][0] - waypoints[i-1][0])/(sqrt(pow(waypoints[i][0] - waypoints[i-1][0],2)+pow(waypoints[i][1] - waypoints[i-1][1],2))));
        //cout << "heading " << boatHead << endl; 
        for (int j = 0; j < obstacles.size(); j++){
            if (collisionCondition(boatSpeed[i-1], boatState[0], boatState[1], boatHead, obstacles[j][0], obstacles[j][1], obstacles[j][2], obstacles[j][3], T)){
                pathReplanning(boatHead, boatSpeed[i-1], boatState, T, obstacles);
                if (i == waypoints.size() - 1){
                    //cout << "new speed for new waypoint :" << boatSpeed[i-1] << endl;
                    boatSpeed.push_back(Interval(2,2.5));
                    waypoints.push_back(waypoints[i]);
                    waypoints[i][0] = boatSpeed[i-1].mid()*cos(boatHead)*T.ub() + boatState[0].mid();
                    waypoints[i][1] = boatSpeed[i-1].mid()*sin(boatHead)*T.ub() + boatState[1].mid();
                }
                else{
                    waypoints[i][0] = boatSpeed[i-1].mid()*cos(boatHead)*T.ub() + boatState[0].mid();
                    waypoints[i][1] = boatSpeed[i-1].mid()*sin(boatHead)*T.ub() + boatState[1].mid();
                }
            }
        }
    }
}

void pathReplanning(double& boatHead, Interval& speed, IntervalVector boatState, Interval T, vector<IntervalVector> obstacles){
    //cout << "initial heading " << boatHead << endl;
    //cout << "initial speed " << speed << endl;
    //cout << "initial boat position " << boatState << endl;

    for ( int i = 0; i < obstacles.size(); i++){
        //cout << "obstacles state " << obstacles[i] << endl;
    }

    IntervalVector speedComponents(2);
    speedComponents[0] = speed*cos(boatHead);
    speedComponents[1] = speed*sin(boatHead);

    vibes::beginDrawing();
    vibes::newFigure("feasible speed");
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

    delete pf1, pf2, pf3, pSep1, pSep2, pSep3, pSep, listSep;

    vibes::drawBoxes({{speedComponents[0].lb(), speedComponents[0].ub(), speedComponents[1].lb(), speedComponents[1].ub()}}, "[red]");

    IntervalVector newSpeed = findClosest(listBoxes, speedComponents);
    vibes::drawBoxes({{newSpeed[0].lb(), newSpeed[0].ub(), newSpeed[1].lb(), newSpeed[1].ub()}}, "[green]");
    //cout << "new speed compoments " << newSpeed << endl;

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
    if (newSpeed[0].diam() < newSpeed[1].diam()){
        speed = newSpeed[0]/cos(boatHead);
    }
    else{
        speed = newSpeed[1]/sin(boatHead);
    }
    //speed = (newSpeed[0].mid()/cos(boatHead))*Interval(0.8,1.2);
    //cout << "modified heading "<< boatHead << endl;
    //cout << "modified speed " << speed << endl;
}
}