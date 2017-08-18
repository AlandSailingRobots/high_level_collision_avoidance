#include <thread>
#include <chrono>
#include <math.h>
#include "ibex.h"
#include "vibes.h"
#include "interval_tools.h"
#include "functions.h"

using namespace ibex;
using namespace std;

namespace functions{
void manageCollision(vector<vector<double>>& waypoints, IntervalVector boatState, vector<Interval>& boatSpeed, vector<IntervalVector> obstacles, vector<vector<vector<double>>> borderList){
    const double timeSafetyMargin = 1.05;
    double boatHead;  

    Interval timeInterval;

    //for each segment of the trajectory 
    for ( int i = 1; i< waypoints.size(); i++){     
        if (i != 1){ //not needed at the first iteration, as everything is already initialized
            //update the initial position of the boat in the current segment
            
            boatState[0] = boatSpeed[i-2]*cos(boatHead)*timeInterval.ub() + boatState[0];
            boatState[1] = boatSpeed[i-2]*sin(boatHead)*timeInterval.ub() + boatState[1];
            
            //update the initial position of the obstacle in the current segment
            for (int j = 0; j < obstacles.size(); j++){
                obstacles[j][1] = obstacles[j][0]*cos(obstacles[j][3])*timeInterval.ub() + obstacles[j][1];
                obstacles[j][2] = obstacles[j][0]*sin(obstacles[j][3])*timeInterval.ub() + obstacles[j][2];
            }
        }          

        //update the duration of the current segment
        timeInterval = Interval(0, (sqrt(pow(waypoints[i][0] - waypoints[i-1][0],2)+pow(waypoints[i][1] - waypoints[i-1][1],2)))/boatSpeed[i-1].mid());

        //update the boat heading
        //assume that the boat heading will be aligned with the 2 waypoints
        boatHead = atan2(waypoints[i][1] - waypoints[i-1][1], waypoints[i][0] - waypoints[i-1][0]);
        
        bool collisionDetected = 0;
        int j = 0;

        //check for each obstacles if there is a collision, if yes, call the pathReplanning method
        while (j < obstacles.size() and collisionDetected == 0){
            // timeInterval*timeSafetyMargin avoid to choose a route and a speed wich will lead to a waypoint very close to a collision
            if (collisionCondition(boatSpeed[i-1], boatState[0], boatState[1], boatHead, obstacles[j][0], obstacles[j][1], obstacles[j][2], obstacles[j][3], timeInterval*timeSafetyMargin)){
                cout << "collision detected in the " << i <<" segment, with obstacles " << j << endl;
                pathReplanning(boatHead, boatSpeed[i-1], boatState, timeInterval*timeSafetyMargin, obstacles, borderList);
                waypointManagement(boatHead, boatSpeed, boatState, timeInterval.ub(), waypoints, i);
                collisionDetected = 1;

            }
            j++;
        }

        j = 0;
        //check for each border if there is a collision
        while (j < borderList.size() and collisionDetected == 0){
            int k = 0;
            while (k < (borderList[j]).size() and collisionDetected == 0){
                if (crossBorder(boatSpeed[i-1], boatState[0], boatState[1], boatHead, timeInterval.ub()*timeSafetyMargin, borderList[j][k], borderList[j][(k+1)%borderList[j].size()])){
                    cout << "collision detected in the " << i << " segment, with the " << k << " segment of the " << j << " border" << endl;
                    pathReplanning(boatHead, boatSpeed[i-1], boatState, timeInterval*timeSafetyMargin, obstacles, borderList);
                    waypointManagement(boatHead, boatSpeed, boatState, timeInterval.ub(), waypoints, i);
                    collisionDetected = 1;
                }
                k++;
            }
            j++;
        }      
    }
}

void waypointManagement(double boatHead, vector<Interval>& boatSpeed, IntervalVector boatState, double endTime, vector<vector<double>>& waypoints, int currentSegmentIndice){
    vector<double> formerWaypts = waypoints[currentSegmentIndice];
    IntervalVector finalState(2);
    waypoints[currentSegmentIndice][0] = boatSpeed[currentSegmentIndice-1].mid()*cos(boatHead)*endTime + boatState[0].mid();
    waypoints[currentSegmentIndice][1] = boatSpeed[currentSegmentIndice-1].mid()*sin(boatHead)*endTime + boatState[1].mid();
    finalState[0] = boatSpeed[currentSegmentIndice-1]*cos(boatHead)*endTime + boatState[0];
    finalState[1] = boatSpeed[currentSegmentIndice-1]*sin(boatHead)*endTime + boatState[1];
    if (currentSegmentIndice == waypoints.size() - 1 and !(finalState[0].contains(formerWaypts[0]) and finalState[1].contains(formerWaypts[1]))){
        boatSpeed.push_back(Interval(2,2.5));
        waypoints.push_back(formerWaypts);
    }
}

void pathReplanning(double& boatHead, Interval& speed, IntervalVector boatState, Interval timeInterval, vector<IntervalVector> obstacles, vector<vector<vector<double>>> borderList){
    IntervalVector speedComponents(2);
    speedComponents[0] = speed*cos(boatHead);
    speedComponents[1] = speed*sin(boatHead);
    cout << "speed component x before " << speedComponents[0] << endl;
    cout << "speed component y before " << speedComponents[1] << endl;

    string name = "path replanning from [" + to_string(boatState[0].lb()) + ", " + to_string(boatState[0].ub()) + "], [" + to_string(boatState[1].lb()) + ", " + to_string(boatState[1].ub()) + "]";
    
    vibes::newFigure(name);
    vibes::setFigureProperties(vibesParams("x", 800, "y", 0, "width", 800, "height", 800));

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
        createSepBorder(borderList[i], listSep, boatState, timeInterval);
    }

    //build one separator per obstacles, the union is made in the paving method
    for ( int i = 0; i < obstacles.size(); i++){
        pf1 = new Function(vx, vy, (vx - obstacles[i][0]*cos(obstacles[i][3]))*timeInterval  +boatState[0] - obstacles[i][1]);
        pf2 = new Function(vx, vy, (vy - obstacles[i][0]*sin(obstacles[i][3]))*timeInterval  + boatState[1] - obstacles[i][2]);
        pf3 = new Function(vx, vy, (vy - obstacles[i][0]*sin(obstacles[i][3]))*(boatState[0] - obstacles[i][1]) - (vx - obstacles[i][0]*cos(obstacles[i][3]))*(boatState[1] - obstacles[i][2]));

        pSep1 = new SepFwdBwd(*pf1, Interval(0,0));
        pSep2 = new SepFwdBwd(*pf2, Interval(0,0));
        pSep3 = new SepFwdBwd(*pf3, Interval(0,0));
        pSep = new SepInter(*pSep1, *pSep2, *pSep3);

        listSep.push_back(pSep);
    }

    double speedIntervalBounds[2][2] = {{-10,10},{-10,10}};
    IntervalVector speedInterval(2, speedIntervalBounds);

    vector<IntervalVector> listFeasibleSpeedBoxes;

    paving(speedInterval, listSep, listFeasibleSpeedBoxes, 0.5);


    vibes::drawBoxes({{speedComponents[0].lb(), speedComponents[0].ub(), speedComponents[1].lb(), speedComponents[1].ub()}}, "[blue]");

    IntervalVector newSpeed = findClosest(listFeasibleSpeedBoxes, speedComponents);
    vibes::drawBoxes({{newSpeed[0].lb(), newSpeed[0].ub(), newSpeed[1].lb(), newSpeed[1].ub()}}, "[green]");
    cout << "speed component x after " << newSpeed[0] << endl;
    cout << "speed component y after " << newSpeed[1] << endl;

    //compute the heading corresponding to the midpoint of the speed components box.
    boatHead = computeHeading(newSpeed[0], newSpeed[1]);

    //compute the speed interval corresponding to this heading, and allow the speed to stay inside the speed components box.
    double speedDiam = min(abs((newSpeed[0].ub() - newSpeed[0].lb())/cos(boatHead)), abs((newSpeed[1].ub() - newSpeed[1].lb())/sin(boatHead)));
    speed = Interval((newSpeed.mid()[0]/cos(boatHead)) - speedDiam/2, (newSpeed.mid()[0]/cos(boatHead)) + speedDiam/2);

    //display the speed box generated by this new heading and speed, in order to check if it is inside the speed components box as intended
    speedComponents[0] = speed*cos(boatHead);
    speedComponents[1] = speed*sin(boatHead);
    vibes::drawBoxes({{speedComponents[0].lb(), speedComponents[0].ub(), speedComponents[1].lb(), speedComponents[1].ub()}}, "[]");
    }

    double computeHeading(Interval XspeedComponent, Interval YspeedComponent){
        if (XspeedComponent.mid()>= 0 && YspeedComponent.mid() >= 0){
            return atan(YspeedComponent.mid()/XspeedComponent.mid());
        }
        else if (XspeedComponent.mid() < 0 && YspeedComponent.mid() >= 0 ){
            return M_PI + atan(YspeedComponent.mid()/XspeedComponent.mid());
        }
        else if (XspeedComponent.mid() < 0 && YspeedComponent.mid() < 0){
            return -M_PI + atan(YspeedComponent.mid()/XspeedComponent.mid());
        }
        else if (XspeedComponent.mid() >= 0 && YspeedComponent.mid() < 0){
            return atan(YspeedComponent.mid()/XspeedComponent.mid());
        }
    }


    void drawTrajectory(vector<vector<double>> waypoints, vector<Interval> boatSpeed, IntervalVector boatState, vector<IntervalVector> obstacles){
        vector<double> drawx, drawy;
        vibes::selectFigure("path");
        vibes::newGroup("trajectories");
        double boatHead;

        Interval timeInterval;

        //for each segment of the trajectory 
        for ( int i = 1; i < waypoints.size(); i++){     
            
            if (i != 1){ //not needed at the first iteration, as everything is already initialized
                //update the initial position of the boat in the current segment
                boatState[0] = boatSpeed[i-2]*cos(boatHead)*timeInterval.ub() + boatState[0];
                boatState[1] = boatSpeed[i-2]*sin(boatHead)*timeInterval.ub() + boatState[1];
                
                //update the initial position of the obstacle in the current segment
                for (int j = 0; j < obstacles.size(); j++){
                    obstacles[j][1] = obstacles[j][0]*cos(obstacles[j][3])*timeInterval.ub() + obstacles[j][1];
                    obstacles[j][2] = obstacles[j][0]*sin(obstacles[j][3])*timeInterval.ub() + obstacles[j][2];
                }
            }        

            //update the duration of the current segment
            timeInterval = Interval(0, (sqrt(pow(waypoints[i][0] - waypoints[i-1][0],2)+pow(waypoints[i][1] - waypoints[i-1][1],2)))/boatSpeed[i-1].mid());

            for (int j = 0; j < obstacles.size(); j++){
                //for drawing the trajectory of the obstacles :
                drawx.resize(0);
                drawy.resize(0);
                drawx.push_back(obstacles[j][1].mid());
                drawy.push_back(obstacles[j][2].mid());
                drawx.push_back((obstacles[j][0]*cos(obstacles[j][3])*timeInterval.ub() + obstacles[j][1]).mid());
                drawy.push_back((obstacles[j][0]*sin(obstacles[j][3])*timeInterval.ub() + obstacles[j][2]).mid());
                vibes::drawLine(drawx, drawy, "black");
            }

            //update the boat heading
            boatHead = atan2(waypoints[i][1] - waypoints[i-1][1], waypoints[i][0] - waypoints[i-1][0]);
            double t = 0;
            double dt = 0.5;
            Interval x, y;
            while ( t<=timeInterval.ub()){
                x = boatSpeed[i-1]*cos(boatHead)*t + boatState[0];
                y = boatSpeed[i-1]*sin(boatHead)*t + boatState[1];
                vibes::drawBox(x.lb(), x.ub(), y.lb(), y.ub(), "[blue]", vibesParams("group", "trajectories"));
                for (int k = 0; k<obstacles.size(); k++){
                    x = obstacles[k][0]*cos(obstacles[k][3])*t + obstacles[k][1];
                    y = obstacles[k][0]*sin(obstacles[k][3])*t + obstacles[k][2];
                    vibes::drawBox(x.lb(), x.ub(), y.lb(), y.ub(), "[black]", vibesParams("group", "trajectories"));
                }

                this_thread::sleep_for(chrono::milliseconds(50));
                vibes::clearGroup("trajectories");
                t+= dt;
            }
        }
    }
}