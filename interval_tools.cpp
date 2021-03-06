#include "ibex.h"
#include "vibes.h"
#include "interval_tools.h"

using namespace ibex;
using namespace std;

/*_____________________________________
useful functions for bisection of intervals and boxes*/
Interval left(const Interval& x){
    if (x.is_empty()){
        return Interval(Interval::EMPTY_SET);
    }
    return Interval(x.lb(), x.mid());
}

Interval right(const Interval& x){
    if (x.is_empty()){
        return Interval(Interval::EMPTY_SET);
    }
    return Interval(x.mid(), x.ub());
}

IntervalVector left(const IntervalVector& X){
    IntervalVector newX(2);
    if (X.is_empty()){
        return newX.empty(2);
    }
    if (X[0].diam() > X[1].diam()){
        newX[0] = left(X[0]);
        newX[1] = X[1];
        return newX;
    }
    newX[0] = X[0];
    newX[1] = left(X[1]);
    return newX;
}

IntervalVector right(const IntervalVector& X){
    IntervalVector newX(2);
    if (X.is_empty()){
        return newX.empty(2);
    }
    if (X[0].diam() > X[1].diam()){
        newX[0] = right(X[0]);
        newX[1] = X[1];
        return newX;
    }
    newX[0] = X[0];
    newX[1] = right(X[1]);       
    return newX;
}
//__________________________________________



void paving(IntervalVector initialBox, vector<shared_ptr<SepInter>> listSep, vector<IntervalVector>& listBoxes, double epsilon){
    if (initialBox.is_empty()){
        return;
    }
    else if (initialBox.max_diam() < epsilon){
        return;
    }
    IntervalVector XinEnd(2);
    IntervalVector XoutEnd(2);
    IntervalVector Xin = initialBox;
    IntervalVector Xout = initialBox;
    IntervalVector maybeBox(2);
    vector<IntervalVector> listXout;
    vector<IntervalVector> listXin;

    for (int i = 0; i < listSep.size(); i++){
        listSep[i]->SepInter::separate(Xin, Xout);
        listXout.push_back(Xout);
        listXin.push_back(Xin);
        Xin = initialBox;
        Xout = initialBox;
    }
    //compute the union of all the separators
    computeUnion(listXout, listXin, XoutEnd, XinEnd);
    //update the feasible speed set with the complementary of the result of the outer contraction
    buildFeasibleSpeedSet(listBoxes, initialBox, XoutEnd);

    vibes::drawBoxes({{XoutEnd[0].lb(), XoutEnd[0].ub(), XoutEnd[1].lb(), XoutEnd[1].ub()}}, "[red]");

    maybeBox = XinEnd & XoutEnd;

    vibes::drawBoxes({{maybeBox[0].lb(), maybeBox[0].ub(), maybeBox[1].lb(), maybeBox[1].ub()}}, "[yellow]");

    paving(left(maybeBox), listSep, listBoxes, epsilon);
    paving(right(maybeBox), listSep, listBoxes, epsilon);
}

void computeUnion(vector<IntervalVector> listXout, vector<IntervalVector> listXin, IntervalVector &XoutEnd, IntervalVector &XinEnd){
    XoutEnd = listXout[0];
    XinEnd = listXin[0];
    //compute the union of separators
    for (int i = 1; i < listXout.size(); i++ ){
        XoutEnd = XoutEnd | listXout[i];
        XinEnd = XinEnd & listXin[i];
    }
}

void buildFeasibleSpeedSet(vector<IntervalVector>& feasibleSpeedSet, IntervalVector X, IntervalVector Xout){
    IntervalVector newBox(2);

    IntervalVector* ListComplementary;

    int size = Xout.complementary(ListComplementary);

    for ( int i = 0; i < size; i++){
        newBox = ListComplementary[i]&X;
        if ( !newBox.is_empty() and newBox.volume() > 1e-15){
            feasibleSpeedSet.push_back(newBox);
            vibes::drawBoxes({{newBox[0].lb(), newBox[0].ub(), newBox[1].lb(), newBox[1].ub()}}, "[cyan]");
        }
    }
    delete[] ListComplementary;
}

void createSepBorder(vector<vector<double>> border, vector<shared_ptr<SepInter>> &listSep, IntervalVector boatInitPos, Interval timeInterval, 
    vector<shared_ptr<Function>> &deleteFunc, vector<shared_ptr<SepFwdBwd>> &deleteInitialSep)
{
    Variable vx, vy;

    for (int i = 0; i < border.size(); i++){
        shared_ptr<Function> pf1 = 
        shared_ptr<Function>(new Function(vx, vy, ((border[i][0] - (vx*timeInterval.ub() + boatInitPos[0]))*(border[i][1] - boatInitPos[1]) - 
                                                                        (border[i][1] - (vy*timeInterval.ub() + boatInitPos[1]))*(border[i][0] - boatInitPos[0]))*
                                                                        ((border[(i+1)%border.size()][0] - (vx*timeInterval.ub() + boatInitPos[0]))*(border[(i+1)%border.size()][1] - boatInitPos[1]) - 
                                                                        (border[(i+1)%border.size()][1] - (vy*timeInterval.ub() + boatInitPos[1]))*(border[(i+1)%border.size()][0] - boatInitPos[0]))
                                                                        ));
        deleteFunc.push_back(pf1);
        
        shared_ptr<Function> pf2 = 
        shared_ptr<Function>(new Function(vx, vy, ((border[(i+1)%border.size()][0] - border[i][0])*(border[i][1] - boatInitPos[1]) - 
                                                                        (border[(i+1)%border.size()][1] - border[i][1])*(border[i][0] - boatInitPos[0]))*
                                                                        ((border[(i+1)%border.size()][0] - border[i][0])*(border[i][1] - (vy*timeInterval.ub() + boatInitPos[1])) - 
                                                                        (border[(i+1)%border.size()][1] - border[i][1])*(border[i][0] - (vx*timeInterval.ub() + boatInitPos[0])))
                                                                        ));
        deleteFunc.push_back(pf2);
        // seems to work, but not sure about that
        shared_ptr<Function> pf3 = 
        shared_ptr<Function>(new Function(vx, vy, ibex::max(abs(ibex::max(border[i][0], border[(i+1)%border.size()][0]) - 
                                                                        ibex::max(boatInitPos[0].ub(), vx*timeInterval.ub() + boatInitPos[0].ub())), 
                                                                        sqrt(sqr(ibex::min(boatInitPos[0].lb(), vx*timeInterval.ub() + boatInitPos[0].lb()) - 
                                                                        ibex::min(border[i][0], border[(i+1)%border.size()][0])))) -
                                                                        ibex::max(ibex::max(border[i][0], border[(i+1)%border.size()][0]) - 
                                                                        ibex::min(border[i][0], border[(i+1)%border.size()][0]),
                                                                        ibex::max(boatInitPos[0].ub(), vx*timeInterval.ub() + boatInitPos[0].ub()) - 
                                                                        ibex::min(boatInitPos[0].lb(), vx*timeInterval.ub() + boatInitPos[0].lb()))
                                                                        ));
        deleteFunc.push_back(pf3);

        shared_ptr<Function> pf4 = 
        shared_ptr<Function>(new Function(vx, vy, ibex::max(abs(ibex::max(border[i][1], border[(i+1)%border.size()][1]) - 
                                                                        ibex::max(boatInitPos[1].ub(), vy*timeInterval.ub() + boatInitPos[1].ub())), 
                                                                        sqrt(sqr(ibex::min(boatInitPos[1].lb(), vy*timeInterval.ub() + boatInitPos[1].lb()) - 
                                                                        ibex::min(border[i][1], border[(i+1)%border.size()][1])))) -
                                                                        ibex::max(ibex::max(border[i][1], border[(i+1)%border.size()][1]) - 
                                                                        ibex::min(border[i][1], border[(i+1)%border.size()][1]), 
                                                                        ibex::max(boatInitPos[1].ub(), vy*timeInterval.ub() + boatInitPos[1].ub()) - 
                                                                        ibex::min(boatInitPos[1].lb(), vy*timeInterval.ub() + boatInitPos[1].lb()))
                                                                        ));
        deleteFunc.push_back(pf4);
        
        
        shared_ptr<SepFwdBwd> pSep1 = shared_ptr<SepFwdBwd>(new SepFwdBwd(*pf1, LEQ));
        deleteInitialSep.push_back(pSep1);
        shared_ptr<SepFwdBwd> pSep2 = shared_ptr<SepFwdBwd>(new SepFwdBwd(*pf2, LEQ));
        deleteInitialSep.push_back(pSep2);
        
        shared_ptr<SepFwdBwd> pSep3 = shared_ptr<SepFwdBwd>(new SepFwdBwd(*pf3, LEQ));
        deleteInitialSep.push_back(pSep3);
        shared_ptr<SepFwdBwd> pSep4 = shared_ptr<SepFwdBwd>(new SepFwdBwd(*pf4, LEQ));
        deleteInitialSep.push_back(pSep4);
        
        shared_ptr<SepInter> pSep = shared_ptr<SepInter>(new SepInter(*pSep1, *pSep2, *pSep3, *pSep4));

        listSep.push_back(pSep);
    }
}

void createSepObstacle(IntervalVector obstacles, vector<shared_ptr<SepInter>> &listSep, IntervalVector boatInitPos, 
    Interval timeInterval, vector<shared_ptr<Function>> &deleteFunc, vector<shared_ptr<SepFwdBwd>> &deleteInitialSep)
{
    Variable vx, vy;

    shared_ptr<Function> pf1 = 
    shared_ptr<Function>(new Function(vx, vy, (vx - obstacles[0]*cos(obstacles[3]))*timeInterval  +boatInitPos[0] - obstacles[1]));
    deleteFunc.push_back(pf1);
    shared_ptr<Function> pf2 = 
    shared_ptr<Function>(new Function(vx, vy, (vy - obstacles[0]*sin(obstacles[3]))*timeInterval  + boatInitPos[1] - obstacles[2]));
    deleteFunc.push_back(pf2);
    shared_ptr<Function> pf3 = 
    shared_ptr<Function>(new Function(vx, vy, (vy - obstacles[0]*sin(obstacles[3]))*(boatInitPos[0] - obstacles[1]) - 
                                                                    (vx - obstacles[0]*cos(obstacles[3]))*(boatInitPos[1] - obstacles[2])
                                                                    ));
    deleteFunc.push_back(pf3);

    shared_ptr<SepFwdBwd> pSep1 = shared_ptr<SepFwdBwd>(new SepFwdBwd(*pf1, Interval(0,0)));
    deleteInitialSep.push_back(pSep1);
    shared_ptr<SepFwdBwd> pSep2 = shared_ptr<SepFwdBwd>(new SepFwdBwd(*pf2, Interval(0,0)));
    deleteInitialSep.push_back(pSep2);
    shared_ptr<SepFwdBwd> pSep3 = shared_ptr<SepFwdBwd>(new SepFwdBwd(*pf3, Interval(0,0)));
    deleteInitialSep.push_back(pSep3);

    shared_ptr<SepInter> pSep = shared_ptr<SepInter>(new SepInter(*pSep1, *pSep2, *pSep3));

    listSep.push_back(pSep);
}


bool collisionCondition(Interval v, Interval x0, Interval y0, double th, Interval vi, Interval xi, Interval yi, Interval thi, Interval t){
    Interval C1, C2, C3;
    C1 = (v*cos(th)-vi*cos(thi))*t+x0-xi;
    C2 = (v*sin(th)-vi*sin(thi))*t+y0-yi;
    C3 = (v*sin(th)-vi*sin(thi))*(x0-xi)-(v*cos(th)-vi*cos(thi))*(y0-yi);
    if (C1.contains(0) and C2.contains(0) and C3.contains(0)){
        return 1;
    }
    else{
        return 0;
    }
}

bool crossBorder(Interval v, Interval x0, Interval y0, double th, double t, vector<double> border1, vector<double> border2){
    Interval C1, C2;
    IntervalVector C3(2);
    /*
    double C3, C4;
    */

    C1 = ((border1[0] - (v*cos(th)*t + x0))*(border1[1] - y0) - 
            (border1[1] - (v*sin(th)*t + y0))*(border1[0] - x0))*
            ((border2[0] - (v*cos(th)*t + x0))*(border2[1] - y0) - 
            (border2[1] - (v*sin(th)*t + y0))*(border2[0] - x0));

    C2 = ((border2[0] - border1[0])*(border1[1] - y0) - 
            (border2[1] - border1[1])*(border1[0] - x0))*
            ((border2[0] - border1[0])*(border1[1] - (v*sin(th)*t + y0)) - 
            (border2[1] - border1[1])*(border1[0] - (v*cos(th)*t + x0)));
    /*
    C3 = max(abs(max(border1[0], border2[0]) - max(x0.ub(), (v*cos(th)*t + x0).ub())), abs(min(x0.lb(), (v*cos(th)*t + x0).lb()) - min(border1[0], border2[0]))) -
                                                    max(max(border1[0], border2[0]) - min(border1[0], border2[0]), max(x0.ub(), (v*cos(th)*t + x0).ub()) - min(x0.lb(), (v*cos(th)*t + x0).lb()));
    
    C4 = max(abs(max(border1[1], border2[1]) - max(y0.ub(), (v*sin(th)*t + y0).ub())), abs(min(y0.lb(), (v*sin(th)*t + y0).lb()) - min(border1[1], border2[1]))) -
                                                    max(max(border1[1], border2[1]) - min(border1[1], border2[1]), max(y0.ub(), (v*sin(th)*t + y0).ub()) - min(y0.lb(), (v*sin(th)*t + y0).lb()));
      */
    IntervalVector boatInitPos(2);
    IntervalVector finalPos(2);
    IntervalVector borderBox1(2);
    IntervalVector borderBox2(2);
    borderBox1[0] = Interval(border1[0]);
    borderBox1[1] = Interval(border1[1]);
    borderBox2[0] = Interval(border2[0]);
    borderBox2[1] = Interval(border2[1]);
    boatInitPos[0] = x0;
    boatInitPos[1] = y0;
    finalPos[0] = v*cos(th)*t + x0;
    finalPos[1] = v*sin(th)*t + y0;
    C3 = (borderBox1 | borderBox2) & (boatInitPos | finalPos);

    if (C1.overlaps(Interval::NEG_REALS) and C2.overlaps(Interval::NEG_REALS) and !C3.is_empty()/*and C3 <= 0 and C4 <= 0*/){
        return 1;
    }
    else{
        return 0;
    }
}

double midPointDistance(IntervalVector X, IntervalVector Y){
    return sqrt(pow(X.mid()[0] - Y.mid()[0], 2) + pow(X.mid()[1] - Y.mid()[1], 2));
}

IntervalVector findClosest(vector<IntervalVector> listBoxes, IntervalVector boatSpeed){
    double dist = numeric_limits<double>::max();
    IntervalVector outputBox(2); 
    for ( int i = 0; i < listBoxes.size(); i++){
        if (midPointDistance(boatSpeed, listBoxes[i]) < dist){
            dist = midPointDistance(boatSpeed, listBoxes[i]);
            outputBox = listBoxes[i];
        }
    }
    return outputBox;
}