#include "ibex.h"
#include "vibes.h"
#include "interval_tools.h"

using namespace ibex;
using namespace std;

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


void paving(IntervalVector X, vector<SepInter*> listSep, vector<IntervalVector>& listBoxes){
    if (X.max_diam() < 0.5){
        return;
    }
    IntervalVector XinEnd(2);
    IntervalVector XoutEnd(2);
    IntervalVector Xin = X;
    IntervalVector Xout = X;
    vector<IntervalVector> listXout;
    vector<IntervalVector> listXin;

    for (int i = 0; i < listSep.size(); i++){
        listSep[i]->SepInter::separate(Xin, Xout);
        listXout.push_back(Xout);
        Xin = X;
        Xout = X;
    }

    XoutEnd = listXout[0];
    
    for (int i = 1; i < listXout.size(); i++ ){
        XoutEnd = XoutEnd | listXout[i];
    }
    
    IntervalVector newBox(2);

    IntervalVector* ListComplementary;

    int size = XoutEnd.complementary(ListComplementary);

    for ( int i = 0; i < size; i++){
        newBox = ListComplementary[i]&X;
        if (!newBox.is_flat()){
            listBoxes.push_back(newBox);
            vibes::drawBoxes({{newBox[0].lb(), newBox[0].ub(), newBox[1].lb(), newBox[1].ub()}}, "[cyan]");
        }
    }

    vibes::drawBoxes({{XoutEnd[0].lb(), XoutEnd[0].ub(), XoutEnd[1].lb(), XoutEnd[1].ub()}}, "[red]");

    Xin = XoutEnd;
    Xout = XoutEnd;

    for (int i = 0; i < listSep.size(); i++){
        listSep[i]->SepInter::separate(Xin, Xout);
        listXin.push_back(Xin);
        Xin = XoutEnd;
        Xout = XoutEnd;
    }   

    XinEnd = listXin[0];
    
    for (int i = 0; i < listXin.size(); i++){
        XinEnd = XinEnd & listXin[i];
    }

    vibes::drawBoxes({{XinEnd[0].lb(), XinEnd[0].ub(), XinEnd[1].lb(), XinEnd[1].ub()}}, "[yellow]");

    paving(left(XinEnd), listSep, listBoxes);
    paving(right(XinEnd), listSep, listBoxes);
}

bool collisionCondition(Interval v, Interval x0, Interval y0, Interval th, Interval vi, Interval xi, Interval yi, Interval thi, Interval t){
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

IntervalVector findClosest(vector<IntervalVector> listBoxes, IntervalVector boatSpeed){
    double dist = 1000000000;
    IntervalVector outputBox(2); 
    for ( int i = 0; i < listBoxes.size(); i++){
        if (distance(boatSpeed, listBoxes[i]) < dist){
            dist = distance(boatSpeed, listBoxes[i]);
            outputBox = listBoxes[i];
        }
    }
    return outputBox;
}