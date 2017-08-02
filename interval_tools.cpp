#include "ibex.h"
#include "vibes.h"
#include "interval_tools.h"

using namespace ibex;
using namespace std;

/*_____________________________________
useful functions for bisection of interval and boxes*/
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



void paving(IntervalVector X, vector<SepInter*> listSep, vector<IntervalVector>& listBoxes){
    if (X.is_empty()){
        return;
    }
    else if (X.max_diam() < 0.5){
        return;
    }
    IntervalVector XinEnd(2);
    IntervalVector XoutEnd(2);
    IntervalVector Xin = X;
    IntervalVector Xout = X;
    IntervalVector maybeBox(2);
    vector<IntervalVector> listXout;
    vector<IntervalVector> listXin;

    for (int i = 0; i < listSep.size(); i++){
        listSep[i]->SepInter::separate(Xin, Xout);
        listXout.push_back(Xout);
        listXin.push_back(Xin);
        Xin = X;
        Xout = X;
    }

    XoutEnd = listXout[0];
    XinEnd = listXin[0];
    
    for (int i = 1; i < listXout.size(); i++ ){
        XoutEnd = XoutEnd | listXout[i];
        XinEnd = XinEnd & listXin[i];
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

    maybeBox = XinEnd & XoutEnd;

    vibes::drawBoxes({{maybeBox[0].lb(), maybeBox[0].ub(), maybeBox[1].lb(), maybeBox[1].ub()}}, "[yellow]");

    paving(left(maybeBox), listSep, listBoxes);
    paving(right(maybeBox), listSep, listBoxes);
}

void createSepBorder(vector<vector<double>> border, vector <SepInter*> &listSep, IntervalVector boatInitPos, Interval T){
    Variable vx, vy;
    Function* pf1;
    Function* pf2;
    Function* pf3;
    Function* pf4;
    SepFwdBwd* pSep1;
    SepFwdBwd* pSep2;
    SepFwdBwd* pSep3;
    SepFwdBwd* pSep4;
    SepInter* pSep;    

    for (int i = 0; i < border.size(); i++){
        pf1 = new Function(vx, vy, ((border[i][0] - (vx*T.ub() + boatInitPos[0]))*(border[i][1] - boatInitPos[1]) - 
                                                    (border[i][1] - (vy*T.ub() + boatInitPos[1]))*(border[i][0] - boatInitPos[0]))*
                                                    ((border[(i+1)%border.size()][0] - (vx*T.ub() + boatInitPos[0]))*(border[(i+1)%border.size()][1] - boatInitPos[1]) - 
                                                    (border[(i+1)%border.size()][1] - (vy*T.ub() + boatInitPos[1]))*(border[(i+1)%border.size()][0] - boatInitPos[0]))
                                                    );
        
        pf2 = new Function(vx, vy, ((border[(i+1)%border.size()][0] - border[i][0])*(border[i][1] - boatInitPos[1]) - 
                                                    (border[(i+1)%border.size()][1] - border[i][1])*(border[i][0] - boatInitPos[0]))*
                                                    ((border[(i+1)%border.size()][0] - border[i][0])*(border[i][1] - (vy*T.ub() + boatInitPos[1])) - 
                                                    (border[(i+1)%border.size()][1] - border[i][1])*(border[i][0] - (vx*T.ub() + boatInitPos[0])))
                                                    );
        
        pf3 = new Function(vx, vy, max(abs(max(border[i][0], border[(i+1)%border.size()][0]) - max(boatInitPos[0].ub(), vx*T.ub() + boatInitPos[0].ub())), sqrt(sqr(min(boatInitPos[0].lb(), vx*T.ub() + boatInitPos[0].lb()) - min(border[i][0], border[(i+1)%border.size()][0])))) -
                                                    max(max(border[i][0], border[(i+1)%border.size()][0]) - min(border[i][0], border[(i+1)%border.size()][0]), max(boatInitPos[0].ub(), vx*T.ub() + boatInitPos[0].ub()) - min(boatInitPos[0].lb(), vx*T.ub() + boatInitPos[0].lb()))
                                                    );

        pf4 = new Function(vx, vy, max(abs(max(border[i][1], border[(i+1)%border.size()][1]) - max(boatInitPos[1].ub(), vy*T.ub() + boatInitPos[1].ub())), sqrt(sqr(min(boatInitPos[1].lb(), vy*T.ub() + boatInitPos[1].lb()) - min(border[i][1], border[(i+1)%border.size()][1])))) -
                                                    max(max(border[i][1], border[(i+1)%border.size()][1]) - min(border[i][1], border[(i+1)%border.size()][1]), max(boatInitPos[1].ub(), vy*T.ub() + boatInitPos[1].ub()) - min(boatInitPos[1].lb(), vy*T.ub() + boatInitPos[1].lb()))
                                                    );
        

        pSep1 = new SepFwdBwd(*pf1, LEQ);
        pSep2 = new SepFwdBwd(*pf2, LEQ);
        
        pSep3 = new SepFwdBwd(*pf3, LEQ);
        pSep4 = new SepFwdBwd(*pf4, LEQ);
        
        pSep = new SepInter(*pSep1, *pSep2, *pSep3, *pSep4);

        listSep.push_back(pSep);
    }
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

double midPointDistance(IntervalVector X, IntervalVector Y){
    return sqrt(pow(X.mid()[0] - Y.mid()[0], 2) + pow(X.mid()[1] - Y.mid()[1], 2));
}

IntervalVector findClosest(vector<IntervalVector> listBoxes, IntervalVector boatSpeed){
    double dist = 1000000000;
    IntervalVector outputBox(2); 
    for ( int i = 0; i < listBoxes.size(); i++){
        if (midPointDistance(boatSpeed, listBoxes[i]) < dist){
            dist = midPointDistance(boatSpeed, listBoxes[i]);
            outputBox = listBoxes[i];
        }
    }
    return outputBox;
}