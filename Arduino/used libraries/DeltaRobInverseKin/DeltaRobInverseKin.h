/*
  DeltaRobInverseKin.h -Å@This library provides a way to evaluate, given
  some x, y, z coordinates, the needed motor positions of a Delta Robot.
  Created by Tobias Kuhn, Sapporo, February 13, 2016. Released into the public domain.
  */

#ifndef DeltaRobInverseKin_h
#define DeltaRobInverseKin_h
#include "Arduino.h"

// - - - - - - - - - - - - - - - - - - -
// - - - - - Encoder CLASS - - - - - - -
// - - - - - - - - - - - - - - - - - - -
class DeltaRobInverseKin
{
    public:
        DeltaRobInverseKin(double L, double l, double wb, double wp, double up, double sp);
        void setGoalCoordinates(double x, double y, double z, int state);
        void resetArr();
        bool debugFlag;
        int goalPos[3];
        int posArr[500][4];
        int maxArrIndex;
    private:
        void _computePara_abc();
        void _computePara_EFG();
        void _computePara_t();
        void _computeAngles();
        void _computeGoalPos();
        int _state;
        double _rat;
        double _L, _l, _wb, _wp, _up, _sp;
        double _a, _b, _c;
        double _x, _y, _z;
        double _E[3], _F[3], _G[3];
        double _t1Num[3], _t1Den[3], _t2Num[3], _t2Den[3];
        double _theta1[3], _theta2[3];
};

#endif














