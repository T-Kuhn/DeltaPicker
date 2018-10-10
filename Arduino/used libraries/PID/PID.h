/*
  PID_T.h - a simple PID_T Controller library for the Arduino platform 
  Created by Tobias Kuhn. Sapporo, December 30, 2015.
  Released into the public domain.
*/

#ifndef PID_h
#define PID_h
#include "Arduino.h"

// - - - - - - - - - - - - - - - - - - -
// - - - - - - - PID_T CLASS - - - - - - -
// - - - - - - - - - - - - - - - - - - -
class PID
{
    public:
        void begin(double kp, double ki, double kd, double tol, double holdPosVal, int mode, bool debug);
        int update(double pv, bool following);
        void setSetPoint(double sp);
    private:
        int _mode;
        // mode = 0 : normal res PID
        double _kp;
        double _kpQuant;
        double _ki;
        double _kiQuant;
        double _kd;
        double _kdQuant;
        double _tol;
        double _holdPosVal;
        double _errorVal;
        double _setPoint;
        double _speed;
        double _controlVar;
        bool _debug;
};

#endif














