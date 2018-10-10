/*
  PID_T.cpp - a simple PID_T Controller library for the Arduino platform 
  Created by Tobias Kuhn. Sapporo, December 30, 2015.
  Released into the public domain.
*/

#include "Arduino.h"
#include "PID.h"
#include <math.h>

// - - - - - - - - - - - - - - - - - - -
// - - - - - - PID_T BEGIN - - - - - - -
// - - - - - - - - - - - - - - - - - - -
void PID::begin(double kp, double ki, double kd, double tol, double holdPosVal, int mode, bool debug)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _tol = tol;

    _debug = debug;

    _holdPosVal = holdPosVal;    
    _mode = mode;

}

// - - - - - - - - - - - - - - - - - - -
// - - - - - - PID_T UPDATE  - - - - - -
// - - - - - - - - - - - - - - - - - - -
int PID::update(double pv, bool following)
// @return: Value from -1023~1023 for PWM 
{
    int res; 
    if(_debug){
        Serial.print(pv);
        Serial.print(";");
        Serial.println(_setPoint);
        }
    
    _errorVal = _setPoint - pv;

    if(!following){
        _kpQuant =  _holdPosVal * _kp * _errorVal;
        _kiQuant += _ki * _errorVal;   
        _kdQuant = _kd * 0.0;
    }else{ 
        _kpQuant =  _kp * _errorVal;
        _kiQuant += _ki * _errorVal;
        _kdQuant = _kd * 0.0;
    }
    
    if(_kiQuant > 50.0f){
       _kiQuant = 50.0f;
    }else if(_kiQuant < -50.0f){
        _kiQuant= -50.0f;
    }
    
    res = (int)round(_kpQuant + _kiQuant + _kdQuant);

    if(res > 1023){
        res =  1023;
    }else if(res < -1023){
        res = -1023;
    }
    
    if(!following){
        _kiQuant = 0;
    }

    if(following && res == 0){
        if(_errorVal > 0){
            res = 1;
        }else{
            res = -1;
        }
    }
    
    if(abs(_errorVal) <= _tol && !following){
        res = 0;
    }

    return res;
}

// - - - - - - - - - - - - - - - - - - -
// - - - - - PID_T SETSETPOINT - - - - -
// - - - - - - - - - - - - - - - - - - -
void PID::setSetPoint(double sp)
{
    _setPoint = sp;
}


