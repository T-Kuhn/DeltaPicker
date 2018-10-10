/*
  EncMotControl.h - A DC motor (geared) position controller
  Created by Tobias Kuhn. Sapporo, February 29, 2016.
  Released into the public domain.
*/

#ifndef EndMotControl_h
#define EndMotControl_h
#include "Arduino.h"
#include "PID.h"
#include "Encoder.h"
#include "MPU6050.h"

// - - - - - - - - - - - - - - - - - - -
// - - - - EncMotControl CLASS - - - - -
// - - - - - - - - - - - - - - - - - - -
class EncMotControl
{
    public:
        EncMotControl(int in1Pin, int in2Pin, int pwmPin, int mpuAddPin, double calib, bool debug);
        PID pid; 
        MPU6050 mpu;
        void begin();
        void initStep1();
        void initStep1_5();
        void initStep2();
        void update(Encoder enc);
        void setMode(int nmbr);
        void move(int goalPos, unsigned int moveTime, unsigned int moveSlopeTime);
        void updatePID_ext(double setPoint, Encoder enc);
        bool pathFollowing;
    private:
        void _updatePID();
        void _updatePIDinit();
        int _setRotDir(int val);
        int _setRotDirInit(int val);
        double _getEncCountDouble();
        double _getRotAngle();
        void _calculatePathVars();
        void _followPath();
        void _followStartSlope();
        void _followStraightLine();
        void _followEndSlope();
        int _currentEncCount;  
        int _motorDriverIN1pin;
        int _motorDriverIN2pin;
        int _motorDriverPWMpin; 
        int _mpuAddPin;
        int _cntr;
        int _mode;
        double _goalPos;
        double _startPos;
        unsigned long _startTime;
        unsigned long _currentTime;
        unsigned long _passedTime;
        unsigned int _moveTime;
        unsigned int _moveSlopeTime;
        unsigned int _moveStraightTime;
        double _distance;
        double _maxSpeed;
        double _slope;
        double _startSlopeEndPos;
        double _straightMoveEndPos;
        // 1 : moving in the positive direction
        // 2 : moving towards the negative
        // 3 : not moving at all
        bool _debug;
        double _calib;
};

#endif














