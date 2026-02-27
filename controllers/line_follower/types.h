#ifndef TYPES_H_
#define TYPES_H_

#include <webots/robot.h>

typedef enum { FORWARD, LEFT, RIGHT } State;

typedef struct {
    WbDeviceTag left;
    WbDeviceTag right;
} Motors, WheelSensors, GroundSensors;

typedef struct {
    double left;
    double right;
} WheelsSpeed, WheelsPosSensor, MotorVelocity;

typedef struct {
    double radius;    // radius of the wheels
    double distance;  // distance between the wheels
} WheelProps;

typedef struct {
    double linear;
    double angular;
} RobotSpeed;

typedef struct {
    // position
    double x;
    double y;
    // orientation
    double phi;
} RobotPose;

typedef struct {
    double prop;   // proportional term
    double integ;  // integral term
    double deriv;  // derivative term
} PIDCoeff;

#endif  // TYPES_H_
