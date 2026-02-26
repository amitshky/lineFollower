#ifndef TYPES_H_
#define TYPES_H_

#include <webots/robot.h>

typedef enum { FORWARD, LEFT, RIGHT } State;

typedef struct {
    double left;
    double right;
} WheelSpeed, WheelPosition, MotorVelocity;

typedef struct {
    double radius;    // radius of the wheels
    double distance;  // distance between the wheels
} WheelProps;

typedef struct {
    double linear;
    double angular;
} RobotSpeeds;

typedef struct {
    double x;
    double y;
    double phi;
} RobotPose;

typedef struct {
    WbDeviceTag left;
    WbDeviceTag right;
} Motors;

typedef struct {
    double position;
    double orientation;
} PoseErrors;

typedef struct {
    double prop;   // proportional term
    double integ;  // integral term
    double deriv;  // derivative term
} PIDCoeff;

#endif  // TYPES_H_
