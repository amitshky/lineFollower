#ifndef UTILS_H_
#define UTILS_H_

#include <math.h>

#include "types.h"

WheelSensorVals get_wheels_speed(const WheelSensorVals encoder_vals,
                                 const WheelSensorVals old_encoder_vals,
                                 const double deltatime);
RobotSpeed get_robot_speeds(const WheelSensorVals speed,
                            const WheelProps props);
RobotPose get_robot_pose(const RobotSpeed speeds, const RobotPose old_pose,
                         const double deltatime);

RobotPose calc_pose_errors(const RobotPose actual, const RobotPose desired);
double pid_controller(const double err, double *const err_prev,
                      double *const err_accumulated, const double deltatime,
                      const PIDCoeff k);

WheelSensorVals wheel_speed_commands(const RobotSpeed desired,
                                     const WheelProps props);

void go_to_goal(const RobotPose target_pose, const RobotPose curr_pose,
                RobotPose *const old_pose, RobotPose *const err_prev,
                RobotPose *const err_accumulated, const WheelSensors motors,
                const double deltatime, const PIDCoeff k);

inline double to_degrees(double rad) { return fmod(rad / M_PI * 180.0, 360.0); }

#endif  // UTILS_H_
