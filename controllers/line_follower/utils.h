#ifndef UTILS_H_
#define UTILS_H_

#include "types.h"

WheelSensorVals get_wheels_speed(const WheelSensorVals encoder_vals,
                                 const WheelSensorVals old_encoder_vals,
                                 const double deltatime);
RobotSpeed get_robot_speeds(const WheelSensorVals speed,
                            const WheelProps props);
RobotPose get_robot_pose(const RobotSpeed speeds, const RobotPose old_pose,
                         const double deltatime);

RobotPose calc_pose_errs(const RobotPose actual_pose,
                         const RobotPose desired_pose);
double pid_controller(const double err, double *const err_prev,
                      double *const err_accumulated, const double deltatime,
                      const PIDCoeff k);
RobotPose pid_controller_pose(const RobotPose err, RobotPose *const err_prev,
                              RobotPose *const err_accumulated,
                              const double deltatime, const PIDCoeff k);
void go_to_goal(const RobotPose target_pose, const RobotPose curr_pose,
                RobotPose *const old_pose, RobotPose *const err_prev,
                RobotPose *const err_accumulated, const double deltatime,
                const PIDCoeff k);

#endif  // UTILS_H_
