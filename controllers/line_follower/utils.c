#include "utils.h"

#include <stdio.h>

WheelSensorVals get_wheels_speed(const WheelSensorVals encoder_vals,
                                 const WheelSensorVals old_encoder_vals,
                                 const double deltatime) {
    const double distance_left = encoder_vals.left - old_encoder_vals.left;
    const double distance_right = encoder_vals.right - old_encoder_vals.right;

    return (WheelSensorVals){
        .left = distance_left / deltatime,
        .right = distance_right / deltatime,
    };
}

RobotSpeed get_robot_speeds(const WheelSensorVals speed,
                            const WheelProps props) {
    return (RobotSpeed){
        .linear = props.radius / 2.0 * (speed.right + speed.left),
        .angular = props.radius / props.distance * (speed.right - speed.left),
    };
}

RobotPose get_robot_pose(const RobotSpeed speed, const RobotPose old_pose,
                         const double deltatime) {
    RobotPose pose = {};
    RobotPose delta = {};

    delta.phi = speed.angular * deltatime;
    pose.phi = delta.phi + old_pose.phi;

    // Normalize to [-pi, pi]
    if (pose.phi > M_PI) {
        pose.phi -= 2 * M_PI;
    } else if (pose.phi < -M_PI) {
        pose.phi += 2 * M_PI;
    }

    delta.x = speed.linear * cos(pose.phi) * deltatime;
    delta.y = speed.linear * sin(pose.phi) * deltatime;

    pose.x = old_pose.x + delta.x;
    pose.y = old_pose.y + delta.y;

    return pose;
}

RobotPose calc_pose_errs(const RobotPose actual_pose,
                         const RobotPose desired_pose) {
    RobotPose err = {};

    err.x = desired_pose.x - actual_pose.x;
    err.y = desired_pose.y - actual_pose.y;

    const double desired_phi = atan2(err.y, err.x);
    const double phi_err = desired_phi - actual_pose.phi;
    // normalized to [-pi, pi] rad
    err.phi = atan2(sin(phi_err), cos(phi_err));

    return err;
}

double pid_controller(const double err, double *const err_prev,
                      double *const err_accumulated, const double deltatime,
                      const PIDCoeff k) {
    PIDCoeff output = {};

    output.prop = k.prop * err;
    output.integ = (k.integ * err * deltatime) + (*err_accumulated);
    output.deriv = k.deriv * (err - (*err_prev)) / deltatime;

    *err_prev = err;
    *err_accumulated = output.integ;

    return output.prop + output.integ + output.deriv;
}

RobotPose pid_controller_pose(const RobotPose err, RobotPose *const err_prev,
                              RobotPose *const err_accumulated,
                              const double deltatime, const PIDCoeff k) {
    return (RobotPose){
        .x = pid_controller(err.x, &err_prev->x, &err_accumulated->x, deltatime,
                            k),
        .y = pid_controller(err.y, &err_prev->y, &err_accumulated->y, deltatime,
                            k),
        .phi = pid_controller(err.phi, &err_prev->phi, &err_accumulated->phi,
                              deltatime, k),
    };
}

void go_to_goal(const RobotPose target_pose, const RobotPose curr_pose,
                RobotPose *const old_pose, RobotPose *const err_prev,
                RobotPose *const err_accumulated, const double deltatime,
                const PIDCoeff k) {
    const RobotPose err = calc_pose_errs(curr_pose, target_pose);
    const RobotPose next_pose =
        pid_controller_pose(err, err_prev, err_accumulated, deltatime, k);

    *old_pose = curr_pose;

    printf("pid x = %.3f  y = %.3f  phi = %.3f\n", next_pose.x, next_pose.y,
           next_pose.phi);
}
