#include "utils.h"

#include <math.h>
#include <stdio.h>
#include <webots/motor.h>

#include "commons.h"

WheelSensorVals get_wheels_speed(const WheelSensorVals encoder_vals,
                                 const WheelSensorVals old_encoder_vals,
                                 const double deltatime) {
    return (WheelSensorVals){
        .left = (encoder_vals.left - old_encoder_vals.left) / deltatime,
        .right = (encoder_vals.right - old_encoder_vals.right) / deltatime,
    };
}

RobotSpeed get_robot_speeds(const WheelSensorVals speed,
                            const WheelProps props) {
    return (RobotSpeed){
        // linear speed is the average speed of the 2 wheels
        .linear = props.radius / 2.0 * (speed.right + speed.left),
        .angular = props.radius / props.distance * (speed.right - speed.left),
    };
}

RobotPose get_robot_pose(const RobotSpeed speed, const RobotPose old_pose,
                         const double deltatime) {
    RobotPose pose = {};
    RobotPose delta = {};

    delta.phi = speed.angular * deltatime;
    pose.phi = old_pose.phi + delta.phi;

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

RobotPose calc_pose_errors(const RobotPose actual, const RobotPose desired) {
    RobotPose err = {};

    err.x = desired.x - actual.x;
    err.y = desired.y - actual.y;

    const double desired_phi = atan2(err.y, err.x);
    const double diff_phi = desired_phi - actual.phi;
    // normalized to [-pi, pi] rad
    err.phi = atan2(sin(diff_phi), cos(diff_phi));

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

WheelSensorVals wheel_speed_commands(const RobotSpeed desired,
                                     const WheelProps props) {
    WheelSensorVals speeds = {
        .left = (2.0 * desired.linear + props.distance * desired.angular) /
                (2.0 * props.radius),
        .right = (2.0 * desired.linear - props.distance * desired.angular) /
                 (2.0 * props.radius),
    };
    return speeds;
}

void go_to_goal(const RobotPose target_pose, const RobotPose curr_pose,
                RobotPose *const old_pose, RobotPose *const err_prev,
                RobotPose *const err_accumulated, const WheelSensors motors,
                const double deltatime, const PIDCoeff k) {
    const RobotPose err = calc_pose_errors(curr_pose, target_pose);
    const double phi = pid_controller(err.phi, &err_prev->phi,
                                      &err_accumulated->phi, deltatime, k);

    *old_pose = curr_pose;

    WheelSensorVals vel = {
        .left = 5.0 - phi,
        .right = 5.0 + phi,
    };
    vel.left = fmax(fmin(vel.left, MAX_SPEED), MAX_SPEED);
    vel.right = fmax(fmin(vel.right, MAX_SPEED), MAX_SPEED);

    wb_motor_set_velocity(motors.left, vel.left);
    wb_motor_set_velocity(motors.right, vel.right);
}
