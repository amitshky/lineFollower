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

PoseError calc_pose_errors(const RobotPose actual_pose,
                           const RobotPose desired_pose) {
    PoseError err = {};

    const double x = desired_pose.x - actual_pose.x;
    const double y = desired_pose.y - actual_pose.y;
    err.position = sqrt(x * x + y * y);

    const double desired_phi = atan2(y, x);
    const double phi_err = desired_phi - actual_pose.phi;
    // normalized to [-pi, pi] rad
    err.orientation = atan2(sin(phi_err), cos(phi_err));

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

PoseError pid_controller_pose(const PoseError err, PoseError *const err_prev,
                              PoseError *const err_accumulated,
                              const double deltatime, const PIDCoeff k) {
    return (PoseError){
        .position = pid_controller(err.position, &err_prev->position,
                                   &err_accumulated->position, deltatime, k),
        .orientation =
            pid_controller(err.orientation, &err_prev->orientation,
                           &err_accumulated->orientation, deltatime, k),
    };
}

void go_to_goal(const RobotPose target_pose, const RobotPose curr_pose,
                RobotPose *const old_pose, PoseError *const err_prev,
                PoseError *const err_accumulated, const WheelSensors motors,
                const double deltatime, const PIDCoeff k) {
    const PoseError err = calc_pose_errors(curr_pose, target_pose);
    const PoseError next_pose =
        pid_controller_pose(err, err_prev, err_accumulated, deltatime, k);

    *old_pose = curr_pose;

    WheelSensorVals vel = {
        .left = 5.0 - next_pose.position - next_pose.orientation,
        .right = 5.0 + next_pose.position + next_pose.orientation,
    };
    vel.left = fmax(fmin(vel.left, MAX_SPEED), MAX_SPEED);
    vel.right = fmax(fmin(vel.right, MAX_SPEED), MAX_SPEED);

    wb_motor_set_velocity(motors.left, vel.left);
    wb_motor_set_velocity(motors.right, vel.right);

    printf("err position = %f orientation = %f\n", err.position,
           err.orientation);
    printf("next pose position = %.3f  orientation = %.3f\n",
           next_pose.position, next_pose.orientation);
}
