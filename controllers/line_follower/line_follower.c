#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "types.h"

#define TIME_STEP 64
#define MAX_SPEED 6.28

void follow_line(const WbDeviceTag ground_sensors[2], const Motors motors,
                 State *const state, int *const counter, const int counter_max);

WheelsSpeed get_wheels_speed(const WheelsPosSensor ps,
                             const WheelsPosSensor old_ps,
                             const double deltatime);
RobotSpeed get_robot_speeds(const WheelsSpeed speed, const WheelProps props);
RobotPose get_robot_pose(const RobotSpeed speeds, const RobotPose old_pose,
                         const double deltatime);

RobotPose calc_pose_errs(const RobotPose actual_pose,
                         const RobotPose desired_pose);
double pid_controller(const double err, double *const err_prev,
                      double *const err_accumulated, const double deltatime,
                      const PIDCoeff k);
void go_to_goal(const RobotPose target_pose, const RobotPose curr_pose,
                RobotPose *const old_pose, RobotPose *const err_prev,
                RobotPose *const err_accumulated, const double deltatime,
                const PIDCoeff k);

int main(void) {
    wb_robot_init();

    const Motors motors = { .left = wb_robot_get_device("left wheel motor"),
                            .right = wb_robot_get_device("right wheel motor") };
    wb_motor_set_position(motors.left, INFINITY);
    wb_motor_set_position(motors.right, INFINITY);
    wb_motor_set_velocity(motors.left, 0.0);
    wb_motor_set_velocity(motors.right, 0.0);

    WbDeviceTag encoders[2] = {};
    const char *const encoder_names[2] = {
        "left wheel sensor",
        "right wheel sensor",
    };

    for (int i = 0; i < 2; ++i) {
        encoders[i] = wb_robot_get_device(encoder_names[i]);
        wb_position_sensor_enable(encoders[i], TIME_STEP);
    }

    WbDeviceTag ground_sensors[2] = {};
    const char *const gs_names[2] = { "gs0", "gs2" };

    for (int i = 0; i < 2; ++i) {
        ground_sensors[i] = wb_robot_get_device(gs_names[i]);
        wb_distance_sensor_enable(ground_sensors[i], TIME_STEP);
    }

    /*State state = FORWARD;*/
    /*int counter = 0;*/
    /*const int counter_max = 5;*/
    const double deltatime = TIME_STEP / 1000.0;  // in seconds
    const PIDCoeff k = { .prop = 1.8, .integ = 1.2, .deriv = 0.9 };
    const WheelProps props = { .radius = 0.0205, .distance = 0.052 };

    WheelsPosSensor old_ps = {};
    RobotPose old_pose = {};
    RobotPose err_prev = {};
    RobotPose err_accumulated = {};
    const RobotPose target_pose = { 1.0, 1.0, 0.0 };

    while (wb_robot_step(TIME_STEP) != -1) {
        /*follow_line(ground_sensors, motors, &state, &counter, counter_max);*/

        const WheelsPosSensor ps = {
            .left = wb_position_sensor_get_value(encoders[0]),
            .right = wb_position_sensor_get_value(encoders[1])
        };
        const WheelsSpeed wheel_speed = get_wheels_speed(ps, old_ps, deltatime);
        const RobotSpeed speeds = get_robot_speeds(wheel_speed, props);
        const RobotPose curr_pose = get_robot_pose(speeds, old_pose, deltatime);

        go_to_goal(target_pose, curr_pose, &old_pose, &err_prev,
                   &err_accumulated, deltatime, k);
    };

    wb_robot_cleanup();

    return 0;
}

void follow_line(const WbDeviceTag ground_sensors[2], const Motors motors,
                 State *const state, int *const counter,
                 const int counter_max) {
    MotorVelocity vel = {};

    const double right_sensor = wb_distance_sensor_get_value(ground_sensors[0]);
    const double left_sensor = wb_distance_sensor_get_value(ground_sensors[1]);

    const bool line_right = right_sensor > 600.0;
    const bool line_left = left_sensor > 600.0;

    if (*state == FORWARD) {
        vel.left = MAX_SPEED;
        vel.right = MAX_SPEED;

        if (line_right && !line_left) {
            *state = RIGHT;
            *counter = 0;
        } else if (line_left && !line_right) {
            *state = LEFT;
            *counter = 0;
        }
    }

    if (*state == RIGHT) {
        vel.left = 0.8 * MAX_SPEED;
        vel.right = 0.5 * MAX_SPEED;

        if (*counter == counter_max) {
            *state = FORWARD;
        }
    }

    if (*state == LEFT) {
        vel.left = 0.5 * MAX_SPEED;
        vel.right = 0.8 * MAX_SPEED;

        if (*counter == counter_max) {
            *state = FORWARD;
        }
    }

    ++(*counter);

    wb_motor_set_velocity(motors.left, vel.left);
    wb_motor_set_velocity(motors.right, vel.right);
}

WheelsSpeed get_wheels_speed(const WheelsPosSensor ps,
                             const WheelsPosSensor old_ps,
                             const double deltatime) {
    const double distance_left = ps.left - old_ps.left;
    const double distance_right = ps.right - old_ps.right;

    return (WheelsSpeed){ .left = distance_left / deltatime,
                          .right = distance_right / deltatime };
}

RobotSpeed get_robot_speeds(const WheelsSpeed speed, const WheelProps props) {
    return (RobotSpeed){
        .linear = props.radius / 2.0 * (speed.right + speed.left),
        .angular = props.radius / props.distance * (speed.right - speed.left)
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

// desired_pose.phi is calculated in within this function so
// you don't have to pass it
RobotPose calc_pose_errs(const RobotPose actual_pose,
                         const RobotPose desired_pose) {
    RobotPose err = {};

    err.x = desired_pose.x - actual_pose.x;
    err.y = desired_pose.y - actual_pose.y;

    const double d_phi = atan2(err.y, err.x);
    const double phi_err = d_phi - actual_pose.phi;
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

void go_to_goal(const RobotPose target_pose, const RobotPose curr_pose,
                RobotPose *const old_pose, RobotPose *const err_prev,
                RobotPose *const err_accumulated, const double deltatime,
                const PIDCoeff k) {
    const RobotPose err = calc_pose_errs(curr_pose, target_pose);

    const RobotPose next_pose = {
        .x = pid_controller(err.x, &err_prev->x, &err_accumulated->x, deltatime,
                            k),
        .y = pid_controller(err.y, &err_prev->y, &err_accumulated->y, deltatime,
                            k),
        .phi = pid_controller(err.phi, &err_prev->phi, &err_accumulated->phi,
                              deltatime, k)
    };

    *old_pose = curr_pose;

    printf("pid x = %.3f  y = %.3f  phi = %.3f\n", next_pose.x, next_pose.y,
           next_pose.phi);
}
