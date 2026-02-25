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

WheelSpeed get_wheels_speed(const WheelPosition pos,
                            const WheelPosition old_pos, int deltatime);
RobotSpeeds get_robot_speeds(const WheelSpeed speed, const WheelProps props);
RobotPose get_robot_pose(const RobotSpeeds speeds, const RobotPose old_pose,
                         const double deltatime);

PoseErrors calc_pose_errs(const RobotPose actual_pose,
                          const RobotPose desired_pose);
double pid_controller(const double err, double *const err_prev,
                      double *const err_accumulated, const double deltatime,
                      const PIDCoeff k);

int main(int argc, char **argv) {
    wb_robot_init();

    const Motors motors = { .left = wb_robot_get_device("left wheel motor"),
                            .right = wb_robot_get_device("right wheel motor") };
    wb_motor_set_position(motors.left, INFINITY);
    wb_motor_set_position(motors.right, INFINITY);
    wb_motor_set_velocity(motors.left, 0.0);
    wb_motor_set_velocity(motors.right, 0.0);

    WbDeviceTag encoders[2] = {};
    char *encoder_names[2] = {
        "left wheel sensor",
        "right wheel sensor",
    };

    for (int i = 0; i < 2; ++i) {
        encoders[i] = wb_robot_get_device(encoder_names[i]);
        wb_position_sensor_enable(encoders[i], TIME_STEP);
    }

    WbDeviceTag ground_sensors[2] = {};
    char *gs_names[2] = { "gs0", "gs2" };

    for (int i = 0; i < 2; ++i) {
        ground_sensors[i] = wb_robot_get_device(gs_names[i]);
        wb_distance_sensor_enable(ground_sensors[i], TIME_STEP);
    }

    int counter = 0;
    const int counter_max = 5;
    State state = FORWARD;
    const double deltatime = TIME_STEP / 1000.0;  // in seconds

    const WheelProps props = { .radius = 0.0205, .distance = 0.052 };

    WheelPosition old_pos = { .left = wb_position_sensor_get_value(encoders[0]),
                              .right =
                                  wb_position_sensor_get_value(encoders[1]) };

    RobotPose old_pose = {};

    while (wb_robot_step(TIME_STEP) != -1) {
        follow_line(ground_sensors, motors, &state, &counter, counter_max);

        WheelPosition pos = { .left = wb_position_sensor_get_value(encoders[0]),
                              .right =
                                  wb_position_sensor_get_value(encoders[1]) };
        WheelSpeed wheel_speed = get_wheels_speed(pos, old_pos, deltatime);
        old_pos = pos;
        printf("speed left = %f    right = %f\n", wheel_speed.left,
               wheel_speed.right);

        RobotSpeeds speeds = get_robot_speeds(wheel_speed, props);
        printf("robot speeds linear = %f    angular = %f\n", speeds.linear,
               speeds.angular);

        RobotPose pose = get_robot_pose(speeds, old_pose, deltatime);
        printf("robot pose x = %f    y = %f    phi = %f\n", pose.x, pose.y,
               pose.phi);
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

WheelSpeed get_wheels_speed(const WheelPosition pos,
                            const WheelPosition old_pos, int deltatime) {
    const double distance_left = pos.left - old_pos.left;
    const double distance_right = pos.left - old_pos.right;

    return (WheelSpeed){ .left = distance_left / deltatime,
                         .right = distance_right / deltatime };
}

RobotSpeeds get_robot_speeds(const WheelSpeed speed, const WheelProps props) {
    return (RobotSpeeds){
        .linear = props.radius / 2.0 * (speed.right + speed.left),
        .angular = props.radius / props.distance * (speed.right - speed.left)
    };
}

RobotPose get_robot_pose(const RobotSpeeds speeds, const RobotPose old_pose,
                         const double deltatime) {
    RobotPose pose = {};
    RobotPose delta = {};

    delta.phi = speeds.angular * deltatime;
    pose.phi = delta.phi + old_pose.phi;

    // Normalize to [-pi, pi]
    if (pose.phi > M_PI) {
        pose.phi -= 2 * M_PI;
    } else if (pose.phi < -M_PI) {
        pose.phi += 2 * M_PI;
    }

    delta.x = speeds.linear * cos(pose.phi) * deltatime;
    delta.y = speeds.linear * sin(pose.phi) * deltatime;
    pose.x = old_pose.x + delta.x;
    pose.y = old_pose.y + delta.y;

    return pose;
}

// desired_pose.phi is calculated in within this function so
// you don't have to pass it
PoseErrors calc_pose_errs(const RobotPose actual_pose,
                          const RobotPose desired_pose) {
    PoseErrors err = {};

    const double x_err = desired_pose.x - actual_pose.x;
    const double y_err = desired_pose.y - actual_pose.y;
    err.position = sqrt(x_err * x_err + y_err * y_err);

    const double d_phi = atan2(y_err, x_err);
    const double phi_err = d_phi - actual_pose.phi;
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
