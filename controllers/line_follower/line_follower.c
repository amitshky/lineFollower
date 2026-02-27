#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include "types.h"
#include "utils.h"

#define TIME_STEP 64
#define MAX_SPEED 6.28

void follow_line(const WheelSensors ground_sensors, const WheelSensors motors,
                 State *const state, int *const counter, const int counter_max);

int main(void) {
    wb_robot_init();

    const WheelSensors motors = {
        .left = wb_robot_get_device("left wheel motor"),
        .right = wb_robot_get_device("right wheel motor"),
    };
    wb_motor_set_position(motors.left, INFINITY);
    wb_motor_set_position(motors.right, INFINITY);
    wb_motor_set_velocity(motors.left, 0.0);
    wb_motor_set_velocity(motors.right, 0.0);

    const WheelSensors encoders = {
        .left = wb_robot_get_device("left wheel sensor"),
        .right = wb_robot_get_device("right wheel sensor"),
    };
    wb_position_sensor_enable(encoders.left, TIME_STEP);
    wb_position_sensor_enable(encoders.right, TIME_STEP);

    const WheelSensors ground_sensors = {
        .left = wb_robot_get_device("gs0"),
        .right = wb_robot_get_device("gs2"),
    };
    wb_distance_sensor_enable(ground_sensors.left, TIME_STEP);
    wb_distance_sensor_enable(ground_sensors.right, TIME_STEP);

    // State state = FORWARD;
    // int counter = 0;
    // const int counter_max = 5;

    const double deltatime = TIME_STEP / 1000.0;  // in seconds
    const PIDCoeff k = { .prop = 1.8, .integ = 1.2, .deriv = 0.9 };
    const WheelProps props = { .radius = 0.0205, .distance = 0.052 };

    WheelSensorVals old_encoder_vals = {};
    RobotPose old_pose = {};
    RobotPose err_prev = {};
    RobotPose err_accumulated = {};
    const RobotPose target_pose = { 1.0, 1.0, 0.0 };

    while (wb_robot_step(TIME_STEP) != -1) {
        // follow_line(ground_sensors, motors, &state, &counter, counter_max);

        const WheelSensorVals encoder_vals = {
            .left = wb_position_sensor_get_value(encoders.left),
            .right = wb_position_sensor_get_value(encoders.right),
        };
        printf("encoder_vals left = %f right = %f\n", encoder_vals.left,
               encoder_vals.right);
        const WheelSensorVals wheel_speed =
            get_wheels_speed(encoder_vals, old_encoder_vals, deltatime);
        const RobotSpeed speeds = get_robot_speeds(wheel_speed, props);
        const RobotPose curr_pose = get_robot_pose(speeds, old_pose, deltatime);

        go_to_goal(target_pose, curr_pose, &old_pose, &err_prev,
                   &err_accumulated, deltatime, k);
    };

    wb_robot_cleanup();

    return 0;
}

void follow_line(const WheelSensors ground_sensors, const WheelSensors motors,
                 State *const state, int *const counter,
                 const int counter_max) {
    WheelSensorVals vel = {};

    const double right_sensor =
        wb_distance_sensor_get_value(ground_sensors.left);
    const double left_sensor =
        wb_distance_sensor_get_value(ground_sensors.right);

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
