#pragma once
#include <box2d/b2_math.h>
#include <cmath>
#include <cstdint>
#include <stdlib.h>
#include <unistd.h>

constexpr float SAFE_ANGLE = M_PI_2;
constexpr float MAX_TURN = M_PI;
constexpr float ROBOT_HALFWIDTH = 0.1275; //local x axis
constexpr float ROBOT_HALFLENGTH = 0.08;  //local y axis
constexpr float ROBOT_BOX_OFFSET_X = 0.105 - ROBOT_HALFWIDTH;
constexpr float ROBOT_BOX_OFFSET_Y = 0;
constexpr float ROBOT_BOX_OFFSET_ANGLE = 0;
constexpr float BETWEEN_WHEELS = .14;
constexpr float MAX_SPEED = .78;
constexpr float MAX_OMEGA = 1.8; //radians
constexpr float ANGLE_ERROR_TOLERANCE = 5 * M_PI / 180;
constexpr float BOX2DRANGE = 5.0;
constexpr float LIDAR_RANGE = 1.01;
constexpr float HZ = 10.0;
constexpr float MAX_ANGLE_ERROR = M_PI;
constexpr float MAX_DISTANCE_ERROR = 2 * BOX2DRANGE;
constexpr float LIDAR_SAMPLING_RATE = 0.1;
constexpr float DISTANCE_ERROR_TOLERANCE = .02; //0.02
constexpr float RELAXED_DIST_ERROR_TOLERANCE = 0.06;
constexpr float D_POSE_MARGIN = 0.065;
constexpr float D_DIMENSIONS_MARGIN = 0.03;
static const b2Transform b2Transform_zero
    = b2Transform (b2Vec2_zero, b2Rot (0));
static const b2Transform b2Transform_inf
    = b2Transform (b2Vec2 (10000, 10000), b2Rot (MAX_ANGLE_ERROR));
constexpr float ANGLE_RESOLUTION = M_PI / (2 * HZ);
constexpr float MIN_BODY_DIMENSION = 0.0005;
constexpr float DEG_TO_RAD_K = 0.01745329252;
constexpr float SIM_DURATION = int (BOX2DRANGE * 2 / MAX_SPEED);

//KINEMATICS

constexpr float WHEEL_SPEED_DEFAULT = 0.5f;
constexpr float WHEEL_SPEED_TURN = (M_PI_2 * BETWEEN_WHEELS) / (MAX_SPEED);

static const b2Vec2 GRAVITY (0, 0);

// Flags for the different bodies
constexpr uintptr_t ROBOT_FLAG = 0x1;
constexpr uintptr_t DISTURBANCE_FLAG = 0x2;

// Worldbuilder
constexpr float HALF_WINDOW_WIDTH = 0.15;

// Abstract task
constexpr float NEARBY_MAX_OBSTACLE_DETECTION_ANGLE = M_PI / 2;
static const float NEARBY_OBSTACLE_DETECTION_RADIUS
    = std::max (ROBOT_HALFLENGTH, ROBOT_HALFWIDTH) * 2;

// Avoidance task
constexpr int MIN_LIDAR_AVOID_SAMPLES = 5;
constexpr int AVOID_TASK_STEERING_GAIN = 2;

// Targeting task
constexpr float TARGET_STEERING_GAIN = 5;
