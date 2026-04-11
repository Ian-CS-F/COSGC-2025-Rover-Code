#pragma once

// ── Navigation directions ─────────────────────────────────────────────────────
#define NAV_CENTER  0
#define NAV_LEFT    1
#define NAV_RIGHT   2
#define NAV_BACK    3

// ── Drive configuration ───────────────────────────────────────────────────────
#define SEGMENT_CM            30.0f
#define DRIVE_SPEED            0.65f
#define START_DELAY_MS         3000

// ── Obstacle / avoidance ──────────────────────────────────────────────────────
#define OBSTACLE_THRESHOLD_CM  60.0f
#define AVOID_TURN_DEG         30.0f
#define AVOID_TURN_PWM         170
#define MS_PER_DEGREE          12

// ── Cliff detection ───────────────────────────────────────────────────────────
#define CLIFF_TILT_DEG         140
#define CLIFF_TILT_SETTLE_MS   300
#define CLIFF_THRESHOLD_CM     40.0f

// ── Heading hold ──────────────────────────────────────────────────────────────
#define IMU_KP                 8.0f
#define ENCODER_KP             4.0f
#define MAX_CORRECTION         60

// ── Hardware ──────────────────────────────────────────────────────────────────
#define PIN_ENA  4
#define PIN_IN1  6
#define PIN_IN2  7
#define PIN_ENB  5
#define PIN_IN3  8
#define PIN_IN4  9

#define PIN_TILT   12
#define PIN_TILT2  13

#define PIN_ULTRA_TRIG  A0
#define PIN_ULTRA_ECHO  A1

#define ENCODER_A_LEFT   2
#define ENCODER_B_LEFT   10
#define ENCODER_A_RIGHT  3
#define ENCODER_B_RIGHT  11

#define COUNTS_PER_REV   64
#define DIST_PER_REV_CM  35.2f
#define MOTOR_TIMEOUT_MS 30000UL

// ── Heightmap ─────────────────────────────────────────────────────────────────
#define SENSOR_HEIGHT_CM   20.0f
#define SCAN_TILT_MIN      90
#define SCAN_TILT_MAX      140
#define SCAN_STEPS         10
#define SCAN_SETTLE_MS     250
#define OBSTACLE_MIN_CM    5.0f
#define OBSTACLE_MAX_CM    60.0f
#define CLIFF_DROP_CM      10.0f
