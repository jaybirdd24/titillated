#include "movement.h"
#include <Arduino.h>
#include "robot_config.h"

// Pin assignments matching base code shield pinout
static const byte PIN_LEFT_FRONT  = 46;
static const byte PIN_LEFT_REAR   = 47;
static const byte PIN_RIGHT_REAR  = 50;
static const byte PIN_RIGHT_FRONT = 51;

// PWM neutral (stop) in microseconds
static const int PWM_NEUTRAL = 1500;

// PID tuning — adjust during physical testing
static const float DEFAULT_KP = 30.0f;
static const float DEFAULT_KI = 0.01f;
static const float DEFAULT_KD = 9.0f;

namespace {

float wrapDegrees(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg <= -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

float scaleSpeed(int speed, float mm_per_sec_at_full)
{
    return (float)constrain(speed, -1000, 1000) * (mm_per_sec_at_full / 1000.0f);
}

}  // namespace

movement::movement(percepetion *perception, pose_estimator *estimator)
    : perception(perception),
      estimator(estimator),
      target_heading(0.0f),
      last_update_us(0),
      Kp(DEFAULT_KP), Ki(DEFAULT_KI), Kd(DEFAULT_KD),
      integral(0.0f), prev_error(0.0f), filtered_derivative(0.0f),
      integral_vy(0.0f), prev_error_vy(0.0f), filtered_derivative_vy(0.0f),
      last_wall_us(0),
      current_speeds{0, 0, 0, 0}, last_slew_us(0),
      current_motion{0.0f, 0.0f, false}
{
}

movement::~movement()
{
}

void movement::enable()
{
    left_front_motor.attach(PIN_LEFT_FRONT);
    left_rear_motor.attach(PIN_LEFT_REAR);
    right_rear_motor.attach(PIN_RIGHT_REAR);
    right_front_motor.attach(PIN_RIGHT_FRONT);
    last_update_us = micros();
    last_slew_us   = micros();
    last_wall_us   = micros();
    current_motion = {0.0f, 0.0f, false};
    target_heading = currentHeadingDeg();
}

void movement::disable()
{
    left_front_motor.detach();
    left_rear_motor.detach();
    right_rear_motor.detach();
    right_front_motor.detach();

    pinMode(PIN_LEFT_FRONT,  INPUT);
    pinMode(PIN_LEFT_REAR,   INPUT);
    pinMode(PIN_RIGHT_REAR,  INPUT);
    pinMode(PIN_RIGHT_FRONT, INPUT);
}

void movement::setPoseEstimator(pose_estimator *next_estimator)
{
    estimator = next_estimator;
    target_heading = currentHeadingDeg();
    resetHeadingPid();
}

void movement::setMotorSpeeds(int lf, int lr, int rr, int rf)
{
    // Compute dt for slew-rate limiting
    unsigned long now = micros();
    float dt = (now - last_slew_us) / 1e6f;
    last_slew_us = now;

    // Cap dt to avoid huge jumps after pauses
    if (dt > 0.1f) dt = 0.1f;

    float max_delta = MAX_SLEW_RATE * dt;

    float target[4] = {(float)lf, (float)lr, (float)rr, (float)rf};
    for (int i = 0; i < 4; i++) {
        float diff = target[i] - current_speeds[i];
        if (diff >  max_delta) diff =  max_delta;
        if (diff < -max_delta) diff = -max_delta;
        current_speeds[i] += diff;
        // Clamp to valid servo range
        current_speeds[i] = constrain(current_speeds[i], -1000.0f, 1000.0f);
    }

    left_front_motor.writeMicroseconds(PWM_NEUTRAL  + (int)current_speeds[0]);
    left_rear_motor.writeMicroseconds(PWM_NEUTRAL   + (int)current_speeds[1]);
    right_rear_motor.writeMicroseconds(PWM_NEUTRAL  + (int)current_speeds[2]);
    right_front_motor.writeMicroseconds(PWM_NEUTRAL + (int)current_speeds[3]);
}

void movement::latchHeading()
{
    // Hold current estimator heading as the new target; reset only PID state.
    target_heading = currentHeadingDeg();
    resetHeadingPid();
    last_update_us = micros();
}

void movement::setTargetHeading(float degrees)
{
    target_heading = degrees;
}

float movement::headingCorrection()
{
    unsigned long now = micros();
    float dt = (now - last_update_us) / 1e6f;
    last_update_us = now;

    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = wrapDegrees(currentHeadingDeg() - target_heading);

    integral += error * dt;
    integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    float raw_derivative = (error - prev_error) / dt;
    filtered_derivative = 0.7f * filtered_derivative + 0.3f * raw_derivative;
    prev_error = error;

    return Kp * error + Ki * integral + Kd * filtered_derivative;
}

void movement::drive(int vx, int vy, int wz)
{
    // Mecanum IK — verified against existing motor sign conventions:
    //   vx+  → forward  (LF+, LR+, RR-, RF-)
    //   vy+  → strafe left (LF-, LR+, RR+, RF-)
    //   wz+  → CCW rotation correction
    setMotorSpeeds( vx - vy + wz,   // LF
                    vx + vy + wz,   // LR
                   -vx + vy + wz,   // RR
                   -vx - vy + wz);  // RF
}

float movement::wallFollowCorrection(float setpoint_mm)
{
    unsigned long now = micros();
    float dt = (now - last_wall_us) / 1e6f;
    last_wall_us = now;

    if (dt > 0.1f) dt = 0.1f;

    float measured = perception->getUltrasonicCm() * 10.0f; // convert to mm for PID calculations   
    float error = -measured + setpoint_mm;  // positive → too far from wall → strafe left

    integral_vy += error * dt;
    integral_vy = constrain(integral_vy, -MAX_INTEGRAL_VY, MAX_INTEGRAL_VY);

    float raw_deriv = (error - prev_error_vy) / dt;
    filtered_derivative_vy = 0.7f * filtered_derivative_vy + 0.3f * raw_deriv;
    prev_error_vy = error;

    return KP_VY * error + KI_VY * integral_vy + KD_VY * filtered_derivative_vy;
}

void movement::MoveForward(int speed)
{
    current_motion = {speedToForwardMmS(speed), 0.0f, false};
    drive(speed, 0, (int)headingCorrection());
}

void movement::MoveBackward(int speed)
{
    current_motion = {-speedToBackwardMmS(speed), 0.0f, false};
    drive(-speed, 0, (int)headingCorrection());
}

void movement::MoveLeft(int speed)
{
    current_motion = {0.0f, speedToLeftMmS(speed), false};
    drive(0, speed, (int)headingCorrection());
}

void movement::MoveRight(int speed)
{
    current_motion = {0.0f, -speedToRightMmS(speed), false};
    drive(0, -speed, (int)headingCorrection());
}

void movement::RotateCW(int speed)
{
    current_motion = {0.0f, 0.0f, true};
    setMotorSpeeds(speed, speed, speed, speed);
}

void movement::RotateCCW(int speed)
{
    current_motion = {0.0f, 0.0f, true};
    setMotorSpeeds(-speed, -speed, -speed, -speed);
}

void movement::resetHeading()
{
    target_heading = currentHeadingDeg();
    resetHeadingPid();
    last_update_us = micros();
}

void movement::Stop(bool immediate)
{
    if (immediate) {
        // Bypass slew-rate limiting for emergency stops
        current_speeds[0] = current_speeds[1] = current_speeds[2] = current_speeds[3] = 0;
        left_front_motor.writeMicroseconds(PWM_NEUTRAL);
        left_rear_motor.writeMicroseconds(PWM_NEUTRAL);
        right_rear_motor.writeMicroseconds(PWM_NEUTRAL);
        right_front_motor.writeMicroseconds(PWM_NEUTRAL);
    } else {
        setMotorSpeeds(0, 0, 0, 0);
    }
    current_motion = {0.0f, 0.0f, false};
    latchHeading();
}

MotionCommand movement::getCurrentMotionCommand() const
{
    return current_motion;
}

float movement::speedToForwardMmS(int speed) const
{
    return scaleSpeed(speed, robot_config::kForwardMmPerSecAtFull);
}

float movement::speedToBackwardMmS(int speed) const
{
    return scaleSpeed(speed, robot_config::kBackwardMmPerSecAtFull);
}

float movement::speedToLeftMmS(int speed) const
{
    return scaleSpeed(speed, robot_config::kLeftMmPerSecAtFull);
}

float movement::speedToRightMmS(int speed) const
{
    return scaleSpeed(speed, robot_config::kRightMmPerSecAtFull);
}

float movement::currentHeadingDeg() const
{
    return estimator != nullptr ? estimator->getHeadingDeg() : 0.0f;
}

void movement::resetHeadingPid()
{
    integral = 0.0f;
    prev_error = 0.0f;
    filtered_derivative = 0.0f;
}
