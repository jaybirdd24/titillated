#include "movement.h"
#include <Arduino.h>

// Pin assignments matching base code shield pinout
static const byte PIN_LEFT_FRONT  = 46;
static const byte PIN_LEFT_REAR   = 47;
static const byte PIN_RIGHT_REAR  = 50;
static const byte PIN_RIGHT_FRONT = 51;

// PWM neutral (stop) in microseconds
static const int PWM_NEUTRAL = 1500;

// PID tuning — adjust during physical testing
static const float DEFAULT_KP = 40.0f;
static const float DEFAULT_KI = 0.02f;
static const float DEFAULT_KD = 5.0f;

movement::movement(percepetion *perception)
    : perception(perception),
      heading(0.0f), target_heading(0.0f),
      last_update_us(0),
      Kp(DEFAULT_KP), Ki(DEFAULT_KI), Kd(DEFAULT_KD),
      integral(0.0f), prev_error(0.0f), filtered_derivative(0.0f),
      integral_vy(0.0f), prev_error_vy(0.0f), filtered_derivative_vy(0.0f),
      last_wall_us(0),
      last_vx(0), last_vy(0), last_wz(0),
      current_speeds{0, 0, 0, 0}, last_slew_us(0)
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

void movement::setMotorSpeeds(int lf, int lr, int rr, int rf)
{
    // Compute dt for slew-rate limiting
    unsigned long now = micros();
    float dt = (now - last_slew_us) / 1e6f;
    last_slew_us = now;

    // Cap dt to avoid huge jumps after pauses
    if (dt > 0.1f) dt = 0.1f;

    float accel_delta = MAX_ACCEL_RATE * dt;
    float decel_delta = MAX_DECEL_RATE * dt;

    float target[4] = {(float)lf, (float)lr, (float)rr, (float)rf};
    for (int i = 0; i < 4; i++) {
        float diff = target[i] - current_speeds[i];
        // Accelerating = moving away from zero, decelerating = moving toward zero
        bool accelerating = (fabsf(target[i]) > fabsf(current_speeds[i]));
        float max_delta = accelerating ? accel_delta : decel_delta;
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
    // Hold current global heading as the new target; reset only PID state
    target_heading = heading;
    integral = 0.0f;
    prev_error = 0.0f;
    filtered_derivative = 0.0f;
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

    // Integrate gyroZ (rad/s) into heading (degrees)
    float gyroZ = perception->getGyroZ();
    heading += gyroZ * (180.0f / PI) * dt;

    float error = -target_heading + heading;

    integral += error * dt;
    integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    float raw_derivative = (error - prev_error) / dt;
    filtered_derivative = 0.7f * filtered_derivative + 0.3f * raw_derivative;
    prev_error = error;

    return Kp * error + Ki * integral + Kd * filtered_derivative;
}

void movement::drive(int vx, int vy, int wz)
{
    last_vx = vx; last_vy = vy; last_wz = wz;
    // Mecanum IK — verified against existing motor sign conventions:
    //   vx+  → forward  (LF+, LR+, RR-, RF-)
    //   vy+  → strafe left (LF-, LR+, RR+, RF-)
    //   wz+  → CCW rotation correction
    setMotorSpeeds( vx - vy + wz,   // LF
                    vx + vy + wz,   // LR
                   -vx + vy + wz,   // RR
                   -vx - vy + wz);  // RF
}

float movement::wallFollowCorrection(float setpoint_mm, bool followLeft)
{
    unsigned long now = micros();
    float dt = (now - last_wall_us) / 1e6f;
    last_wall_us = now;

    if (dt > 0.1f) dt = 0.1f;

    float measured = followLeft ? perception->getIRLongLeft()
                                : perception->getIRMedRight();
    // Right wall: positive error → too far → strafe left
    // Left wall:  positive error → too close → strafe right (sign flipped)
    float error = followLeft ? (measured - setpoint_mm)
                             : (-measured + setpoint_mm);

    integral_vy += error * dt;
    integral_vy = constrain(integral_vy, -MAX_INTEGRAL_VY, MAX_INTEGRAL_VY);

    float raw_deriv = (error - prev_error_vy) / dt;
    filtered_derivative_vy = 0.7f * filtered_derivative_vy + 0.3f * raw_deriv;
    prev_error_vy = error;




    return KP_VY * error + KI_VY * integral_vy + KD_VY * filtered_derivative_vy;
}

void movement::MoveForward(int speed)
{
    drive(speed, 0, (int)headingCorrection());
}

void movement::MoveBackward(int speed)
{
    drive(-speed, 0, (int)headingCorrection());
}

void movement::MoveLeft(int speed)
{
    drive(0, speed, (int)headingCorrection());
}

void movement::MoveRight(int speed)
{
    drive(0, -speed, (int)headingCorrection());
}

void movement::RotateCW(int speed)
{
    unsigned long now = micros();
    float dt = (now - last_update_us) / 1e6f;
    last_update_us = now;
    heading += perception->getGyroZ() * (180.0f / PI) * dt;
    setMotorSpeeds(speed, speed, speed, speed);
}

void movement::RotateCCW(int speed)
{
    unsigned long now = micros();
    float dt = (now - last_update_us) / 1e6f;
    last_update_us = now;
    heading += perception->getGyroZ() * (180.0f / PI) * dt;
    setMotorSpeeds(-speed, -speed, -speed, -speed);
}

void movement::resetHeading()
{
    heading = 0.0f;
    target_heading = 0.0f;
    integral = 0.0f;
    prev_error = 0.0f;
    filtered_derivative = 0.0f;
    last_update_us = micros();
}

void movement::resetWallFollow()
{
    integral_vy = 0.0f;
    prev_error_vy = 0.0f;
    filtered_derivative_vy = 0.0f;
    last_wall_us = micros();
}

void movement::Stop(bool immediate)
{
    if (immediate) {
        current_speeds[0] = current_speeds[1] = current_speeds[2] = current_speeds[3] = 0;
        left_front_motor.writeMicroseconds(PWM_NEUTRAL);
        left_rear_motor.writeMicroseconds(PWM_NEUTRAL);
        right_rear_motor.writeMicroseconds(PWM_NEUTRAL);
        right_front_motor.writeMicroseconds(PWM_NEUTRAL);
    } else {
        setMotorSpeeds(0, 0, 0, 0);
    }
    latchHeading();
}
