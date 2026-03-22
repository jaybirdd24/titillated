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
static const float DEFAULT_KP = 2.5f;
static const float DEFAULT_KI = 0.0f;
static const float DEFAULT_KD = 0.5f;

movement::movement(percepetion *perception)
    : perception(perception),
      heading(0.0f), target_heading(0.0f),
      last_update_us(0),
      Kp(DEFAULT_KP), Ki(DEFAULT_KI), Kd(DEFAULT_KD),
      integral(0.0f), prev_error(0.0f), filtered_derivative(0.0f)
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
    // Clamp each channel to valid servo range
    lf = constrain(lf, -1000, 1000);
    lr = constrain(lr, -1000, 1000);
    rr = constrain(rr, -1000, 1000);
    rf = constrain(rf, -1000, 1000);

    left_front_motor.writeMicroseconds(PWM_NEUTRAL + lf);
    left_rear_motor.writeMicroseconds(PWM_NEUTRAL  + lr);
    right_rear_motor.writeMicroseconds(PWM_NEUTRAL  + rr);
    right_front_motor.writeMicroseconds(PWM_NEUTRAL + rf);
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

    float error = target_heading - heading;

    integral += error * dt;
    integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    float raw_derivative = (error - prev_error) / dt;
    filtered_derivative = 0.7f * filtered_derivative + 0.3f * raw_derivative;
    prev_error = error;

    return Kp * error + Ki * integral + Kd * filtered_derivative;
}

// Positive correction turns robot CCW (adds to left, subtracts from right)
void movement::MoveForward(int speed)
{
    float correction = headingCorrection();
    int c = (int)correction;

    // Left motors +speed, right motors -speed (reference code sign convention)
    setMotorSpeeds( speed + c,   // left front
                    speed + c,   // left rear
                   -speed + c,   // right rear
                   -speed + c);  // right front
}

void movement::MoveBackward(int speed)
{
    float correction = headingCorrection();
    int c = (int)correction;

    setMotorSpeeds(-speed + c,
                   -speed + c,
                    speed + c,
                    speed + c);
}

// Strafe left: LF-, LR+, RR+, RF-  (reference code strafe_left)
void movement::MoveLeft(int speed)
{
    float correction = headingCorrection();
    int c = (int)correction;

    setMotorSpeeds(-speed + c,
                    speed + c,
                    speed + c,
                   -speed + c);
}

// Strafe right: LF+, LR-, RR-, RF+  (reference code strafe_right)
void movement::MoveRight(int speed)
{
    float correction = headingCorrection();
    int c = (int)correction;

    setMotorSpeeds( speed + c,
                   -speed + c,
                   -speed + c,
                    speed + c);
}

void movement::Stop()
{
    setMotorSpeeds(0, 0, 0, 0);
    latchHeading();  // hold current heading as target; reset PID integrators
}
