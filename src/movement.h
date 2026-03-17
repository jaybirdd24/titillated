#pragma once

#include <Servo.h>
#include "percepetion.h"

class movement
{
    private:
        Servo left_front_motor;
        Servo left_rear_motor;
        Servo right_rear_motor;
        Servo right_front_motor;

        percepetion *perception;

        // Heading PID state
        float heading;          // integrated heading (degrees)
        float target_heading;
        unsigned long last_update_us;

        float Kp;
        float Ki;
        float Kd;
        float integral;
        float prev_error;

        // Sets raw PWM microseconds on all 4 motors (1500 = stop)
        void setMotorSpeeds(int lf, int lr, int rr, int rf);

        // Integrates gyroZ into heading, returns PID correction value
        float headingCorrection();

        // Record target_heading and reset PID state; call when starting a new motion
        void latchHeading();

    public:
        movement(percepetion *perception);
        ~movement();

        void enable();           // attach servos to pins
        void disable();          // detach servos

        // speed: 0–1000  (matches reference code speed_val range)
        void MoveForward(int speed);
        void MoveBackward(int speed);
        void MoveLeft(int speed);
        void MoveRight(int speed);
        void Stop();
};
