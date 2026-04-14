#pragma once

#include "robot_config.h"

struct Pose2D {
    float x_mm;
    float y_mm;
    float theta_rad;
};

struct MotionCommand {
    float vx_body_mm_s;
    float vy_body_mm_s;
    bool  rotating;
};

struct SensorReadings {
    float front_mm;
    float rear_mm;
    float left_mm;
    float right_mm;
    float us_right_mm;
    float gyro_z_rad_s;
};

struct Covariance3x3 {
    float m[3][3];
};

class pose_estimator
{
public:
    pose_estimator();

    void begin();
    void resetPose(const Pose2D &pose,
                   float xy_std_mm = 20.0f,
                   float theta_std_rad = 3.0f * robot_config::kDegToRad);

    void predict(const MotionCommand &cmd, float gyro_z_rad_s, float dt_s);
    void correct(const SensorReadings &readings);

    Pose2D       getPose() const;
    float        getHeadingDeg() const;
    Covariance3x3 getCovariance() const;
    bool         isPoseInitialized() const;

private:
    Pose2D        pose;
    Covariance3x3 covariance;
    MotionCommand last_command;
    bool          pose_initialized;

    static float wrapAngleRad(float angle_rad);
    static float wrapAngleDeg(float angle_deg);
    static float square(float value);

    bool  nearestAxis(float *axis_theta_rad) const;
    void  applyScalarMeasurement(float measurement,
                                 float predicted,
                                 const float H[3],
                                 float variance,
                                 float innovation_gate,
                                 bool wrap_innovation = false);
    bool  predictRangeForSensor(float forward_offset_mm,
                                float left_offset_mm,
                                float yaw_offset_rad,
                                float *predicted_mm) const;
    void  numericalJacobian(float forward_offset_mm,
                            float left_offset_mm,
                            float yaw_offset_rad,
                            float jacobian[3]) const;
};
