#include "pose_estimator.h"

#include <math.h>

namespace {

constexpr float kFiniteDiffTheta = 0.01f;
constexpr float kFiniteDiffPos = 1.0f;
constexpr float kAxisAngles[4] = {
    0.0f,
    0.5f * robot_config::kPi,
    robot_config::kPi,
    -0.5f * robot_config::kPi,
};

float absf(float value) {
    return value < 0.0f ? -value : value;
}

bool inRange(float value, float min_value, float max_value) {
    return value >= min_value && value <= max_value;
}

}  // namespace

pose_estimator::pose_estimator()
    : pose{0.0f, 0.0f, 0.0f},
      covariance{{{0.0f, 0.0f, 0.0f},
                  {0.0f, 0.0f, 0.0f},
                  {0.0f, 0.0f, 0.0f}}},
      last_command{0.0f, 0.0f, false},
      pose_initialized(false)
{
}

void pose_estimator::begin()
{
    pose = {0.0f, 0.0f, 0.0f};
    covariance.m[0][0] = square(400.0f);
    covariance.m[0][1] = 0.0f;
    covariance.m[0][2] = 0.0f;
    covariance.m[1][0] = 0.0f;
    covariance.m[1][1] = square(400.0f);
    covariance.m[1][2] = 0.0f;
    covariance.m[2][0] = 0.0f;
    covariance.m[2][1] = 0.0f;
    covariance.m[2][2] = square(15.0f * robot_config::kDegToRad);
    last_command = {0.0f, 0.0f, false};
    pose_initialized = false;
}

void pose_estimator::resetPose(const Pose2D &new_pose,
                               float xy_std_mm,
                               float theta_std_rad)
{
    pose = new_pose;
    pose.theta_rad = wrapAngleRad(pose.theta_rad);

    covariance.m[0][0] = square(xy_std_mm);
    covariance.m[0][1] = 0.0f;
    covariance.m[0][2] = 0.0f;
    covariance.m[1][0] = 0.0f;
    covariance.m[1][1] = square(xy_std_mm);
    covariance.m[1][2] = 0.0f;
    covariance.m[2][0] = 0.0f;
    covariance.m[2][1] = 0.0f;
    covariance.m[2][2] = square(theta_std_rad);
    pose_initialized = true;
}

void pose_estimator::predict(const MotionCommand &cmd, float gyro_z_rad_s, float dt_s)
{
    if (dt_s <= 0.0f) {
        last_command = cmd;
        return;
    }

    const float theta = pose.theta_rad;
    const float sin_theta = sinf(theta);
    const float cos_theta = cosf(theta);

    const float dx = (sin_theta * cmd.vx_body_mm_s + cos_theta * cmd.vy_body_mm_s) * dt_s;
    const float dy = (-cos_theta * cmd.vx_body_mm_s + sin_theta * cmd.vy_body_mm_s) * dt_s;
    const float dtheta = gyro_z_rad_s * dt_s;

    pose.x_mm += dx;
    pose.y_mm += dy;
    pose.theta_rad = wrapAngleRad(pose.theta_rad + dtheta);

    const float dfdtheta_x = (cos_theta * cmd.vx_body_mm_s - sin_theta * cmd.vy_body_mm_s) * dt_s;
    const float dfdtheta_y = (sin_theta * cmd.vx_body_mm_s + cos_theta * cmd.vy_body_mm_s) * dt_s;

    float F[3][3] = {
        {1.0f, 0.0f, dfdtheta_x},
        {0.0f, 1.0f, dfdtheta_y},
        {0.0f, 0.0f, 1.0f},
    };

    float FP[3][3] = {{0.0f, 0.0f, 0.0f},
                      {0.0f, 0.0f, 0.0f},
                      {0.0f, 0.0f, 0.0f}};
    float predicted_cov[3][3] = {{0.0f, 0.0f, 0.0f},
                                 {0.0f, 0.0f, 0.0f},
                                 {0.0f, 0.0f, 0.0f}};

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 3; ++k) {
                FP[row][col] += F[row][k] * covariance.m[k][col];
            }
        }
    }

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 3; ++k) {
                predicted_cov[row][col] += FP[row][k] * F[col][k];
            }
        }
    }

    const float command_speed = sqrtf(square(cmd.vx_body_mm_s) + square(cmd.vy_body_mm_s));
    const float linear_std = robot_config::kLinearProcessStdMmPerSec + 0.25f * command_speed;
    const float theta_std = robot_config::kGyroProcessStdRadPerSec * (cmd.rotating ? 2.0f : 1.0f);

    predicted_cov[0][0] += square(linear_std * dt_s);
    predicted_cov[1][1] += square(linear_std * dt_s);
    predicted_cov[2][2] += square(theta_std * dt_s);

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            covariance.m[row][col] = predicted_cov[row][col];
        }
    }

    last_command = cmd;
}

void pose_estimator::correct(const SensorReadings &readings)
{
    if (!pose_initialized) {
        return;
    }

    float axis_theta = 0.0f;
    if (!nearestAxis(&axis_theta)) {
        return;
    }

    if (!last_command.rotating) {
        const float heading_H[3] = {0.0f, 0.0f, 1.0f};
        float innovation = wrapAngleRad(axis_theta - pose.theta_rad);
        applyScalarMeasurement(pose.theta_rad + innovation,
                               pose.theta_rad,
                               heading_H,
                               square(robot_config::kHeadingMeasurementStdRad),
                               robot_config::kInnovationGateRad,
                               true);
    }

    struct MeasurementSpec {
        float measurement;
        float min_value;
        float max_value;
        float variance;
        robot_config::SensorMount mount;
    };

    const MeasurementSpec specs[] = {
        {readings.front_mm, robot_config::kMedIRMinMm, robot_config::kMedIRMaxMm,
         square(robot_config::kFrontIRStdMm), robot_config::kFrontIRMount},
        {readings.rear_mm, robot_config::kLongIRMinMm, robot_config::kLongIRMaxMm,
         square(robot_config::kRearIRStdMm), robot_config::kRearIRMount},
        {readings.left_mm, robot_config::kLongIRMinMm, robot_config::kLongIRMaxMm,
         square(robot_config::kLeftIRStdMm), robot_config::kLeftIRMount},
        {readings.right_mm, robot_config::kMedIRMinMm, robot_config::kMedIRMaxMm,
         square(robot_config::kRightIRStdMm), robot_config::kRightIRMount},
        {readings.us_right_mm, robot_config::kUsMinMm, robot_config::kUsMaxMm,
         square(robot_config::kUltrasonicStdMm), robot_config::kRightUSMount},
    };

    for (const MeasurementSpec &spec : specs) {
        if (!inRange(spec.measurement, spec.min_value, spec.max_value)) {
            continue;
        }

        float predicted = 0.0f;
        if (!predictRangeForSensor(spec.mount.forward_offset_mm,
                                   spec.mount.left_offset_mm,
                                   spec.mount.yaw_offset_rad,
                                   &predicted)) {
            continue;
        }

        float jacobian[3] = {0.0f, 0.0f, 0.0f};
        numericalJacobian(spec.mount.forward_offset_mm,
                          spec.mount.left_offset_mm,
                          spec.mount.yaw_offset_rad,
                          jacobian);

        applyScalarMeasurement(spec.measurement,
                               predicted,
                               jacobian,
                               spec.variance,
                               robot_config::kInnovationGateMm);
    }
}

Pose2D pose_estimator::getPose() const
{
    return pose;
}

float pose_estimator::getHeadingDeg() const
{
    return wrapAngleDeg(pose.theta_rad * robot_config::kRadToDeg);
}

Covariance3x3 pose_estimator::getCovariance() const
{
    return covariance;
}

bool pose_estimator::isPoseInitialized() const
{
    return pose_initialized;
}

float pose_estimator::wrapAngleRad(float angle_rad)
{
    while (angle_rad > robot_config::kPi) {
        angle_rad -= 2.0f * robot_config::kPi;
    }
    while (angle_rad <= -robot_config::kPi) {
        angle_rad += 2.0f * robot_config::kPi;
    }
    return angle_rad;
}

float pose_estimator::wrapAngleDeg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

float pose_estimator::square(float value)
{
    return value * value;
}

bool pose_estimator::nearestAxis(float *axis_theta_rad) const
{
    float best_error = 1e9f;
    float best_axis = 0.0f;

    for (float axis : kAxisAngles) {
        const float error = absf(wrapAngleRad(pose.theta_rad - axis));
        if (error < best_error) {
            best_error = error;
            best_axis = axis;
        }
    }

    if (best_error > robot_config::kHeadingAlignmentTolRad) {
        return false;
    }

    if (axis_theta_rad != nullptr) {
        *axis_theta_rad = best_axis;
    }
    return true;
}

void pose_estimator::applyScalarMeasurement(float measurement,
                                            float predicted,
                                            const float H[3],
                                            float variance,
                                            float innovation_gate,
                                            bool wrap_innovation)
{
    float innovation = measurement - predicted;
    if (wrap_innovation) {
        innovation = wrapAngleRad(innovation);
    }

    if (absf(innovation) > innovation_gate) {
        return;
    }

    float PHt[3] = {0.0f, 0.0f, 0.0f};
    for (int row = 0; row < 3; ++row) {
        for (int k = 0; k < 3; ++k) {
            PHt[row] += covariance.m[row][k] * H[k];
        }
    }

    float S = variance;
    for (int k = 0; k < 3; ++k) {
        S += H[k] * PHt[k];
    }

    if (S <= 1e-6f) {
        return;
    }

    float K[3] = {
        PHt[0] / S,
        PHt[1] / S,
        PHt[2] / S,
    };

    pose.x_mm += K[0] * innovation;
    pose.y_mm += K[1] * innovation;
    pose.theta_rad = wrapAngleRad(pose.theta_rad + K[2] * innovation);

    float updated[3][3];
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            updated[row][col] = covariance.m[row][col] - K[row] * PHt[col];
        }
    }

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            covariance.m[row][col] = 0.5f * (updated[row][col] + updated[col][row]);
        }
    }
}

bool pose_estimator::predictRangeForSensor(float forward_offset_mm,
                                           float left_offset_mm,
                                           float yaw_offset_rad,
                                           float *predicted_mm) const
{
    const float sin_theta = sinf(pose.theta_rad);
    const float cos_theta = cosf(pose.theta_rad);

    const float sensor_x = pose.x_mm + sin_theta * forward_offset_mm + cos_theta * left_offset_mm;
    const float sensor_y = pose.y_mm - cos_theta * forward_offset_mm + sin_theta * left_offset_mm;

    const float ray_theta = pose.theta_rad + yaw_offset_rad;
    const float dir_x = sinf(ray_theta);
    const float dir_y = -cosf(ray_theta);

    float best_t = -1.0f;

    auto consider = [&](float t) {
        if (t <= 0.0f) {
            return;
        }
        const float hit_x = sensor_x + dir_x * t;
        const float hit_y = sensor_y + dir_y * t;
        if (hit_x < -1.0f || hit_x > robot_config::kArenaWidthMm + 1.0f) {
            return;
        }
        if (hit_y < -1.0f || hit_y > robot_config::kArenaHeightMm + 1.0f) {
            return;
        }
        if (best_t < 0.0f || t < best_t) {
            best_t = t;
        }
    };

    if (fabsf(dir_x) > 1e-5f) {
        consider((0.0f - sensor_x) / dir_x);
        consider((robot_config::kArenaWidthMm - sensor_x) / dir_x);
    }
    if (fabsf(dir_y) > 1e-5f) {
        consider((0.0f - sensor_y) / dir_y);
        consider((robot_config::kArenaHeightMm - sensor_y) / dir_y);
    }

    if (best_t <= 0.0f) {
        return false;
    }

    *predicted_mm = best_t;
    return true;
}

void pose_estimator::numericalJacobian(float forward_offset_mm,
                                       float left_offset_mm,
                                       float yaw_offset_rad,
                                       float jacobian[3]) const
{
    float predicted_center = 0.0f;
    if (!predictRangeForSensor(forward_offset_mm, left_offset_mm, yaw_offset_rad, &predicted_center)) {
        jacobian[0] = 0.0f;
        jacobian[1] = 0.0f;
        jacobian[2] = 0.0f;
        return;
    }

    pose_estimator perturbed = *this;

    perturbed.pose.x_mm += kFiniteDiffPos;
    float predicted_x = predicted_center;
    if (perturbed.predictRangeForSensor(forward_offset_mm, left_offset_mm, yaw_offset_rad, &predicted_x)) {
        jacobian[0] = (predicted_x - predicted_center) / kFiniteDiffPos;
    } else {
        jacobian[0] = 0.0f;
    }

    perturbed = *this;
    perturbed.pose.y_mm += kFiniteDiffPos;
    float predicted_y = predicted_center;
    if (perturbed.predictRangeForSensor(forward_offset_mm, left_offset_mm, yaw_offset_rad, &predicted_y)) {
        jacobian[1] = (predicted_y - predicted_center) / kFiniteDiffPos;
    } else {
        jacobian[1] = 0.0f;
    }

    perturbed = *this;
    perturbed.pose.theta_rad = wrapAngleRad(perturbed.pose.theta_rad + kFiniteDiffTheta);
    float predicted_theta = predicted_center;
    if (perturbed.predictRangeForSensor(forward_offset_mm, left_offset_mm, yaw_offset_rad, &predicted_theta)) {
        jacobian[2] = (predicted_theta - predicted_center) / kFiniteDiffTheta;
    } else {
        jacobian[2] = 0.0f;
    }
}
