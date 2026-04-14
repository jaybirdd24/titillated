#pragma once

namespace robot_config {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kDegToRad = kPi / 180.0f;

// Rectangular arena dimensions; tune on hardware.
constexpr float kArenaWidthMm = 1217.0f;
constexpr float kArenaHeightMm = 1991.0f;
constexpr float kLaneStepMm = 150.0f;

// Sensor mounts in the robot body frame:
//   +forward points toward the front wall when theta = 0
//   +left points toward world +x.
struct SensorMount {
    float forward_offset_mm;
    float left_offset_mm;
    float yaw_offset_rad;
};

constexpr SensorMount kFrontIRMount = {70.0f, 0.0f, 0.0f};
constexpr SensorMount kRearIRMount = {-70.0f, 0.0f, kPi};
constexpr SensorMount kLeftIRMount = {0.0f, 65.0f, 0.5f * kPi};
constexpr SensorMount kRightIRMount = {0.0f, -65.0f, -0.5f * kPi};
constexpr SensorMount kRightUSMount = {0.0f, -75.0f, -0.5f * kPi};

// Calibrated translational speeds at command magnitude 1000.
constexpr float kForwardMmPerSecAtFull = 700.0f;
constexpr float kBackwardMmPerSecAtFull = 700.0f;
constexpr float kLeftMmPerSecAtFull = 900.0f;
constexpr float kRightMmPerSecAtFull = 900.0f;

// EKF tuning.
constexpr float kHeadingAlignmentTolRad = 10.0f * kDegToRad;
constexpr float kHeadingMeasurementStdRad = 2.0f * kDegToRad;
constexpr float kGyroProcessStdRadPerSec = 3.0f * kDegToRad;
constexpr float kLinearProcessStdMmPerSec = 120.0f;
constexpr float kInnovationGateMm = 250.0f;
constexpr float kInnovationGateRad = 15.0f * kDegToRad;

// Measurement noise.
constexpr float kFrontIRStdMm = 18.0f;
constexpr float kRearIRStdMm = 30.0f;
constexpr float kLeftIRStdMm = 35.0f;
constexpr float kRightIRStdMm = 18.0f;
constexpr float kUltrasonicStdMm = 25.0f;

// Sensor validity windows.
constexpr float kMedIRMinMm = 40.0f;
constexpr float kMedIRMaxMm = 300.0f;
constexpr float kLongIRMinMm = 100.0f;
constexpr float kLongIRMaxMm = 800.0f;
constexpr float kUsMinMm = 20.0f;
constexpr float kUsMaxMm = 3000.0f;

}  // namespace robot_config
