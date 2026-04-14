#include <assert.h>
#include <math.h>

#include "pose_estimator.h"

namespace {

float absf(float value)
{
    return value < 0.0f ? -value : value;
}

}  // namespace

int main()
{
    pose_estimator estimator;
    estimator.begin();
    estimator.resetPose({200.0f, 400.0f, 0.0f}, 50.0f, 5.0f * robot_config::kDegToRad);

    const Covariance3x3 before_predict = estimator.getCovariance();
    estimator.predict({200.0f, 0.0f, false}, 0.0f, 1.0f);
    const Pose2D after_predict = estimator.getPose();
    const Covariance3x3 after_predict_cov = estimator.getCovariance();

    assert(absf(after_predict.x_mm - 200.0f) < 1.0f);
    assert(absf(after_predict.y_mm - 200.0f) < 1.0f);
    assert(after_predict_cov.m[1][1] > before_predict.m[1][1]);

    estimator.resetPose({300.0f, 250.0f, 0.0f}, 120.0f, 5.0f * robot_config::kDegToRad);
    const Covariance3x3 before_correct_cov = estimator.getCovariance();
    const SensorReadings wall_fix = {
        80.0f,   // front
        0.0f,    // rear invalid
        0.0f,    // left invalid
        135.0f,  // right
        125.0f,  // ultrasonic right
        0.0f
    };
    estimator.correct(wall_fix);
    const Pose2D after_correct = estimator.getPose();
    const Covariance3x3 after_correct_cov = estimator.getCovariance();

    assert(after_correct.x_mm < 300.0f);
    assert(after_correct.y_mm < 250.0f);
    assert(after_correct_cov.m[0][0] < before_correct_cov.m[0][0]);
    assert(after_correct_cov.m[1][1] < before_correct_cov.m[1][1]);

    estimator.resetPose({300.0f, 250.0f, 0.0f}, 120.0f, 5.0f * robot_config::kDegToRad);
    const Pose2D before_reject = estimator.getPose();
    estimator.correct({
        900.0f,  // invalid front
        0.0f,    // invalid rear
        900.0f,  // invalid left
        900.0f,  // invalid right
        0.0f,    // invalid us
        0.0f
    });
    const Pose2D after_reject = estimator.getPose();
    assert(absf(after_reject.x_mm - before_reject.x_mm) < 0.01f);
    assert(absf(after_reject.y_mm - before_reject.y_mm) < 0.01f);

    estimator.resetPose({300.0f, 600.0f, 179.0f * robot_config::kDegToRad},
                        50.0f,
                        5.0f * robot_config::kDegToRad);
    estimator.predict({0.0f, 0.0f, true}, 4.0f * robot_config::kDegToRad, 1.0f);
    const float wrapped_heading = estimator.getHeadingDeg();
    assert(wrapped_heading < -170.0f);
    estimator.correct({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    const float corrected_heading = estimator.getHeadingDeg();
    assert(absf(absf(corrected_heading) - 180.0f) < 10.0f);

    return 0;
}
