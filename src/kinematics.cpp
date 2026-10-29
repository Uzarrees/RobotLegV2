#include "kinematics.h"
#include <math.h>

bool solveLeg2D(float x, float y, float L1, float L2,
                float& hipAngle, float& kneeAngle) {
    // L1 = first link  (your a2, the thigh)
    // L2 = second link (your a4, the shin)

    // distance from the hip origin to the target point
    float r = sqrtf(x * x + y * y);

    // --- reachability guard ---
    // too far to reach, or folded closer than the links allow
    if (r > (L1 + L2) || r < fabsf(L1 - L2)) {
        return false;
    }

    // --- knee angle (law of cosines, interior angle between the links) ---
    float cosKnee = (r * r - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    if (cosKnee > 1.0f)  cosKnee = 1.0f;      // clamp rounding overshoot
    if (cosKnee < -1.0f) cosKnee = -1.0f;
    float knee = acosf(cosKnee);

    // --- hip angle ---
    // direction straight to the target...
    float toTarget = atan2f(y, x);
    // ...minus the correction from how far the thigh leads the bend
    float cosCorr = (r * r + L1 * L1 - L2 * L2) / (2.0f * L1 * r);
    if (cosCorr > 1.0f)  cosCorr = 1.0f;
    if (cosCorr < -1.0f) cosCorr = -1.0f;
    float hip = toTarget - acosf(cosCorr);

    // --- radians -> degrees for the joint layer ---
    const float RAD_TO_DEG = 180.0f / 3.14159265358979f;
    hipAngle  = hip  * RAD_TO_DEG;
    kneeAngle = knee * RAD_TO_DEG;
    return true;
}
