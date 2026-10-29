#ifndef KINEMATICS_H
#define KINEMATICS_H

// Solve a 2-link planar leg. Given a target point (x, y) in the leg's
// own frame (hip at origin) and the two link lengths, compute the hip
// and knee angles in degrees.
//
// Returns false if the point is unreachable (too far or too close),
// in which case the output angles are left untouched.
bool solveLeg2D(float x, float y, float L1, float L2,
                float& hipAngle, float& kneeAngle);

#endif