#include <math.h>
#include "TwistSolver.hpp"

TwistSolver::TwistSolver(BodyParams bodyParameters) : body(bodyParameters) {}

bool TwistSolver::solve(double linearX, double angularZ, WheelAngular* wheel)
{
    if (linearX != 0 && angularZ != 0 && (fabs(linearX / angularZ) < body.bodyWheelRadius))
    {
        return false;
    }

    double angDelta = angularZ * body.bodyWheelRadius / body.wheelRadius;
    double bodyAng = linearX / body.wheelRadius;
    wheel->left = bodyAng - angDelta;
    wheel->right = bodyAng + angDelta;

    return true;
}