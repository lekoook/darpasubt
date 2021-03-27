#ifndef H_TWIST_SOLVER
#define H_TWIST_SOLVER

/**
 * @brief Angular velocities for left and right wheel.
 * 
 */
struct WheelAngular
{
    /**
     * @brief Left wheel angular velocity.
     * 
     */
    double left = 0;

    /**
     * @brief Right wheel angular velocity.
     * 
     */
    double right = 0;
};

/**
 * @brief Contains the configuration values for various aspect of the robot body and wheel.
 * 
 */
struct BodyParams
{
    /**
     * @brief Distance between the left and right wheel's ground contact point.
     * 
     */
    double horizontalWheelDistance = 0;

    /**
     * @brief Half of the distance between the left and right wheel's ground contact point.
     * 
     */
    double bodyWheelRadius = 0;

    /**
     * @brief Radius of each wheel.
     * 
     */
    double wheelRadius = 0;

    /**
     * @brief Construct a new Body Params object.
     * 
     */
    BodyParams() {}

    /**
     * @brief Construct a new Body Params object.
     * 
     * @param horizontalWheelDistance Distance between the left and right wheel's ground contact point.
     * @param wheelRadius Radius of each wheel.
     */
    BodyParams(double horizontalWheelDistance, double wheelRadius) 
        : horizontalWheelDistance(horizontalWheelDistance), wheelRadius(wheelRadius)
    {
        bodyWheelRadius = horizontalWheelDistance / 2.0f;
    }

    /**
     * @brief Sets the configuration values. Should use this method to set instead of accessing directly.
     * 
     * @param horizontalWheelDistance Distance between the left and right wheel's ground contact point.
     * @param wheelRadius Radius of each wheel.
     */
    void set(double horizontalWheelDistance, double wheelRadius)
    {
        this->horizontalWheelDistance = horizontalWheelDistance;
        this->wheelRadius = wheelRadius;
        this->bodyWheelRadius = horizontalWheelDistance / 2.0f;
    }
};

/**
 * @brief This is a simple class to solve the linear velocity along the X-axis and angular velocity about the Z-axis of
 * a body into individual wheel's angular velocity.
 * 
 */
class TwistSolver
{
private:
    /**
     * @brief Configuration parameters of the robot body.
     * 
     */
    const BodyParams body;

public:
    /**
     * @brief Construct a new Twist Solver object.
     * 
     * @param bodyParameters BodyParams object that contains the configuration dimensions of the robot body.
     */
    TwistSolver(BodyParams bodyParameters);

    /**
     * @brief Solves the specified linear and angular velocity into left and right wheels' angular velocity.
     * 
     * @param linearX Linear velocity along X-axis (forward/backward).
     * @param angularZ Angular velocity about Z-axis (yaw).
     * @param wheel Contains the computed wheels' angular velocity values.
     * @return true If the turn radius (for arc turns) is < than half the distance between left and right wheels.
     * @return false If the turn radius (for arc turns) is >= than half the distance between left and right wheels.
     */
    bool solve(double linearX, double angularZ, WheelAngular* wheel);
};

#endif // H_TWIST_SOLVER