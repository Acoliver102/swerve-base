# swerve-base
It's a base for a basic swerve drive.


    /** SWERVE DRIVE EXPLANATION:
     *
     *              ^
     *              \ FORWARD
     *  ____________\______________
     *  |                          |
     *  |                          |
     *  |                          |
     *  |             ___          |
     *  |           /    \         |  STRAFING
     *  |          V  X  |         | --->
     *  |          \ ___/          |
     *  |         ROTATION         |
     *  |                          |
     *  |                          |
     *  |__________________________|
     *
     *  The X AXIS of the LEFT joystick controls strafing
     *  The Y AXIS of the LEFT joystick controls forward motion
     *  The X AXIS of the RIGHT joystick controls rotation
     *
     *
     *
     *  Command: forward, strafe are percentages
     *           rotation is counterclockwise
     *
     *  Axis motors are configured to turn counterclockwise since we install them upside down in 2021 FRC competition build
     */
     
     #Goals
     - add implementation to allow FRC Simulation to simulate motors which needs a custom physics simulation
     --Initialize stationary robot and set weight, acceleration, velocity_X,velocity_Y and angular velocity to zero and robot orientation
     --- update in 0.1 second ( cycle Period) steps with nonslipping model summing wheel forces at center of mass/rotation
     --deltaOmega ~ (angular velocity * timeStep) = 
     --deltaVelocityX
Standard Java var names     
vx: The velocity of the robot in the x (forward) direction.
vy: The velocity of the robot in the y (sideways) direction. (Positive values mean the robot is moving to the left).
omega: The angular velocity of the robot in radians per second.
Pose2d: 

    #odometry
     ---assume gyro sensor updates proportional to ideal initialOrientation + deltaOmega
     --assume accelerometer updates proportional to ideal dv/dt
     d
