# swerve-base
It's a base for a basic swerve drive.

Verify build.gradle for dependancies in plguins and libraries

    id "java"
    id "edu.wpi.first.GradleRIO" version "2020.3.2"
    id "org.jetbrains.kotlin.jvm" version "1.3.50"
    id "org.jlleitschuh.gradle.ktlint" version "9.2.1"

dependencies {
    implementation "org.jetbrains.kotlin:kotlin-stdlib"
    implementation 'io.github.microutils:kotlin-logging:1.7.8'
    implementation 'org.slf4j:slf4j-simple:1.7.26'

    implementation wpi.deps.wpilib()
    nativeZip wpi.deps.wpilibJni(wpi.platforms.roborio)
    nativeDesktopZip wpi.deps.wpilibJni(wpi.platforms.desktop)

    implementation wpi.deps.vendor.java()
    nativeZip wpi.deps.vendor.jni(wpi.platforms.roborio)
    nativeDesktopZip wpi.deps.vendor.jni(wpi.platforms.desktop)

    testImplementation 'junit:junit:4.12'

    // Enable simulation gui support. Must check the box in vscode to enable support
    // upon debugging
    simulation wpi.deps.sim.gui(wpi.platforms.desktop, false)
}

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
     
     #Goals  make a class SwervelDrivetrainSim
     - add implementation to allow FRC Simulation to simulate motors which needs a custom physics simulation
     --Initialize stationary robot and set weight, acceleration, velocity_X,velocity_Y and angular velocity to zero and robot orientation
     --- update in 0.02 second ( cycle Period) steps with nonslipping model summing wheel forces at center of mass/rotation
     --deltaOmega ~ (angular velocity * timeStep) = 
     
     using LinearSystemSim class. Subclasses override the UpdateX(x, u, dt)
     --deltaVelocityX
Standard Java var names     
vx: The velocity of the robot in the x (forward) direction.
vy: The velocity of the robot in the y (sideways) direction. (Positive values mean the robot is moving to the left).
omega: The angular velocity of the robot in radians per second.

use inpout to motor control  as idea
        var forward_speed = forward_input*SWERVE_FORWARD_SPEED_MAX
        var strafe_speed = strafe_input*SWERVE_STRAFE_SPEED_MAX
        var rot_speed =  rot_input* SWERVE_ROT_SPEED_MAX
vx + d_strafe_speed/dt
vy + d_forward_speed/dt
omega + d_rot_speed/dt

Pose2d: 

inc onstants.kt
WHEEL_RADIUS_METERS:wheel radius:
TRACK_WIDTH_METERS = 0.3429
TRACK_LENGTH_METERS = 0.3429

need to measure:
moment of inertia of the drivetrain est. 3-8 kgm^2

robot mass:
decision on measurement noise: initial set to null

    #odometry
     ---assume gyro sensor updates proportional to ideal initialOrientation + deltaOmega
     --assume accelerometer updates proportional to ideal dv/dt
     d
