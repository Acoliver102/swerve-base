/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
object Constants {
    const val SCHEDULER_RATE = 20 // ms

    object Inputs {
        const val ID_CONTROLLER = 0
    }

    object Chassis {
        const val ID_TALONFX_F_L = 3
        const val ID_TALONFX_B_L = 2
        const val ID_TALONFX_F_R = 7
        const val ID_TALONFX_B_R = 0

        const val ID_AXIS_F_L = 4
        const val ID_AXIS_B_L = 6
        const val ID_AXIS_F_R = 1
        const val ID_AXIS_B_R = 5

        const val DRIVE_kF = 0.0
        const val DRIVE_kP = 0.1
        const val DRIVE_kI = 0.0
        const val DRIVE_kD = 0.0

        const val AXIS_kF = 0.0
        const val AXIS_kP = 0.2
        const val AXIS_kI = 0.0
        const val AXIS_kD = 0.0

        const val STEERING_RATIO = 12.8
        const val DRIVING_RATIO = 6.86

        const val TRACK_WIDTH_METERS = 0.3429
        const val TRACK_LENGTH_METERS = 0.3429
        const val WHEEL_RADIUS_METERS = 0.0508


        const val SWERVE_FORWARD_SPEED_MAX = 0.5 // meters/Sec
        const val SWERVE_STRAFE_SPEED_MAX = 0.5
        const val SWERVE_ROT_SPEED_MAX = 1.0 // rad/sec

    }
}
