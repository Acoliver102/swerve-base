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
        const val ID_TALONFX_F_L = 11
        const val ID_TALONFX_B_L = 13
        const val ID_TALONFX_F_R = 10
        const val ID_TALONFX_B_R = 12


        const val VOLTS = 10.0
        const val VOLT_SEC_PER_METER = 5.0
        const val VOLT_SEC_SQUARED_PER_METER = 2.0

        const val TRACK_WIDTH_METERS = 0.3429
        const val TRACK_LENGTH_METERS = 0.3429
        const val WHEEL_RADIUS_METERS = 0.5842

        val DRIVE_KINEMATICS = DifferentialDriveKinematics(TRACK_WIDTH_METERS)

        const val MAX_SPEED_METERS_PER_SEC = 10.0
        const val MAX_ACCEL_METERS_PER_SEC_SQUARED = 2.0

        const val RAMSETE_B = 10.0
        const val RAMSETE_ZETA = 10.0

        const val P_DRIVE_VEL = 10.0

        const val SWERVE_FORWARD_SPEED_MAX = 0.0 // meters/Sec
        const val SWERVE_STRAFE_SPEED_MAX = 0.0
        const val SWERVE_ROT_SPEED_MAX = 0.0 // rad/sec

    }
}
