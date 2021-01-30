/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RamseteCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.autonomous.AutonomousSad
import frc.robot.commands.chassis.ChassisRunBasic
import frc.robot.commands.chassis.EncoderReset
import frc.robot.commands.test.TestCommand


import frc.robot.subsystems.Chassis
import java.io.IOException
import java.nio.file.Path
import javax.naming.ldap.Control
import javax.sql.XAConnectionBuilder


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private val mChassis = Chassis



    private var mAutoCommandChooser: SendableChooser<Command> = SendableChooser()

    val mAutonomousSad = AutonomousSad()


    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    init {
        // Configure the button bindings
        configureButtonBindings()
        mAutoCommandChooser.setDefaultOption("Autonomous Sad", mAutonomousSad)
        SmartDashboard.putData("Auto mode", mAutoCommandChooser)


    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    fun configureButtonBindings() {
        JoystickButton(Controls.controller, XboxController.Button.kB.value)
                .whenPressed(TestCommand())
        JoystickButton(Controls.controller, XboxController.Button.kBumperRight.value)
                .whenPressed(ChassisRunBasic())
        JoystickButton(Controls.controller, XboxController.Button.kA.value)
                .whenPressed(EncoderReset())
    }

//    fun getAutonomousCommand(): Command {
//       var trajectoryJSON = "paths/Unnamed_0.wpilib.json";
//        var trajectory = Trajectory(trajectoryJSON)
//        try {
//            var trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//            var trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//        }   catch (ex: IOException) {
//        DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
//      }
//    }


}



