package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.TalonFXInvertType
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.Chassis.DRIVING_RATIO
import frc.robot.Constants.Chassis.SWERVE_FORWARD_SPEED_MAX
import frc.robot.Constants.Chassis.SWERVE_ROT_SPEED_MAX
import frc.robot.Constants.Chassis.SWERVE_STRAFE_SPEED_MAX
import frc.robot.Constants.Chassis.TRACK_LENGTH_METERS
import frc.robot.Constants.Chassis.TRACK_WIDTH_METERS
import frc.robot.Constants.Chassis.WHEEL_RADIUS_METERS
import frc.robot.commands.chassis.ChassisRunJoystick
import frc.robot.fusion.motion.*
import kotlin.math.*

object Chassis : SubsystemBase() { // Start by defining motors
    // Motor Controllers
    private val talonFXFrontLeft = FTalonFX(MotorID(Constants.Chassis.ID_TALONFX_F_L, "talonFXFrontLeft", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.Clockwise)
        configNeutralDeadband(0.05)
        selectedSensorPosition = 0
    }
    private val talonFXBackLeft = FTalonFX(MotorID(Constants.Chassis.ID_TALONFX_B_L, "talonFXBackLeft", MotorModel.TalonFX)).apply {
        setInverted(TalonFXInvertType.FollowMaster)
        control(ControlMode.Follower, FollowerConfig(talonFXFrontLeft))
        configNeutralDeadband(0.05)
    }
    private val talonFXFrontRight = FTalonFX(MotorID(Constants.Chassis.ID_TALONFX_F_R, "talonFXFrontRight", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.CounterClockwise)
        configNeutralDeadband(0.05)
        selectedSensorPosition = 0
    }
    private val talonFXBackRight = FTalonFX(MotorID(Constants.Chassis.ID_TALONFX_B_R, "talonFXBackRight", MotorModel.TalonFX)).apply {
        setInverted(TalonFXInvertType.FollowMaster)
        control(ControlMode.Follower, FollowerConfig(talonFXFrontRight))
        configNeutralDeadband(0.05)
    }
    private val axisControllerFrontLeft = FTalonFX(MotorID(Constants.Chassis.ID_AXIS_F_L, "axisFrontLeft", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.Clockwise)
        configNeutralDeadband(0.05)
        selectedSensorPosition = 0
    }
    private val axisControllerBackLeft = FTalonFX(MotorID(Constants.Chassis.ID_AXIS_B_L, "axisBackLeft", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.Clockwise)
        configNeutralDeadband(0.05)
        selectedSensorPosition = 0
    }
    private val axisControllerFrontRight = FTalonFX(MotorID(Constants.Chassis.ID_AXIS_F_R, "axisFrontRight", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.Clockwise)
        configNeutralDeadband(0.05)
        selectedSensorPosition = 0
    }
    private val axisControllerBackRight = FTalonFX(MotorID(Constants.Chassis.ID_AXIS_B_R, "axisBackRight", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.Clockwise)
        configNeutralDeadband(0.05)
        selectedSensorPosition = 0
    }


    // wheel position sensor sets
    val leftPosition: Double get() = talonFXFrontLeft.selectedSensorPosition / 4096 * 2 * PI * Constants.Chassis.WHEEL_RADIUS_METERS
    val rightPosition: Double get() = talonFXFrontRight.selectedSensorPosition / 4096 * 2 * PI * Constants.Chassis.WHEEL_RADIUS_METERS
    val wheelSpeeds: DifferentialDriveWheelSpeeds
        get() = DifferentialDriveWheelSpeeds(
            talonFXFrontLeft.selectedSensorVelocity.toDouble() / 4096 * 2 * PI * Constants.Chassis.WHEEL_RADIUS_METERS * 10,
            talonFXFrontRight.selectedSensorVelocity.toDouble() / 4096 * 2 * PI * Constants.Chassis.WHEEL_RADIUS_METERS * 10
        )




    var generalMotionCharacteristics = MotionCharacteristics(ControlMode.DutyCycle, dutyCycleConfig = DutyCycleConfig(0.5))



    init {
        defaultCommand = ChassisRunJoystick() // Set default state to run with joystick

        Shuffleboard.getTab("Chassis").add(talonFXFrontRight)
        Shuffleboard.getTab("Chassis").add(talonFXFrontLeft)
        Shuffleboard.getTab("Chassis").add(talonFXBackRight)
        Shuffleboard.getTab("Chassis").add(talonFXBackLeft)

        Shuffleboard.getTab("Chassis").add(this)
    }

    override fun periodic() {

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
     *  Axis motors are configured to turn counterclockwise (we install them upside down)
     */

    // TODO : Implement efficient axis turning (limit to 180 degrees of motion)
    // TODO : Set Safe Max speed for testing

    fun swerveDriveMath(forward_input:Double, strafe_input:Double, rot_input: Double): List<Double> {

        var forward_speed = forward_input*SWERVE_FORWARD_SPEED_MAX
        var strafe_speed = strafe_input*SWERVE_STRAFE_SPEED_MAX
        var rot_speed =  rot_input* SWERVE_ROT_SPEED_MAX

        val alpha = atan(TRACK_LENGTH_METERS / TRACK_WIDTH_METERS)/2
        val distance_to_wheel = sqrt((TRACK_LENGTH_METERS).pow(2) + TRACK_WIDTH_METERS.pow(2))/2
        var rotation_added_speed = rot_speed*distance_to_wheel

        var angleFrontLeft = atan((rotation_added_speed* sin(alpha) - strafe_speed)/(forward_speed -
                rotation_added_speed* cos(alpha)))
        var speedFrontLeft = ((forward_speed - rotation_added_speed* cos(alpha)).pow(2) + (rotation_added_speed* sin(alpha)
                - strafe_speed).pow(2)).pow(1/2)

        var angleBackLeft = atan((-rotation_added_speed* sin(alpha) - strafe_speed)/(forward_speed -
                rotation_added_speed* cos(alpha)))
        var speedBackLeft = ((forward_speed - rotation_added_speed* cos(alpha)).pow(2) + (-rotation_added_speed* sin(alpha)
                - strafe_speed).pow(2)).pow(1/2)

        var angleFrontRight = atan((rotation_added_speed*sin(alpha) - strafe_speed)/(rotation_added_speed*cos(alpha)
                + forward_speed))
        var speedFrontRight = ((rotation_added_speed*sin(alpha) - strafe_speed).pow(2) + (rotation_added_speed*cos(alpha)
                + forward_speed).pow(2)).pow(1/2)

        var angleBackRight = atan((-rotation_added_speed*sin(alpha) - strafe_speed)/(rotation_added_speed*cos(alpha)
                + forward_speed))
        var speedBackRight = ((-rotation_added_speed*sin(alpha) - strafe_speed).pow(2) + (rotation_added_speed*cos(alpha)
                + forward_speed).pow(2)).pow(1/2)

        // TODO : Convert m/s -> wheel speeds, set up PID for axis motors

        var speeds = mutableListOf<Double>(speedFrontLeft, speedBackLeft, speedFrontRight, speedBackRight)
        var angles = mutableListOf<Double>(angleFrontLeft, angleBackLeft, angleFrontRight, angleBackRight)

        // Conversions

        speeds.replaceAll { s -> s/WHEEL_RADIUS_METERS } // to rad/s
        speeds.replaceAll { s -> 2048*s/(2*PI) } // to ticks/s
        speeds.replaceAll { s -> s*DRIVING_RATIO } // account for driving ratio

        angles.replaceAll { a -> 2048*a/(2*PI) } // to ticks

        var outputs = listOf(angleFrontLeft, speedFrontLeft, angleBackLeft, speedBackLeft, angleFrontRight, speedFrontRight
        , angleBackRight, speedBackRight)

        return outputs

    }

    fun runSwerveJoystick(lStickYAxis: Double, lStickXAxis: Double, rStickXAxis: Double) {
        val settings = Chassis.swerveDriveMath(lStickYAxis, lStickXAxis, rStickXAxis)

        var angleFL = settings.get(0)
        axisControllerFrontLeft.control(ControlMode.Position, PositionConfig(angleFL.toInt()))
        var speedFL = settings.get(1)
        talonFXFrontLeft.control(ControlMode.Velocity, VelocityConfig(speedFL.toInt()))
        var angleBL = settings.get(2)
        axisControllerFrontLeft.control(ControlMode.Position, PositionConfig(angleBL.toInt()))
        var speedBL = settings.get(3)
        talonFXFrontLeft.control(ControlMode.Velocity, VelocityConfig(speedBL.toInt()))
        var angleFR = settings.get(4)
        axisControllerFrontLeft.control(ControlMode.Position, PositionConfig(angleFR.toInt()))
        var speedFR = settings.get(5)
        talonFXFrontLeft.control(ControlMode.Velocity, VelocityConfig(speedFR.toInt()))
        var angleBR = settings.get(6)
        axisControllerFrontLeft.control(ControlMode.Position, PositionConfig(angleBR.toInt()))
        var speedBR = settings.get(7)
        talonFXFrontLeft.control(ControlMode.Velocity, VelocityConfig(speedBR.toInt()))


    }

}