package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.TalonFXInvertType
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.Constants.Chassis.AXIS_kD
import frc.robot.Constants.Chassis.AXIS_kF
import frc.robot.Constants.Chassis.AXIS_kI
import frc.robot.Constants.Chassis.AXIS_kP
import frc.robot.Constants.Chassis.DRIVE_kD
import frc.robot.Constants.Chassis.DRIVE_kF
import frc.robot.Constants.Chassis.DRIVE_kI
import frc.robot.Constants.Chassis.DRIVE_kP
import frc.robot.Constants.Chassis.DRIVING_RATIO
import frc.robot.Constants.Chassis.STEERING_RATIO
import frc.robot.Constants.Chassis.SWERVE_FORWARD_SPEED_MAX
import frc.robot.Constants.Chassis.SWERVE_ROT_SPEED_MAX
import frc.robot.Constants.Chassis.SWERVE_STRAFE_SPEED_MAX
import frc.robot.Constants.Chassis.TRACK_LENGTH_METERS
import frc.robot.Constants.Chassis.TRACK_WIDTH_METERS
import frc.robot.Constants.Chassis.WHEEL_RADIUS_METERS
import frc.robot.commands.chassis.ChassisRunSwerve
import frc.robot.fusion.motion.*
import mu.KotlinLogging
import kotlin.math.*

object Chassis : SubsystemBase() { // Start by defining motors
    // Motor Controllers
    private val talonFXFrontLeft = FTalonFX(MotorID(Constants.Chassis.ID_TALONFX_F_L, "talonFXFrontLeft", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.CounterClockwise)
        configNeutralDeadband(0.01)
        selectedSensorPosition = 0
        control(
                DutyCycleConfig(0.1)
        )
        configPID(DRIVE_kF, DRIVE_kP, DRIVE_kI, DRIVE_kD)
    }
    private val talonFXBackLeft = FTalonFX(MotorID(Constants.Chassis.ID_TALONFX_B_L, "talonFXBackLeft", MotorModel.TalonFX)).apply {
        setInverted(TalonFXInvertType.CounterClockwise)
        configNeutralDeadband(0.01)
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        selectedSensorPosition = 0
        control(
                DutyCycleConfig(0.1)
        )
        configPID(DRIVE_kF, DRIVE_kP, DRIVE_kI, DRIVE_kD)
    }
    private val talonFXFrontRight = FTalonFX(MotorID(Constants.Chassis.ID_TALONFX_F_R, "talonFXFrontRight", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.CounterClockwise)
        configNeutralDeadband(0.01)
        selectedSensorPosition = 0
        control(
                DutyCycleConfig(0.1)
        )
        configPID(DRIVE_kF, DRIVE_kP, DRIVE_kI, DRIVE_kD)
    }
    private val talonFXBackRight = FTalonFX(MotorID(Constants.Chassis.ID_TALONFX_B_R, "talonFXBackRight", MotorModel.TalonFX)).apply {
        setInverted(TalonFXInvertType.CounterClockwise)
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        configNeutralDeadband(0.01)
        selectedSensorPosition = 0
        control(
                DutyCycleConfig(0.1)
        )
        configPID(DRIVE_kF, DRIVE_kP, DRIVE_kI, DRIVE_kD)
    }
    private val axisControllerFrontLeft = FTalonFX(MotorID(Constants.Chassis.ID_AXIS_F_L, "axisFrontLeft", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.Clockwise)
        configNeutralDeadband(0.05)
        selectedSensorPosition = 0
        configPID(AXIS_kF, AXIS_kP, AXIS_kI, AXIS_kD)
    }
    private val axisControllerBackLeft = FTalonFX(MotorID(Constants.Chassis.ID_AXIS_B_L, "axisBackLeft", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.Clockwise)
        configNeutralDeadband(0.05)
        selectedSensorPosition = 0
        configPID(AXIS_kF, AXIS_kP, AXIS_kI, AXIS_kD)
    }
    private val axisControllerFrontRight = FTalonFX(MotorID(Constants.Chassis.ID_AXIS_F_R, "axisFrontRight", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.Clockwise)
        configNeutralDeadband(0.05)
        selectedSensorPosition = 0
        configPID(AXIS_kF, AXIS_kP, AXIS_kI, AXIS_kD)
    }
    private val axisControllerBackRight = FTalonFX(MotorID(Constants.Chassis.ID_AXIS_B_R, "axisBackRight", MotorModel.TalonFX)).apply {
        configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        setInverted(TalonFXInvertType.Clockwise)
        configNeutralDeadband(0.05)
        selectedSensorPosition = 0
        configPID(AXIS_kF, AXIS_kP, AXIS_kI, AXIS_kD)
    }

    private var prevAngleFL = 0.0
    private var prevAngleBL = 0.0
    private var prevAngleFR = 0.0
    private var prevAngleBR = 0.0

    private var switch_status = false


    // wheel position sensor sets
//    val leftPosition: Double get() = talonFXFrontLeft.selectedSensorPosition / 4096 * 2 * PI * Constants.Chassis.WHEEL_RADIUS_METERS
//    val rightPosition: Double get() = talonFXFrontRight.selectedSensorPosition / 4096 * 2 * PI * Constants.Chassis.WHEEL_RADIUS_METERS
//    val wheelSpeeds: DifferentialDriveWheelSpeeds
//        get() = DifferentialDriveWheelSpeeds(
//            talonFXFrontLeft.selectedSensorVelocity.toDouble() / 4096 * 2 * PI * Constants.Chassis.WHEEL_RADIUS_METERS * 10,
//            talonFXFrontRight.selectedSensorVelocity.toDouble() / 4096 * 2 * PI * Constants.Chassis.WHEEL_RADIUS_METERS * 10
//        )

    private val ahrs = AHRS(SPI.Port.kMXP).apply {
        calibrate() // motion sensor
    }

    val heading: Double get() = ahrs.angle.IEEErem(360.0)

    fun resetHeading() {
        ahrs.reset()
    }

    fun resetMotorEncoder() {
        axisControllerFrontLeft.selectedSensorPosition = 0
    }



    var generalMotionCharacteristics = MotionCharacteristics(ControlMode.DutyCycle, dutyCycleConfig = DutyCycleConfig(0.5))



    init {
        defaultCommand = ChassisRunSwerve() // Set default state to run with joystick

//        Shuffleboard.getTab("Chassis").add(talonFXFrontRight)
//        Shuffleboard.getTab("Chassis").add(talonFXFrontLeft)
//        Shuffleboard.getTab("Chassis").add(talonFXBackRight)
//        Shuffleboard.getTab("Chassis").add(talonFXBackLeft)
//
//        Shuffleboard.getTab("Chassis").add(this)
    }

    override fun periodic() {

    }

    fun chassisBasic() {
        talonFXFrontLeft.workaroundRunVelocity(266252.0)
    }

    fun reverseCheck(angle: Double, encodeVal: Double): Boolean {
        var mAngle = angle
        var mEncodeValue = encodeVal

        return(abs(mAngle - mEncodeValue) < abs(PI + mAngle - mEncodeValue))


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

        var encAngleFL = axisControllerFrontLeft.workaroundGetPosition()/STEERING_RATIO/2048*(2*PI)
        var encAngleBL = axisControllerBackLeft.workaroundGetPosition()/STEERING_RATIO/2048*(2*PI)
        var encAngleFR = axisControllerFrontRight.workaroundGetPosition()/STEERING_RATIO/2048*(2*PI)
        var encAngleBR = axisControllerBackRight.workaroundGetPosition()/STEERING_RATIO/2048*(2*PI)




        val alpha = atan(TRACK_LENGTH_METERS / TRACK_WIDTH_METERS)
        var coeff = alpha/PI
//        KotlinLogging.logger("Angle").info {coeff}
        val distance_to_wheel = sqrt((TRACK_LENGTH_METERS).pow(2) + TRACK_WIDTH_METERS.pow(2))/2
        var rotation_added_speed = rot_speed*distance_to_wheel



        var forwardSpeedFL = forward_speed - rotation_added_speed*cos(alpha)
        var strafeSpeedFL = strafe_speed - rotation_added_speed*sin(alpha)
        var speedFrontLeft = sqrt(forwardSpeedFL.pow(2) + strafeSpeedFL.pow(2))
        var angleFrontLeft = atan2(strafeSpeedFL, forwardSpeedFL)

        var forwardSpeedBL = forward_speed - rotation_added_speed*sin(alpha)
        var strafeSpeedBL = strafe_speed + rotation_added_speed*cos(alpha)
        var speedBackLeft = sqrt(forwardSpeedBL.pow(2) + strafeSpeedBL.pow(2))
        var angleBackLeft = atan2(strafeSpeedBL, forwardSpeedBL)

        var forwardSpeedFR = forward_speed + rotation_added_speed*sin(alpha)
        var strafeSpeedFR = strafe_speed - rotation_added_speed*cos(alpha)
        var speedFrontRight = sqrt(forwardSpeedFR.pow(2) + strafeSpeedFR.pow(2))
        var angleFrontRight = atan2(strafeSpeedFR, forwardSpeedFR)

//        KotlinLogging.logger("Output Speed").info {speedFrontLeft}

        var forwardSpeedBR = forward_speed + rotation_added_speed*cos(alpha)
        var strafeSpeedBR = strafe_speed + rotation_added_speed*sin(alpha)
        var speedBackRight = sqrt(forwardSpeedBR.pow(2) + strafeSpeedBR.pow(2))
        var angleBackRight = atan2(strafeSpeedBR, forwardSpeedBR)

        // TODO : Convert m/s -> wheel speeds, set up PID for axis motors

        var speeds = mutableListOf<Double>(speedFrontLeft, speedBackLeft, speedFrontRight, speedBackRight)
        var angles = mutableListOf<Double>(angleFrontLeft, angleBackLeft, angleFrontRight, angleBackRight)

        // Conversions

        speeds.replaceAll { s -> s/WHEEL_RADIUS_METERS } // to rad/s
        speeds.replaceAll { s -> 204.8*s/(2*PI) } // to ticks/100 ms
        speeds.replaceAll { s -> s*DRIVING_RATIO } // account for driving ratio

        val driveConstant = 204.8/(2*PI)*DRIVING_RATIO/WHEEL_RADIUS_METERS

        speedFrontLeft *= driveConstant
        speedBackLeft *= driveConstant
        speedFrontRight *= driveConstant
        speedBackRight *= driveConstant

        var angleConstant = 2048/(2*PI)*STEERING_RATIO

        if (angleFrontLeft < 0) {
            angleFrontLeft += 2 * PI
        }
        if (angleBackLeft < 0) {
            angleBackLeft += 2 * PI
        }
        if (angleFrontRight < 0) {
            angleFrontRight += 2 * PI
        }
        if (angleBackRight < 0) {
            angleBackRight += 2 * PI
        }


        var encTrueValueFL = encAngleFL%(2*PI)
        var encTrueValueBL = encAngleBL%(2*PI)
        var encTrueValueFR = encAngleFR%(2*PI)
        var encTrueValueBR = encAngleBR%(2*PI)

        KotlinLogging.logger("ETV").info {encTrueValueFL/(2*PI)*360}

        var dThetaFL = angleFrontLeft - encTrueValueFL
        var dThetaBL = angleBackLeft - encTrueValueBL
        var dThetaFR = angleFrontRight - encTrueValueFR
        var dThetaBR = angleBackRight - encTrueValueBR

        if (abs(-2*PI + dThetaFL) < abs(dThetaFL)) {
            if (abs(-2*PI + dThetaFL) < abs(2*PI + dThetaFL)) {
                dThetaFL = -2*PI + dThetaFL
            } else {
                dThetaFL = 2*PI + dThetaFL
            }
        } else if (abs(dThetaFL) > abs(2*PI + dThetaFL)) {
            dThetaFL = 2*PI + dThetaFL
        }

        if (abs(-2*PI + dThetaBL) < abs(dThetaBL)) {
            if (abs(-2*PI + dThetaBL) < abs(2*PI + dThetaBL)) {
                dThetaBL = -2*PI + dThetaBL
            } else {
                dThetaBL = 2*PI + dThetaBL
            }
        } else if (abs(dThetaBL) > abs(2*PI + dThetaBL)) {
            dThetaBL = 2*PI + dThetaBL
        }

        if (abs(-2*PI + dThetaFR) < abs(dThetaFR)) {
            if (abs(-2*PI + dThetaFR) < abs(2*PI + dThetaFR)) {
                dThetaFR = -2*PI + dThetaFR
            } else {
                dThetaFR = 2*PI + dThetaFR
            }
        } else if (abs(dThetaFR) > abs(2*PI + dThetaFR)) {
            dThetaFR = 2*PI + dThetaFR
        }

        if (abs(-2*PI + dThetaBR) < abs(dThetaBR)) {
            if (abs(-2*PI + dThetaBR) < abs(2*PI + dThetaBR)) {
                dThetaBR = -2*PI + dThetaBR
            } else {
                dThetaBR = 2*PI + dThetaBR
            }
        } else if (abs(dThetaBR) > abs(2*PI + dThetaBR)) {
            dThetaBR = 2*PI + dThetaBR
        }

        KotlinLogging.logger("DTheta").info {dThetaFL/(2*PI)*360}

        angleFrontLeft = encAngleFL + dThetaFL
        angleBackLeft = encAngleBL + dThetaBL
        angleFrontRight = encAngleFR + dThetaFR
        angleBackRight = encAngleBR + dThetaBR

        KotlinLogging.logger("Angle Passed").info {angleFrontLeft/(2*PI)*360}


        var rawEnc = axisControllerFrontLeft.workaroundGetPosition()



//        angleFrontLeft += excessRots*2*PI


        angleFrontLeft *= angleConstant
        angleBackLeft *= angleConstant
        angleFrontRight *= angleConstant
        angleBackRight *= angleConstant

//        KotlinLogging.logger("Output Angle").info {angleFrontLeft}

        var outputs = listOf(angleFrontLeft, speedFrontLeft, angleBackLeft, speedBackLeft, angleFrontRight, speedFrontRight
        , angleBackRight, speedBackRight)

//        for (o in outputs) {
//            KotlinLogging.logger("Outputs").info { o.toString() }
//        }

        return outputs

    }

    fun runSwerveJoystick(lStickYAxis: Double, lStickXAxis: Double, rStickXAxis: Double) {
        val settings = Chassis.swerveDriveMath(lStickYAxis, lStickXAxis, rStickXAxis)
//        KotlinLogging.logger("Swerve Test").info {settings[1]}

        var angleFL = settings[0]
        axisControllerFrontLeft.workaroundRunPosition(angleFL)
        var speedFL = settings[1]
        talonFXFrontLeft.workaroundRunVelocity(speedFL)
        var angleBL = settings[2]
        axisControllerBackLeft.workaroundRunPosition(angleBL)
        var speedBL = settings[3]
        talonFXBackLeft.workaroundRunVelocity(speedBL)
        var angleFR = settings[4]
        axisControllerFrontRight.workaroundRunPosition(angleFR)
        var speedFR = settings[5]
        talonFXFrontRight.workaroundRunVelocity(speedFR)
        var angleBR = settings[6]
        axisControllerBackRight.workaroundRunPosition(angleBR)
        var speedBR = settings[7]
        talonFXBackRight.workaroundRunVelocity(speedBR)
//

    }

}