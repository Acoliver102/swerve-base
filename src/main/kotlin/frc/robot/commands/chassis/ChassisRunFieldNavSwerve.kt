package frc.robot.commands.chassis

import edu.wpi.first.wpilibj.SlewRateLimiter
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.Controls
import frc.robot.subsystems.Chassis
import mu.KotlinLogging
import kotlin.math.cos
import kotlin.math.sin

class ChassisRunFieldNavSwerve: CommandBase() {
    private val speedLimiter = SlewRateLimiter(2.5) // Cap accel and sens
    private val rotationLimiter = SlewRateLimiter(2.5)

    init {
        addRequirements(Chassis)
    }

    override fun execute() {
        Chassis.runSwerveJoystick(
                Controls.controller.getRawAxis(1)*cos(Chassis.heading) - Controls.controller.
                        getRawAxis(0)*sin(Chassis.heading),
                -Controls.controller.getRawAxis(1)*sin(Chassis.heading) - Controls.controller.
                        getRawAxis(0)*cos(Chassis.heading),
                Controls.controller.getRawAxis(4)
        )
//        KotlinLogging.logger("Swerver").info {"Working on it."}
    }

}