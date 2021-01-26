package frc.robot.commands.chassis

import edu.wpi.first.wpilibj.SlewRateLimiter
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Controls
import frc.robot.subsystems.Chassis

class ChassisRunSwerve: CommandBase() {
    private val speedLimiter = SlewRateLimiter(0.5) // Cap accel and sens
    private val rotationLimiter = SlewRateLimiter(0.5)

    init {
        addRequirements(Chassis)
    }

    override fun execute() {
        Chassis.runSwerveJoystick(
            -speedLimiter.calculate(Controls.controller.getRawAxis(1)),
            speedLimiter.calculate(Controls.controller.getRawAxis(0)),
            rotationLimiter.calculate(Controls.controller.getRawAxis(4))
        )
    }

}