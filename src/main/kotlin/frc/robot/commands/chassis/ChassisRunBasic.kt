package frc.robot.commands.chassis

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Chassis

class ChassisRunBasic: InstantCommand() {

    init {
        addRequirements(Chassis)
    }

    override fun initialize() {
        Chassis.chassisBasic()
    }

}