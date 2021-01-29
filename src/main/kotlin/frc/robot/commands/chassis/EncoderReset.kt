package frc.robot.commands.chassis

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Chassis
import mu.KotlinLogging

class EncoderReset: InstantCommand() {

    init {

    }

    override fun initialize() {
        Chassis.resetMotorEncoder()
    }

}