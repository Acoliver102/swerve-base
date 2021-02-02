package frc.robot.commands.test

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.Chassis
import mu.KotlinLogging

class ResetNAVX: InstantCommand() {

    init {

    }

    override fun initialize() {
        Chassis.resetHeading()
    }

}