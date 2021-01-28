package frc.robot.commands.test

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import mu.KotlinLogging

class TestCommand: InstantCommand() {

    init {

    }

    override fun initialize() {
        KotlinLogging.logger("TestCommand").info { "TestCommand test successful" }
    }

}