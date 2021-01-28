package frc.robot.commands.test

import edu.wpi.first.wpilibj2.command.CommandBase
import mu.KotlinLogging

class TestCommand: CommandBase() {

    init {

    }

    override fun initialize() {
        KotlinLogging.logger("TestCommand").info { "TestCommand test successful" }
    }

}