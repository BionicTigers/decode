package org.firstinspires.ftc.teamcode.profiles

import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Analog
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter

open class BaseProfile {
    val drivetrain = object : Drivetrain.Schema {
        override val desiredGamepad = Gamepads.GAMEPAD_1
        override val orientation = Drivetrain.DriveOrientation.ROBOT
        override val x = Analog.LEFT_STICK_X.continuous()
        override val y = Analog.LEFT_STICK_Y.continuous()
        override val rot = Analog.RIGHT_STICK_X.continuous()
    }

    val intake = object : Intake.Schema {
        override val intake = null
        override val stop = null
        override val toggle = Digital.A.press()
        override val desiredGamepad = Gamepads.GAMEPAD_1
    }

    val output = object : Output.Schema {
        override val shoot = null
        override val stop = null
        override val toggle = Digital.Y.press()
        override val toggleSlow = Digital.B.press()
        override val desiredGamepad = Gamepads.GAMEPAD_2
    }

    val sorter = object : Sorter.Schema {
        override val forward = Digital.RIGHT_BUMPER.press()
        override val backward = Digital.LEFT_BUMPER.press()
        override val green = Digital.DPAD_LEFT.press()
        override val purple = Digital.DPAD_RIGHT.press()
        override val openIntake = null
        override val openHuman = null
        override val desiredGamepad = Gamepads.GAMEPAD_2
    }

    val kicker = object : Kicker.Schema {
        override val kick : Digital = Digital.X.press()
        override val up : Digital = Digital.DPAD_UP.press()
        override val down: Digital = Digital.DPAD_DOWN.press()
        override val desiredGamepad = Gamepads.GAMEPAD_2
    }

    companion object {
        val default = BaseProfile()
    }
}