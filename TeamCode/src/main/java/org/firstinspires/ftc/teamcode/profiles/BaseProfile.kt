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
        override val intake = Digital.A
        override val stop = Digital.B
        override val speedUp = Digital.DPAD_UP
        override val speedDown = Digital.DPAD_DOWN
        override val desiredGamepad = Gamepads.GAMEPAD_2
    }

    val output = object : Output.Schema {
        override val shoot = Digital.Y
        override val stop = Digital.X
        override val speedDown = Digital.DPAD_DOWN
        override val speedUp = Digital.DPAD_UP
        override val desiredGamepad = Gamepads.GAMEPAD_2
    }

    val sorter = object : Sorter.Schema {
        override val forward = Digital.RIGHT_BUMPER.release()
        override val backward = Digital.LEFT_BUMPER.release()
        override val desiredGamepad = Gamepads.GAMEPAD_2
    }

    val kicker = object : Kicker.Schema {
        override val kick : Digital = Digital.DPAD_UP
        override val desiredGamepad = Gamepads.GAMEPAD_1
    }

    companion object {
        val default = BaseProfile()
    }
}