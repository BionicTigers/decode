package org.firstinspires.ftc.teamcode.profiles

import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Analog
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Output

open class BaseProfile {
    val drivetrain = object : Drivetrain.Schema {
        override val desiredGamepad = Gamepads.GAMEPAD_1
        override val orientation = Drivetrain.DriveOrientation.ROBOT
        override val x = Analog.LEFT_STICK_X
        override val y = Analog.LEFT_STICK_Y
        override val rot = Analog.RIGHT_STICK_X
    }

    val intake = object : Intake.Schema {
        override val intake = Digital.A
        override val stop = Digital.B
        override val desiredGamepad = Gamepads.GAMEPAD_2
    }

    val output = object : Output.Schema {
        override val shoot = Digital.Y
        override val stop = Digital.X
        override val desiredGamepad = Gamepads.GAMEPAD_2
    }

    companion object {
        val default = BaseProfile()
    }
}