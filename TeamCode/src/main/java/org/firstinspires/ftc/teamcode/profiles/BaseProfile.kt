package org.firstinspires.ftc.teamcode.profiles

import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Analog
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain

open class BaseProfile {
    val drivetrain = object : Drivetrain.Schema {
        override val desiredGamepad = Gamepads.GAMEPAD_1
        override val orientation = Drivetrain.DriveOrientation.ROBOT
        override val x = Analog.LEFT_STICK_X
        override val y = Analog.LEFT_STICK_Y
        override val rot = Analog.RIGHT_STICK_X
    }

    companion object {
        val default = BaseProfile()
    }
}