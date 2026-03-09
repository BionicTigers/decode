package org.firstinspires.ftc.teamcode.profiles

import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Analog
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.mechanisms.Transfer

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
        override val reverse = Digital.B.press()
        override val desiredGamepad = Gamepads.GAMEPAD_1
    }

    val sorter = object : Transfer.Schema {
        override val sort = null//Digital.LEFT_BUMPER
        override val shoot = null//Digital.RIGHT_BUMPER
        override val desiredGamepad = Gamepads.GAMEPAD_1
    }

    val shooter = object : Shooter.Schema {
        override val togglePower = Digital.X.press()
        override val flywlInc = Digital.RIGHT_BUMPER.press()
        override val flywlDec = Digital.LEFT_BUMPER.press()
        override val aimLeft = Digital.DPAD_LEFT
        override val aimRight = Digital.DPAD_RIGHT
        override val hoodUp = Digital.DPAD_UP
        override val hoodDown = Digital.DPAD_DOWN
        override val desiredGamepad = Gamepads.GAMEPAD_1
    }

    companion object {
        val default = BaseProfile()
    }
}