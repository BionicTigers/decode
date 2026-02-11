package org.firstinspires.ftc.teamcode.profiles

import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter

object OutputTune : BaseProfile() {
    override val output = object : Output.Schema {
        override val shoot = null
        override val stop = null
        override val toggle = null
        override val toggleSlow = null
        override val aimLeft = null
        override val aimRight = null
        override val resetOdometry = null
        override val incVel: Digital = Digital.DPAD_UP.press()
        override val decVel: Digital = Digital.DPAD_DOWN.press()
        override val smallIncVel: Digital = Digital.DPAD_RIGHT.press()
        override val smallDecVel: Digital = Digital.DPAD_LEFT.press()
        override val desiredGamepad = Gamepads.GAMEPAD_1 // change back to 2 before comp
    }

    override val kicker = object : Kicker.Schema {
        override val kick : Digital = Digital.X.press()
        override val up : Digital = Digital.A.press()
        override val down: Digital = Digital.B.press()
        override val desiredGamepad = Gamepads.GAMEPAD_1
    }

    override val sorter = object : Sorter.Schema {
        override val forward = Digital.RIGHT_BUMPER.press()
        override val backward = Digital.LEFT_BUMPER.press()
        override val green = null
        override val purple = null
        override val openIntake = null
        override val openHuman = null
        override val outputToggle = Digital.A.press()
        override val desiredGamepad = Gamepads.GAMEPAD_1
    }
}