package org.firstinspires.ftc.teamcode.profiles

import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.teamcode.mechanisms.MilesIntake
import org.firstinspires.ftc.teamcode.mechanisms.MilesSlides
import org.firstinspires.ftc.teamcode.mechanisms.Output

open class BaseProfile {
    open val milesIntake = object : MilesIntake.Schema {
        override val suckOut: Digital = Digital.DPAD_LEFT.hold()
        override val suckIn: Digital = Digital.DPAD_RIGHT.hold()
        override val desiredGamepad: Gamepads = Gamepads.GAMEPAD_1

    }
    open val milesSlides = object : MilesSlides.Schema {
        override val up: Digital = Digital.DPAD_UP.press()
        override val down: Digital = Digital.DPAD_DOWN.press()

        override val desiredGamepad: Gamepads = Gamepads.GAMEPAD_1
    }
    open val output = object : Output.Schema{
        override val shootClose: Digital = Digital.A.press()
        override val shootFar: Digital = Digital.B.press()

        override val desiredGamepad: Gamepads = Gamepads.GAMEPAD_1

    }



    companion object {
        val default = BaseProfile()
    }
}