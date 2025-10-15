package org.firstinspires.ftc.teamcode.mechanisms

import android.transition.Slide
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.BaseCommand
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema

import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.getByName


class MilesSlides(hardwareMap: HardwareMap): System(), Controllable<BaseProfile> {
    interface Schema : ControlSchema{
        val up: Digital
        val down: Digital
    }
    override val name = "slides"

    val slides = hardwareMap.getByName<DcMotorEx>("slides")
    fun up(): BaseCommand = Command.instant { slides.power = .3 }
    fun down(): BaseCommand = Command.instant { slides.power = -.3 }
    override fun bindControls(
        profile: BaseProfile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ) {
        profile.milesSlides.up.let { builder.register(it) { up() } }
        profile.milesSlides.down.let { builder.register(it) { down() } }
    }


}