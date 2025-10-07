package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.getByName

class Output(hardwareMap: HardwareMap): System(), Controllable<BaseProfile> {
    override val name: String = "Output"

    interface Schema : ControlSchema {
        val shoot: Digital
        val stop: Digital
    }

    val motor = hardwareMap.getByName<DcMotorEx>("output")

    fun shoot() = Command.instant {
        motor.power = 1.0
    }

    fun stop() = Command.instant {
        motor.power = 0.0
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.output) {
            builder.register(shoot) { shoot() }
            builder.register(stop) { stop() }
        }
    }
}