package org.firstinspires.ftc.teamcode.mechanisms

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

class Output(hardwareMap: HardwareMap): System(), Controllable<BaseProfile> {
    interface Schema: ControlSchema{
        val shootClose: Digital
        val shootFar: Digital
    }
    override val name = "output"
    val output = hardwareMap.getByName<DcMotorEx>("output")
    fun shootClose(): BaseCommand = Command.instant{ output.power = 1.0  }
    fun shootFar(): BaseCommand = Command.instant { output.power = 0.4 }
    override fun bindControls(
        profile: BaseProfile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ) {
        profile.output.shootClose.let { builder.register(it) { shootClose() } }
        profile.output.shootFar.let { builder.register(it) { shootFar() } }
    }

}
