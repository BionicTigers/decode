package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
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

class Intake(hardwareMap: HardwareMap): System(), Controllable<BaseProfile> {
    override val name: String = "Intake"

    interface Schema : ControlSchema {
        val intake: Digital
        val stop: Digital
    }

    val motor = hardwareMap.getByName<DcMotorEx>("intake")

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
    }
    var newSpeed = 0.8
    fun intake() = Command.instant {
        motor.power = newSpeed
    }

    fun stop() = Command.instant {
        motor.power = 0.0
    }

    fun speedUp() = Command.instant {
         newSpeed = motor.power + 0.1
        if (newSpeed >= 1.0) {motor.power = 1.0; newSpeed = 1.0}
        newSpeed = motor.power
    }

    fun slowDown() = Command.instant {
         newSpeed = motor.power - 0.1
        if (newSpeed <= 0.0) {motor.power = 0.0; newSpeed = 0.0}
        newSpeed = motor.power
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.intake) {
            builder.register(intake) { intake() }
            builder.register(stop) { stop() }
        }
    }

}