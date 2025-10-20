package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.BaseCommandState
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.min

class Intake(hardwareMap: HardwareMap): System(), Controllable<BaseProfile> {
    override val name: String = "Intake"

    interface Schema : ControlSchema {
        val intake: Digital
        val stop: Digital
        val speedUp: Digital
        val speedDown: Digital
    }

    val motor = hardwareMap.getByName<DcMotorEx>("intake")

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
    }
    var state = StateIntake()

    fun intake() = Command.instant("Intake Enable", state) {
        motor.power = it.speed
    }

    fun stop() = Command.instant {
        motor.power = 0.0
    }

    fun speedUp() = Command.instant("intake Increase",state) {
        it.speed = (it.speed + 0.1).coerceIn(0.0, 1.0)
        motor.power = it.speed
    }

    fun slowDown() = Command.instant("intake decrease",state) {
        it.speed = (it.speed - 0.1).coerceIn(0.0, 1.0)
        motor.power = it.speed
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.intake) {
            builder.register(intake) { intake() }
            builder.register(stop) { stop() }
            builder.register(speedUp) { speedUp() }
            builder.register(speedDown) { slowDown() }
        }
    }
    data class StateIntake(var speed: Double = 0.8): BaseCommandState()


}