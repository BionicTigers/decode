package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import io.github.bionictigers.axiom.core.commands.BaseCommandState
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.TimeMark
import kotlin.time.TimeSource

class Kicker(hardwareMap: HardwareMap, telemetry: Telemetry? = null): System(), Controllable<BaseProfile> {
    override val name: String = "Kicker"

    val state = KickerState(TimeSource.Monotonic.markNow())

    val defaultPosition = 0.0
    val kickPosition = 0.8

    interface Schema : ControlSchema {
        val kick: Digital
    }

    override val beforeRun = SystemCommand.continuous("Kicker Update", state) {
        if (it.lastKickedAt.elapsedNow() > 250.milliseconds) {
            servo.position = defaultPosition
        }
    }

    val servo = hardwareMap.getByName<Servo>("kicker")

    fun kick() = SystemCommand.instant("Kick", state) {
        servo.position = kickPosition
        it.lastKickedAt = TimeSource.Monotonic.markNow()
        println("kick")
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.kicker) {
            builder.register(kick) { kick() }
        }
    }
    data class KickerState(
        var lastKickedAt: TimeMark
    ) : BaseCommandState()
}
