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
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Digital
import io.github.bionictigers.axiom.core.web.Editable
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

    interface Schema : ControlSchema {
        val kick: Digital?
        val up: Digital?
    }

    override val beforeRun = SystemCommand.continuous("Kicker Update", state) {
        it.kickedThisCycle = false
        if (it.lastKickedAt.elapsedNow() > 250.milliseconds && it.reset) {
            servo.position = it.defaultPosition
            it.reset = false
        }
    }

    val servo = hardwareMap.getByName<Servo>("kicker")

    fun kick() = SystemCommand.instant("Kick", state) {
        servo.position = it.kickPosition
        it.lastKickedAt = TimeSource.Monotonic.markNow()
        it.reset = true
        it.kickedThisCycle = true
    }

    fun up() = SystemCommand.instant("Up", state) {
        servo.position = it.upPosition
        it.reset = false
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.kicker) {
            if (!desiredGamepad.matches(gamepad)) return

            kick?.let { builder.register(it) { kick() } }
            up?.let { builder.register(it) { up() } }
        }
    }
    data class KickerState(
        var lastKickedAt: TimeMark,
        @Editable
        var defaultPosition: Double = 0.35,
        @Editable
        var kickPosition: Double = .95,
        @Editable
        var upPosition: Double = 0.0,
        var reset: Boolean = false,
        var kickedThisCycle: Boolean = false
    ) : BaseCommandState()
}
