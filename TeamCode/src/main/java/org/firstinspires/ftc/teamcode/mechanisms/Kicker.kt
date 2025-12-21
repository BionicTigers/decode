package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Digital
import io.github.bionictigers.axiom.core.web.Editable
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.TimeMark
import kotlin.time.TimeSource

class Kicker(hardwareMap: HardwareMap, telemetry: Telemetry? = null): System(), Controllable<BaseProfile> {
    override val name: String = "Kicker"

    var lastKickedAt: TimeMark = TimeSource.Monotonic.markNow()
    @Editable
    var defaultPosition: Double = 0.35
    @Editable
    var kickPosition: Double = .95
    @Editable
    var upPosition: Double = 0.1
    var reset: Boolean = false
    var kickedThisCycle: Boolean = false
    var up: Boolean = false
    var time: Duration = 250.milliseconds
    interface Schema : ControlSchema {
        val kick: Digital?
        val up: Digital?
        val down: Digital?
    }

    override val update = SystemCommand.continuous("Kicker Update") {
        kickedThisCycle = false
        if (lastKickedAt.elapsedNow() > time && reset) {
            servo.position = defaultPosition
            reset = false
        }
    }

    val servo = hardwareMap.getByName<Servo>("kicker")

    fun kick() = SystemCommand.instant("Kick") {
        servo.position = kickPosition
        lastKickedAt = TimeSource.Monotonic.markNow()
        reset = true
        kickedThisCycle = true
        up = false
        time = 250.milliseconds
    }

    fun up() = SystemCommand.instant("Up") {
        servo.position = upPosition
        reset = false
        up = true
    }

    fun down() = SystemCommand.instant("Down") {
        servo.position = defaultPosition
        reset = true
        up = false
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.kicker) {
            if (!desiredGamepad.matches(gamepad)) return

            kick?.let { builder.register(it) { kick() } }
            up?.let { builder.register(it) { up() } }
            down?.let {builder.register(it) { down() } }
        }
    }
}
