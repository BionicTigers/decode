package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Digital
import io.github.bionictigers.axiom.core.web.Editable
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.drivers.OctoQuadFWv3
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.RollingAverage
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.abs
import kotlin.math.min

class Intake(hardwareMap: HardwareMap, val octoQuad: OctoQuad): System(), Controllable<BaseProfile> {
    //ch 1
    override val name: String = "Intake"

    interface Schema : ControlSchema {
        val intake: Digital?
        val stop: Digital?
        val toggle: Digital?
    }

    val motor = hardwareMap.getByName<DcMotorEx>("intake")

    var active: Boolean = false
    var velocity = RollingAverage(3)
    @Editable
    var targetVelocity = 2700.0
    private val velocitySampleIntervalMs = 20

    @Editable
    var pid = PID(1.0, 0.0, 0.0, 0.0, 0.0, 2700.0, -1.0, 1.0)
    @Editable
    var ff = .00075

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
        octoQuad.octoQuad.setSingleEncoderDirection(6, OctoQuadFWv3.EncoderDirection.FORWARD)
        octoQuad.octoQuad.setSingleVelocitySampleInterval(6, velocitySampleIntervalMs)
    }

    override val apply = SystemCommand.continuous("Intake After Run") {
        val ticksPerSec = octoQuad.encoderData.velocity[6] * 1000 / velocitySampleIntervalMs
        velocity.plusAssign(ticksPerSec)
        if (active) {
            motor.ePower = (pid.compute(abs(velocity.average), targetVelocity) + ff * targetVelocity).coerceIn(-1.0, 1.0)
        }
    }

    fun intake() = SystemCommand.instant("Intake Enable") {
        active = true
    }

    fun stop() = SystemCommand.instant("Intake Disable") {
        motor.power = 0.0
        active = false
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.intake) {
            if (!desiredGamepad.matches(gamepad)) return

            intake?.let { builder.register(it) { intake() } }
            stop?.let { builder.register(it) { stop() } }
            toggle?.let { builder.register(it) { if (active) stop() else intake() } }
        }
    }
}