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
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.min

class Intake(hardwareMap: HardwareMap, val drivetrain: Drivetrain? = null): System(), Controllable<BaseProfile> {
    override val name: String = "Intake"

    interface Schema : ControlSchema {
        val intake: Digital?
        val stop: Digital?
        val toggle: Digital?
    }

    val motor = hardwareMap.getByName<DcMotorEx>("intake")

    var active: Boolean = false
    var stillSpeed: Double = .7

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
    }

    override val apply = SystemCommand.continuous("Intake After Run") {
        if (active) {
            val power = stillSpeed + (drivetrain?.yControl ?: 1.0) * (1.0 - stillSpeed)
            if (motor.power != power) motor.power = power
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