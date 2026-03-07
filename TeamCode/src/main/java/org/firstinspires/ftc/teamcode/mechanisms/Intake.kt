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

class Intake(hardwareMap: HardwareMap): System(), Controllable<BaseProfile> {
    override val name: String = "Intake"

    interface Schema : ControlSchema {
        val intake: Digital?
        val stop: Digital?
        val toggle: Digital?
        val reverse: Digital?
    }

    val motor = hardwareMap.getByName<DcMotorEx>("intake")
    var active = false

//    init {
//        motor.direction = DcMotorSimple.Direction.REVERSE
//    }

    fun intake() = SystemCommand.instant("Intake Enable") {
        active = true
        motor.ePower = 1.0
    }

    fun stop() = SystemCommand.instant("Intake Disable") {
        active = false
        motor.ePower = 0.0
    }

    fun reverse() = SystemCommand.instant("Intake Reverse") {
        active = true
        motor.ePower = -1.0
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.intake) {
            if (!desiredGamepad.matches(gamepad)) return

            intake?.let { builder.register(it) { intake() } }
            stop?.let { builder.register(it) { stop() } }
            toggle?.let { builder.register(it) { if (active) stop() else intake() } }
            reverse?.let { builder.register(it) { reverse() } }
        }
    }
}