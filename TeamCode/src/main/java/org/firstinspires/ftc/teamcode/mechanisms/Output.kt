package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
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
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Digital
import io.github.bionictigers.axiom.core.web.Editable
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.max

class Output(hardwareMap: HardwareMap, telemetry: Telemetry? = null): System(), Controllable<BaseProfile> {
    override val name: String = "Output"

    interface Schema : ControlSchema {
        val shoot: Digital?
        val stop: Digital?
        val toggle: Digital?
    }

    val motor = hardwareMap.getByName<DcMotorEx>("output")

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    var state = StateOutput()

    override val beforeRun = SystemCommand.continuous("Output Data", state) {
        it.velocity = motor.velocity
        it.maxVelocity = max(it.velocity, it.maxVelocity)
        telemetry?.addData("maxVelocity", it.maxVelocity)

//        if (it.active) {
//            motor.ePower = it.pid.compute(it.velocity, it.targetVelocity)
//        } else {
//            motor.ePower = 0.0
//        }
    }

    fun shoot() = SystemCommand.instant("Output Enable", state) {
        motor.power = 0.9
        it.active = true
    }

    fun stop() = SystemCommand.instant("Output Disable", state) {
        motor.power = 0.0
        it.active = false
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.output) {
            if (!desiredGamepad.matches(gamepad)) return

            shoot?.let { builder.register(it) { shoot() } }
            stop?.let { builder.register(it) { stop() } }
            toggle?.let { builder.register(it) { if (state.active) stop() else shoot() } }
        }
    }
    data class StateOutput(
        var active: Boolean = false,
        var velocity: Double = 0.0,
        var targetVelocity: Double = 0.0,
        @Editable
        val pid: PID = PID(1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        var maxVelocity: Double = 0.0
    ): BaseCommandState()
}