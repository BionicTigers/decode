package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import io.github.bionictigers.axiom.core.commands.BaseCommandState
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
import org.firstinspires.ftc.teamcode.utils.NewRollingAverage
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.eq
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.seconds
import kotlin.math.max

class Output(hardwareMap: HardwareMap, telemetry: Telemetry? = null): System(), Controllable<BaseProfile> {
    override val name: String = "Output"

    interface Schema : ControlSchema {
        val shoot: Digital?
        val stop: Digital?
        val toggle: Digital?
        val toggleSlow: Digital?
    }

    val motor = hardwareMap.getByName<DcMotorEx>("output")
    val indcLight = hardwareMap.getByName<Servo>("indcLight")
    val hub = ControlHub(hardwareMap, "Control Hub")

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER
        hub.setJunkTicks()
    }
    companion object {
        var farTarget = 1880.0
        var closeTarget = 1680.0
    }

    var state = StateOutput()

    override val beforeRun = SystemCommand.continuous("Output Data", state) {
        hub.refreshBulkData()
        val lVel = hub.getEncoderTicks(0) / it.deltaTime.seconds
        if (lVel.isFinite()) {
            it.velocity.plusAssign(lVel)
        }
        it.maxVelocity = max(it.velocity.average, it.maxVelocity)
        telemetry?.addData("maxVelocity", it.maxVelocity)
        telemetry?.addData("target", it.targetVelocity)
        telemetry?.addData("velocity", it.velocity.average)
        telemetry?.addData("power", motor.ePower)

        if (it.active) {
            motor.ePower = it.pid.compute(it.velocity.average, it.targetVelocity) + .05
        } else {
            motor.ePower = 0.0
        }

        if (it.active && it.velocity.average.eq(it.targetVelocity, 45.0))
            indcLight.position = .5
        else if (it.active)
            indcLight.position = .28
        else
            indcLight.position = .63


        hub.setJunkTicks()
    }

    fun shoot() = SystemCommand.instant("Output Enable", state) {
//        motor.power = .83
        it.targetVelocity = farTarget
        it.active = true
    }

    fun shootClose() = SystemCommand.instant("Output Enable Slow", state) {
//        motor.power = .73
        it.targetVelocity = closeTarget
        it.active = true
    }

    fun stop() = SystemCommand.instant("Output Disable", state) {
//        motor.power = 0.0
        it.targetVelocity = 0.0
        it.active = false
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.output) {
            if (!desiredGamepad.matches(gamepad)) return

            shoot?.let { builder.register(it) { shoot() } }
            stop?.let { builder.register(it) { stop() } }
            toggle?.let { builder.register(it) { if (state.active && state.targetVelocity == farTarget) stop() else shoot() } }
            toggleSlow?.let { builder.register(it) { if (state.active && state.targetVelocity == closeTarget) stop() else shootClose() } }
        }
    }
    data class StateOutput(
        var active: Boolean = false,
        var velocity: NewRollingAverage = NewRollingAverage(5),
        var targetVelocity: Double = 0.0,
        @Editable
        val pid: PID = PID(50.0, 2.0, 0.0, 1.0, 0.0, 2100.0, 0.0, 1.0),
        var maxVelocity: Double = 0.0,
    ): BaseCommandState()
}