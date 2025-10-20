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
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class Sorter(hardwareMap: HardwareMap, telemetry: Telemetry): System(), Controllable<BaseProfile> {
    override val name: String = "Sorter"

    interface Schema : ControlSchema {
        val forward: Digital
        val backward: Digital
//        val kick: Digital?
    }

    val motor = hardwareMap.getByName<DcMotorEx>("sorter")
    val hub = ControlHub(hardwareMap, "Control Hub")
    val pid = PID(1.0, 0.0, 0.0, 0.0, 0.0, 360.0, -0.5, 0.5)

    val state = SorterState(hub)

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        hub.setJunkTicks(3)
        pid.reset()
    }

    override val beforeRun = SystemCommand.continuous("Sorter Update", state) {
        hub.refreshBulkData()

        val deltaTicks = hub.getEncoderTicks(3)

        it.angle -= (deltaTicks / 8192.0) * 360.0

        if (it.angle < 0) {
            it.angle += 360.0
        }

        it.angle %= 360.0

        telemetry.addData("step", it.step)
        telemetry.addData("ticks from encoder", hub.getEncoderTicks(3))
        telemetry.addData("angle", it.angle)
        telemetry.addData("target", it.target)
        telemetry.addData("power", motor.power)

        when (it.step) {
            1 -> it.target = 0.0
            2 -> it.target = 120.0
            3 -> it.target = 240.0
        }

        val error = Math.toDegrees(
            atan2(
                sin(Math.toRadians(it.target - it.angle)),
                cos(Math.toRadians(it.target - it.angle))
            )
        )

        telemetry.addData("error", error)

        if (abs(error) > 5)
            motor.power = pid.compute(0.0, error)
        else
            motor.power = 0.0

        hub.setJunkTicks()
    }

    fun moveForward() = SystemCommand.instant("sorter increment", state) {
        it.step++
        if (it.step > 3) it.step = 1
//        pid.reset()
    }

    fun moveBackward() = SystemCommand.instant("sorter decrement", state) {
        it.step--
        if (it.step < 1) it.step = 3
//        pid.reset()
    }

//    fun forward() = SystemCommand.instant(state = state) {
//        motor.power = .3
//    }
//
//    fun backward() = SystemCommand.instant(state = state) {
//        motor.power = -0.3
//    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.sorter) {
            builder.register(forward) { moveForward() }
            builder.register(backward) { moveBackward() }
        }
    }
    data class SorterState(
        val hub: ControlHub,
        var step: Int = 1,
        var angle: Double = 0.0,
        var target: Double = 0.0
    ) : BaseCommandState()
}