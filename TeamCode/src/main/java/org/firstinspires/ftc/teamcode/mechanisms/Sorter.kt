package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.ColorSensor
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
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

class Sorter(hardwareMap: HardwareMap, kicker: Kicker? = null, telemetry: Telemetry? = null): System(), Controllable<BaseProfile> {
    override val name: String = "Sorter"

    enum class BallColor {
        Green,
        Purple,
        None
    }

    interface Schema : ControlSchema {
        val forward: Digital?
        val backward: Digital?                                                                                      
        val green: Digital?
        val purple: Digital?
        val openIntake: Digital?
        val openHuman: Digital?
//        val kick: Digital?
    }

    val motor = hardwareMap.getByName<DcMotorEx>("sorter")
    val hub = ControlHub(hardwareMap, "Control Hub")
    val colorSensor = hardwareMap.getByName<ColorSensor>("intakeColor")

    val state = SorterState(hub)

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        hub.setJunkTicks(3)
        state.pid.reset()
    }

    override val beforeRun = SystemCommand.continuous("Sorter Update", state) {
        val isGreen = colorSensor.green() > 200
        val isPurple = colorSensor.red() > 100 && colorSensor.blue() > 100

        if (isGreen)
            it.colors[it.step] = BallColor.Green
        else if (isPurple)
            it.colors[it.step] = BallColor.Purple

        if (kicker?.state?.kickedThisCycle ?: false) {
            it.colors[(it.step + 2) % 3] = BallColor.None
        }
    }

    override val afterRun = SystemCommand.continuous("Sorter Update", state) {
        hub.refreshBulkData()

        val deltaTicks = hub.getEncoderTicks(3)

        it.angle -= (deltaTicks / 8192.0) * 360.0

        if (it.angle < 0) {
            it.angle += 360.0
        }

        it.angle %= 360.0

        telemetry?.addData("step", it.step)
        telemetry?.addData("ticks from encoder", hub.getEncoderTicks(3))
        telemetry?.addData("angle", it.angle)
        telemetry?.addData("target", it.target)
        telemetry?.addData("power", motor.power)

        when (it.step) {
            0 -> it.target = 0.0
            1 -> it.target = 120.0
            2 -> it.target = 240.0
        }

        val error = Math.toDegrees(
            atan2(
                sin(Math.toRadians(it.target - it.angle)),
                cos(Math.toRadians(it.target - it.angle))
            )
        )

        telemetry?.addData("error", error)

        if (abs(error) > 1)
            motor.power = -it.pid.compute(0.0, error)
        else
            motor.power = 0.0

        if (kicker?.state?.reset ?: false)
            motor.power += .1

        hub.setJunkTicks()
    }

//    override val beforeRun = SystemCommand.continuous("Sorter Update Simple", state) {
//        hub.refreshBulkData()
//
//        it.ticks = hub.getEncoderTicks(3).toDouble()
//
//        telemetry.addData("ticks from encoder", it.ticks)
//        telemetry.addData("target", it.target)
//        telemetry.addData("power", motor.power)
//
//        if (abs(it.target - it.ticks) > 5)
//            motor.power = pid.compute(it.ticks, it.target)
//        else
//            motor.power = 0.0
//    }

    fun moveForward() = SystemCommand.instant("sorter increment", state) {
        it.step = (it.step + 1) % 3
    }

    fun moveBackward() = SystemCommand.instant("sorter decrement", state) {
        it.step = (it.step + 2) % 3
    }

    fun forward() = SystemCommand.instant("sorter forward", state) {
        it.target = (it.ticks / (8192.0 / 3.0)).roundToInt() * 8192.0 / 3.0 + 8192.0 / 3.0
    }

    fun backward() = SystemCommand.instant("sorter forward", state) {
        it.target = (it.ticks / (8192.0 / 3.0)).roundToInt() * 8192.0 / 3.0 - 8192.0 / 3.0
    }

    fun targetGreen() = SystemCommand.instant("target green", state) {
        val currentStep = state.step
        val desiredOutputStep = (currentStep + 2) % 3

        val greenSteps = state.colors.mapIndexed { index, color -> if (color == BallColor.Green) index else -1 }.filter { it != -1 }
        if (greenSteps.isEmpty()) return@instant

        // Choose the nearest forward-rotation green index (respecting forward-only major movement)
        val forwardDistances = greenSteps.map { targetIndex ->
            val forwardSteps = (targetIndex - currentStep + 3) % 3
            targetIndex to forwardSteps
        }

        val (nearestGreenIndex, _) = forwardDistances.minBy { it.second }

        // Compute how many forward steps are needed to bring that green to the output index
        val deltaStepsToOutput = (desiredOutputStep - nearestGreenIndex + 3) % 3
        val newStep = (currentStep + deltaStepsToOutput) % 3

        state.step = newStep
    }

    fun targetPurple() = SystemCommand.instant("target purple", state) {
        val currentStep = state.step
        val desiredOutputStep = (currentStep + 2) % 3

        val purpleSteps = state.colors.mapIndexed { index, color -> if (color == BallColor.Purple) index else -1 }.filter { it != -1 }
        if (purpleSteps.isEmpty()) return@instant

        val forwardDistances = purpleSteps.map { targetIndex ->
            val forwardSteps = (targetIndex - currentStep + 3) % 3
            targetIndex to forwardSteps
        }

        val (nearestPurpleIndex, _) = forwardDistances.minBy { it.second }

        val deltaStepsToOutput = (desiredOutputStep - nearestPurpleIndex + 3) % 3
        val newStep = (currentStep + deltaStepsToOutput) % 3

        state.step = newStep
    }

    fun openIntake() = SystemCommand.instant("open intake", state) {
        val currentStep = state.step
        val desiredStep = 0

        val noneSteps = state.colors.mapIndexed { index, color -> if (color == BallColor.None) index else -1 }.filter { it != -1 }
        if (noneSteps.isEmpty()) return@instant

        val deltas = noneSteps.map { idx -> idx to ((desiredStep - idx + 3) % 3) }
        val (_, minDelta) = deltas.minBy { it.second }
        state.step = (currentStep + minDelta) % 3
    }

    fun openHuman() = SystemCommand.instant("open human", state) {
        val currentStep = state.step
        val desiredStep = 2

        val noneSteps = state.colors.mapIndexed { index, color -> if (color == BallColor.None) index else -1 }.filter { it != -1 }
        if (noneSteps.isEmpty()) return@instant

        val deltas = noneSteps.map { idx -> idx to ((desiredStep - idx + 3) % 3) }
        val (_, minDelta) = deltas.minBy { it.second }
        state.step = (currentStep + minDelta) % 3
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.sorter) {
            if (!desiredGamepad.matches(gamepad)) return

            forward?.let { builder.register(it) { moveForward() } }
            backward?.let { builder.register(it) { moveBackward() } }
            green?.let { builder.register(it) { targetGreen() } }
            purple?.let { builder.register(it) { targetPurple() } }
            openIntake?.let { builder.register(it) { openIntake() } }
            openHuman?.let { builder.register(it) { openHuman() } }
        }
    }
    data class SorterState(
        val hub: ControlHub,
        var step: Int = 1,
        var angle: Double = 0.0,
        var target: Double = 0.0,
        var ticks: Double = 0.0,
        val pid: PID = PID(2.0, 0.0, 0.0, 0.0, 0.0, 360.0, -0.65, 0.65),
        val colors: MutableList<BallColor> = MutableList(3) { BallColor.None }
    ) : BaseCommandState()
}