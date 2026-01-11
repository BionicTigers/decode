package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.DynamicPID
import org.firstinspires.ftc.teamcode.control.GainSchedule
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.time.Duration.Companion.milliseconds

class Sorter(hardwareMap: HardwareMap, kicker: Kicker? = null, telemetry: Telemetry? = null): System(), Controllable<BaseProfile> {
    override val name: String = "Sorter"

    var step: Int = 0
    var angle: Double = 0.0
    var target: Double = 0.0
    var ticks: Double = 0.0
    var isOutput: Boolean = true
    val pid: DynamicPID = DynamicPID(GainSchedule(
        mapOf(
            0.0 to 3.0, //8.0,
            90.0 to 3.0, //1.5,
            180.0 to 3.0, //.4, //2, 1.5, .75
        ),
        mapOf(0.0 to 0.0),
        mapOf(0.0 to 0.0)
    ), 0.0, -180.0, 180.0, -1.0, 1.0, 20.milliseconds, { abs(it) })
    val colors: MutableList<BallColor> = MutableList(3) { BallColor.None }

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
//    val colorSensor = hardwareMap.getByName<ColorSensor>("intakeColor")
//    val limitSwitch = hardwareMap.getByName<DigitalChannel>("limitSwitch")

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        hub.setJunkTicks(3)
        pid.reset()
    }

    override val update = SystemCommand.continuous("Sorter Update") {
//        val isGreen = colorSensor.green() > 200
//        val isPurple = colorSensor.red() > 100 && colorSensor.blue() > 100
//
//        if (isGreen)
//            colors[step] = BallColor.Green
//        else if (isPurple)
//            colors[step] = BallColor.Purple
//
//        if (kicker?.state?.kickedThisCycle ?: false) {
//            colors[(step + 2) % 3] = BallColor.None
//        }
//        if (limitSwitch.state) {
//            println("on")
//        } else {
//            println("off")
//        }
    }

    override val apply = SystemCommand.continuous("Sorter Update") {
        hub.refreshBulkData()

        val deltaTicks = hub.getEncoderTicks(3)

        angle -= (deltaTicks / 8192.0) * 360.0

        if (angle < 0) {
            angle += 360.0
        }

        angle %= 360.0

        telemetry?.addData("step", step)
        telemetry?.addData("ticks from encoder", hub.getEncoderTicks(3))
        telemetry?.addData("angle", angle)
        telemetry?.addData("target", target)
        telemetry?.addData("power", motor.power)

        when (step) {
            0 -> target = 0.0
            1 -> target = 120.0
            2 -> target = 240.0
        }

        val error = -Math.toDegrees(
            atan2(
                sin(Math.toRadians(target - angle)),
                cos(Math.toRadians(target - angle))
            )
        )

        telemetry?.addData("error", error)

        if (abs(error) > .5)
            motor.ePower = -pid.compute(0.0, error)
        else
            motor.ePower = 0.0

        if (kicker?.reset ?: false) {
            if (!kicker.up)
                motor.ePower -= .35
            else
                motor.ePower -= .35
        }


        hub.setJunkTicks()
    }

//    override val beforeRun = SystemCommand.continuous("Sorter Update Simple", state) {
//        hub.refreshBulkData()
//
//        ticks = hub.getEncoderTicks(3).toDouble()
//
//        telemetry.addData("ticks from encoder", ticks)
//        telemetry.addData("target", target)
//        telemetry.addData("power", motor.power)
//
//        if (abs(target - ticks) > 5)
//            motor.power = pid.compute(ticks, target)
//        else
//            motor.power = 0.0
//    }

    fun moveForward() = SystemCommand.instant("sorter increment") {
        step = (step + 1) % 3
    }

    fun moveBackward() = SystemCommand.instant("sorter decrement") {
        step = (step + 2) % 3
    }

    fun forward() = SystemCommand.instant("sorter forward") {
        target = (ticks / (8192.0 / 3.0)).roundToInt() * 8192.0 / 3.0 + 8192.0 / 3.0
    }

    fun backward() = SystemCommand.instant("sorter forward") {
        target = (ticks / (8192.0 / 3.0)).roundToInt() * 8192.0 / 3.0 - 8192.0 / 3.0
    }

    fun targetGreen() = SystemCommand.instant("target green") {
        val currentStep = step
        val desiredOutputStep = (currentStep + 2) % 3

        val greenSteps = colors.mapIndexed { index, color -> if (color == BallColor.Green) index else -1 }.filter { it != -1 }
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

        step = newStep
    }

    fun targetPurple() = SystemCommand.instant("target purple") {
        val currentStep = step
        val desiredOutputStep = (currentStep + 2) % 3

        val purpleSteps = colors.mapIndexed { index, color -> if (color == BallColor.Purple) index else -1 }.filter { it != -1 }
        if (purpleSteps.isEmpty()) return@instant

        val forwardDistances = purpleSteps.map { targetIndex ->
            val forwardSteps = (targetIndex - currentStep + 3) % 3
            targetIndex to forwardSteps
        }

        val (nearestPurpleIndex, _) = forwardDistances.minBy { it.second }

        val deltaStepsToOutput = (desiredOutputStep - nearestPurpleIndex + 3) % 3
        val newStep = (currentStep + deltaStepsToOutput) % 3

        step = newStep
    }

    fun openIntake() = SystemCommand.instant("open intake") {
        val currentStep = step
        val desiredStep = 0

        val noneSteps = colors.mapIndexed { index, color -> if (color == BallColor.None) index else -1 }.filter { it != -1 }
        if (noneSteps.isEmpty()) return@instant

        val deltas = noneSteps.map { idx -> idx to ((desiredStep - idx + 3) % 3) }
        val (_, minDelta) = deltas.minBy { it.second }
        step = (currentStep + minDelta) % 3
    }

    fun openHuman() = SystemCommand.instant("open human") {
        val currentStep = step
        val desiredStep = 2

        val noneSteps = colors.mapIndexed { index, color -> if (color == BallColor.None) index else -1 }.filter { it != -1 }
        if (noneSteps.isEmpty()) return@instant

        val deltas = noneSteps.map { idx -> idx to ((desiredStep - idx + 3) % 3) }
        val (_, minDelta) = deltas.minBy { it.second }
        step = (currentStep + minDelta) % 3
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
}