package org.firstinspires.ftc.teamcode.mechanisms

import android.view.MotionEvent
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Digital
import io.github.bionictigers.axiom.core.web.Editable
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.DynamicPID
import org.firstinspires.ftc.teamcode.control.GainSchedule
import org.firstinspires.ftc.teamcode.control.MotionProfile
import org.firstinspires.ftc.teamcode.control.MotionProfile.MotionResult
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.seconds
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.sin
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.TimeMark
import kotlin.time.TimeSource

class Sorter(hardwareMap: HardwareMap, kicker: Kicker? = null, telemetry: Telemetry? = null): System(), Controllable<BaseProfile> {
    override val name: String = "Sorter"

    companion object {
        val jerk = 3730
        val maxAccel = 373070.9
        val maxVel =  2680.3023
    }

    var isOutput: Boolean = false
    var step: Int = 0
    var angle: Double = 0.0
    var angleVel: Double = 0.0
    var maxVel: Double = 0.0
    var maxAccel: Double = 0.0
    var target: Double = 0.0
    var ticks: Double = 0.0
//    val pid: DynamicPID = DynamicPID(GainSchedule(
//        mapOf(
//            0.0 to .8, //8.0,
//            90.0 to 1.0, //1.5,
//            180.0 to 2.0, //.4, //2, 1.5, .75
//        ),
//        mapOf(0.0 to 0.0),
//        mapOf(0.0 to 0.0)
//    ), 0.0, -180.0, 180.0, -1.0, 1.0, 20.milliseconds, { abs(it) })
    @Editable
    val pid: PID = PID(1.5,0.0,0.0, 0.0, -180.0, 180.0, -1.0, 1.0, 20.milliseconds) // test
    @Editable
    val motionProfile: MotionProfile = MotionProfile(jerk,
        Companion.maxAccel,
        Companion.maxVel, 12.12)
    var currentProfile: MotionResult? = null
    var startTime: TimeMark? = null

    var error = 0.0
    var mpTarget = 0.0

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
        val outputToggle: Digital?
//        val kick: Digital?
    }

    val motor = hardwareMap.getByName<DcMotorEx>("sorter")
    val hub = ControlHub(hardwareMap, "Control Hub")
    val colorSensor = hardwareMap.getByName<ColorSensor>("intakeColor")
    val limitSwitch = hardwareMap.getByName<DigitalChannel>("limitSwitch")

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.REVERSE
        hub.setJunkTicks(3)
        pid.reset()
    }

    override val update = SystemCommand.continuous("Sorter Update") {
        val oldAngle = angle
        val oldAngleVel = angleVel

        if (!limitSwitch.state) {
            angle = 20.0
        } else {
            hub.refreshBulkData()
            val deltaTicks = hub.getEncoderTicks(3)
            angle -= (deltaTicks / 8192.0) * 360.0
            if (angle < 0) {
                angle += 360.0
            }
            angle %= 360.0
            hub.setJunkTicks()
        }
        angleVel = (angle - oldAngle)/it.deltaTime.seconds

        maxVel = max(angleVel, maxVel)
        maxAccel = max((angleVel - oldAngleVel)/it.deltaTime.seconds, maxAccel)

        telemetry?.addData("sorter max velocity", maxVel)
        telemetry?.addData("sorter max acceleration", maxAccel)

        val isGreen = colorSensor.green() > 200
        val isPurple = colorSensor.red() > 100 && colorSensor.blue() > 100
        telemetry?.addData("green", isGreen)
        telemetry?.addData("purple", isPurple)

//        if (isGreen)
//            colors[step] = BallColor.Green
//        else if (isPurple)
//            colors[step] = BallColor.Purple
//
//        if (kicker?.state?.kickedThisCycle ?: false) {
//            colors[(step + 2) % 3] = BallColor.None
//        }

        if (isGreen)
            println("Green")
        else if (isPurple)
            println("Purple")
        else
            println("Nothing!!")
    }

    override val apply = SystemCommand.continuous("Sorter Update") {

        if (currentProfile != null) {
            mpTarget = currentProfile!!.getPosition(startTime!!.elapsedNow())
            error = -Math.toDegrees(
                atan2(
                    sin(Math.toRadians(target - angle)),
                    cos(Math.toRadians(target - angle))
                )
            )
            motor.ePower = -pid.compute(
                error,
                mpTarget
            )
        }


        if (kicker?.reset ?: false) {
            if (!kicker.up)
                motor.ePower -= .35
            else
                motor.ePower -= .35
        }

        telemetry?.addData("step", step)
        telemetry?.addData("ticks from encoder", hub.getEncoderTicks(3))
        telemetry?.addData("angle", angle)
        telemetry?.addData("target", target)
        telemetry?.addData("power", motor.power)
    }

    fun outputPositionToggle() = SystemCommand.instant("toggle output positions") {
        isOutput = !isOutput
    }

    fun moveForward() = SystemCommand.instant("sorter increment") {
        step = (step + 1) % 3
        when (step) {
            0 -> target = 0.0
            1 -> target = 120.0
            2 -> target = 240.0
        }
        if (isOutput) {
            target += 80 // check which position flips to output position
        }
        val error = -Math.toDegrees(
            atan2(
                sin(Math.toRadians(target - angle)),
                cos(Math.toRadians(target - angle))
            )
        )
        currentProfile = motionProfile.generate(error, 0)
        startTime = TimeSource.Monotonic.markNow()
        pid.reset()
    }

    fun moveBackward() = SystemCommand.instant("sorter decrement") {
        step = (step + 2) % 3
        when (step) {
            0 -> target = 0.0
            1 -> target = 120.0
            2 -> target = 240.0
        }
        if (isOutput) {
            target += 80 // check which position flips to output position
        }
        val error = -Math.toDegrees(
            atan2(
                sin(Math.toRadians(target - angle)),
                cos(Math.toRadians(target - angle))
            )
        )
        currentProfile = motionProfile.generate(error, 0)
        startTime = TimeSource.Monotonic.markNow()
        pid.reset()
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
            outputToggle?.let { builder.register(it) { outputPositionToggle() } }
        }
    }
}