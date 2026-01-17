package org.firstinspires.ftc.teamcode.mechanisms

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
import org.firstinspires.ftc.teamcode.control.MotionProfile
import org.firstinspires.ftc.teamcode.control.MotionProfile.MotionResult
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.drivers.OctoQuadFWv3
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.seconds
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.withSign
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.TimeMark
import kotlin.time.TimeSource

class Sorter(hardwareMap: HardwareMap, kicker: Kicker? = null, telemetry: Telemetry? = null, octoQuad: OctoQuad): System(), Controllable<BaseProfile> {
    override val name: String = "Sorter"
    override val dependencies = listOf(octoQuad)

    companion object {
        val jerk = 15000
        val maxAccel = 8690.9
        val maxVel =  2680.3023

        private const val TICKS_PER_REV = 8192.0
        private const val HOME_ANGLE_DEG = 20.0

        private fun wrap360(deg: Double): Double {
            val m = deg % 360.0
            return if (m < 0) m + 360.0 else m
        }

        /** Signed smallest angular difference (deg) in [-180, 180]. */
        private fun angleErrorDeg(targetDeg: Double, currentDeg: Double): Double {
            return -Math.toDegrees(
                atan2(
                    sin(Math.toRadians(targetDeg - currentDeg)),
                    cos(Math.toRadians(targetDeg - currentDeg))
                )
            )
        }
    }

    var isOutput: Boolean = false
    var step: Int = 0
    var angle: Double = 0.0
    private var angleUnwrapped: Double = 0.0
    private var targetUnwrapped: Double = 0.0  // PID tracks this against angleUnwrapped
    var angleVel: Double = 0.0
    var maxVel: Double = 0.0
    var maxAccel: Double = 0.0
    var target: Double = 0.0
    private var lastLimitPressed: Boolean = false
    @Editable var debugColorPrints: Boolean = false
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
    val pid: PID = PID(0.3,2.0,0.0, 0.0, -180.0, 180.0, -1.0, 1.0, 20.milliseconds) // test
    @Editable var kS: Double = 0.01
    @Editable var kV: Double = 0.0007
    @Editable var kA: Double = 0.0
    @Editable
    val motionProfile: MotionProfile = MotionProfile(jerk,
        Companion.maxAccel,
        Companion.maxVel, 12.12)
    var currentProfile: MotionResult? = null
    var startTime: TimeMark? = null

    var profileVelocity = 0.0
    var profileAcceleration = 0.0
    var junkTicks = octoQuad.encoderData.position[5]
    private var profileDirection: Double = 1.0

    @Editable
    var offset = 80

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

//        octoQuad.octoQuad.channelBankConfig = OctoQuadFWv3.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH
//        octoQuad.octoQuad.setSingleChannelPulseWidthParams(5, 1, 1024)
    }

    override val update = SystemCommand.continuous("Sorter Update") {
        val oldAngleVel = angleVel

        if (!limitSwitch.state) {
            angle = 20.0
        } else {
            val deltaTicks = octoQuad.encoderData.position[5] - junkTicks
            val dt = it.deltaTime.seconds

            val deltaAngle = -(deltaTicks / TICKS_PER_REV) * 360.0
            angleUnwrapped += deltaAngle
            angle = wrap360(angleUnwrapped)
            angleVel = if (dt == 0.0) 0.0 else deltaAngle / dt
        }

        maxVel = max(angleVel, maxVel)
        maxAccel = max((angleVel - oldAngleVel)/it.deltaTime.seconds, maxAccel)

        telemetry?.addData("sorter max velocity", maxVel)
        telemetry?.addData("sorter max acceleration", maxAccel)

        val isGreen  = colorSensor.red() < 250 && colorSensor.green() > 200 && colorSensor.blue() < 260
        val isPurple = colorSensor.red() > 180 && colorSensor.green() < 400 && colorSensor.blue() > 300

        telemetry?.addData("red", colorSensor.red())
        telemetry?.addData("green", colorSensor.green())
        telemetry?.addData("blue", colorSensor.blue())


        telemetry?.addData("green?", isGreen)
        telemetry?.addData("purple?", isPurple)

//        if (isGreen)
//            colors[step] = BallColor.Green
//        else if (isPurple)
//            colors[step] = BallColor.Purple
//
//        if (kicker?.state?.kickedThisCycle ?: false) {
//            colors[(step + 2) % 3] = BallColor.None
//        }

        if (debugColorPrints) {
            if (isGreen) println("Green")
            else if (isPurple) println("Purple")
            else println("Nothing!!")
        }

        junkTicks = octoQuad.encoderData.position[5]
    }

    override val apply = SystemCommand.continuous("Sorter Apply") {
        val limitPressed = false

        if (limitPressed) {
            motor.ePower = 0.0
            telemetry?.addData("sorter homing", true)
            return@continuous
        }

        // Simple PID using unwrapped angles - no clamping needed
        // The target is set in move() to always be in the forward direction
        error = angleUnwrapped - targetUnwrapped
        var pidOutput = -pid.compute(error, 0.0)
        val voltage = hub.getVoltage()
        if (voltage < 12.0)
            pidOutput += ((1.0 - hub.getVoltage() / 12.0 + .2) * .25).withSign(pidOutput)

        if (abs(error) > 40)
            motor.ePower = pidOutput.coerceIn(-1.0, 1.0) + (.02 / (40 - 5) * abs(error) + .05 ).withSign(pidOutput)
        else if (abs(error) > 5)
            motor.ePower = pidOutput.coerceIn(-1.0, 1.0) + .05.withSign(pidOutput)
        else
            motor.ePower = pidOutput.coerceIn(-1.0, 1.0) + .04.withSign(pidOutput)


        telemetry?.addData("step", step)
        telemetry?.addData("ticks from encoder", hub.getEncoderTicks(3))
        telemetry?.addData("angle", angle)
        telemetry?.addData("target", target)
        telemetry?.addData("error", error)
        telemetry?.addData("power", motor.power)
    }

    fun outputPositionToggle() = SystemCommand.instant("toggle output positions") {
        isOutput = !isOutput
        move()
    }

    fun move() {
        when (step) {
            0 -> target = 0.0
            1 -> target = 120.0
            2 -> target = 240.0
        }
        if (isOutput) {
            target -= offset // ball in intake position flips to output position
        }
        
        // Set targetUnwrapped so we always go backward (increasing angle) for big moves
        // angleErrorDeg: negative = target ahead (need to go backward/increase), positive = target behind (need to go forward/decrease)
        val shortestError = angleErrorDeg(target, angle)
        if (shortestError < 50) {
            // Shortest path is backward (increasing) - use it
            targetUnwrapped = angleUnwrapped - shortestError
        } else {
            // Shortest path is forward - go the long way backward instead
            targetUnwrapped = angleUnwrapped + (360.0 - shortestError)
        }
        
        pid.reset()
    }

    fun moveForward() = SystemCommand.instant("sorter increment") {
        step = (step + 1) % 3
        move()
    }

    fun moveBackward() = SystemCommand.instant("sorter decrement") {
        step = (step + 2) % 3
        move()
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