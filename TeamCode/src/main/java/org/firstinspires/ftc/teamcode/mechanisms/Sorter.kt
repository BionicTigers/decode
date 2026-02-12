package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.TouchSensor
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Digital
import io.github.bionictigers.axiom.core.web.Editable
import io.github.bionictigers.axiom.core.web.Hidden
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.MotionProfile
import org.firstinspires.ftc.teamcode.control.MotionResult
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.seconds
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.withSign
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.TimeMark

class Sorter(
    hardwareMap: HardwareMap,
    kicker: Kicker? = null,
    telemetry: Telemetry? = null,
    octoQuad: OctoQuad
) : System(), Controllable<BaseProfile> {
    override val name: String = "Sorter"
    override val dependencies = listOf(octoQuad)

    companion object {
        val jerk = listOf(
            15000,
            13000,
            11000,
            9000
        )
        val maxAccel = listOf(
            8690.9,
            8090.9,
            7490.9,
            7190.9,
        )
        val maxVel = listOf(
            2680.3023,
            2280.3023,
            1700.3023,
            1500.3023,
        )

        private const val TICKS_PER_REV = 8192.0

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

    enum class BallColor {
        Green,
        Purple,
        None
    }

    /**
     * Physical positions around the carousel relative to robot hardware.
     * These stay fixed while `step` changes as the carousel rotates.
     */
    enum class SlotPosition {
        Intake,
        Middle,
        Output
    }

    interface Schema : ControlSchema {
        val forward: Digital?
        val backward: Digital?
        val green: Digital?
        val purple: Digital?
        val openIntake: Digital?
        val openHuman: Digital?
        val outputToggle: Digital?
    }

    // Hardware
    private val motor = hardwareMap.getByName<DcMotorEx>("sorter")
    private val hub = ControlHub(hardwareMap, "Control Hub")
    private val colorSensor = hardwareMap.getByName<ColorSensor>("intakeColor")
    private val limitSwitch = hardwareMap.getByName<TouchSensor>("limitSwitch")

    // Pose + controller state
    var isOutput: Boolean = false
    var step: Int = 0
    var angle: Double = 0.0
    private var angleUnwrapped: Double = 0.0
    private var targetUnwrapped: Double = 0.0
    var angleVel: Double = 0.0
    @Hidden
    var maxVel: Double = 0.0
    @Hidden
    var maxAccel: Double = 0.0
    var target: Double = 0.0
    var error = 0.0


    // Runtime buffers
    private var junkTicks = octoQuad.encoderData.position[5]
    val colors: MutableList<BallColor> = MutableList(3) { BallColor.None }

    @Editable var kG = 0.067
    @Editable
    val pid: PID = PID(1.3, 0.0, 0.0, 0.0, -180.0, 180.0, -1.0, 1.0, 20.milliseconds)
//    @Editable var kS: Double = 0.01
//    @Editable var kV: Double = 0.0007
//    @Editable var kA: Double = 0.0
    @Editable
    val motionProfile: MotionProfile = MotionProfile(
        jerk[0],
        Companion.maxAccel[0],
        Companion.maxVel[0],
        12.12
    )
    @Editable
    @Hidden
    var offset = -40.0
    @Hidden
    var currentProfile: MotionResult? = null
    @Hidden
    var startTime: TimeMark? = null
    @Hidden
    var profileVelocity = 0.0
    @Hidden
    var profileAcceleration = 0.0
    @Hidden
    var mpTarget = 0.0

    // Gravity Feed Forward
    var ffg = 0.0
    private var lastSeenIntakeColor: BallColor = BallColor.None

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.FORWARD
        hub.setJunkTicks(3)
        pid.reset()
    }

    override val update = SystemCommand.continuous("Sorter Update") {
        val oldAngleVel = angleVel
        val deltaTicks = octoQuad.encoderData.position[5] - junkTicks
        val dt = it.deltaTime.seconds
        val deltaAngle = -(deltaTicks / TICKS_PER_REV) * 360.0

        angleUnwrapped += deltaAngle
        angle = wrap360(angleUnwrapped)
        angleVel = if (dt == 0.0) 0.0 else deltaAngle / dt

        maxVel = max(angleVel, maxVel)
        maxAccel = max((angleVel - oldAngleVel) / it.deltaTime.seconds, maxAccel)

        telemetry?.addData("sorter max velocity", maxVel)
        telemetry?.addData("sorter max acceleration", maxAccel)

        val red = colorSensor.red()
        val green = colorSensor.green()
        val blue = colorSensor.blue()

        val isGreen = green > red && green > blue - 100 && green > 300
        val isPurple = blue > red && blue > green && blue > 300
        val detectedColor = when {
            isGreen -> BallColor.Green
            isPurple -> BallColor.Purple
            else -> BallColor.None
        }

        telemetry?.addData("red", red)
        telemetry?.addData("green", green)
        telemetry?.addData("blue", blue)
        telemetry?.addData("green?", isGreen)
        telemetry?.addData("purple?", isPurple)

        // Track which ball is currently in the transfer intake position.
        // We only latch on a rising edge (None -> Color) to avoid noise/flapping writes.
        if (detectedColor != BallColor.None && lastSeenIntakeColor == BallColor.None) {
            colors[slotIndexForPosition(SlotPosition.Intake)] = detectedColor
        }
        lastSeenIntakeColor = detectedColor

        // Remove the ball that just left through the kicker output.
        if (kicker?.kickedThisCycle == true) {
            colors[slotIndexForPosition(SlotPosition.Output)] = BallColor.None
        }

        ffg = abs(listOfNotNull(
            sin(Math.toRadians(angle - 30.45)).takeIf { colors[0] != BallColor.None },
            sin(Math.toRadians(angle - 30.45) + 2 * PI / 3).takeIf { colors[1] != BallColor.None },
            sin(Math.toRadians(angle - 30.45) + 4 * PI / 3).takeIf { colors[2] != BallColor.None },
        ).sum())

        motionProfile.setConstants(
            jerk[occupiedBays],
            Companion.maxAccel[occupiedBays],
            Companion.maxVel[occupiedBays]
        )

        junkTicks = octoQuad.encoderData.position[5]
    }

    override val apply = SystemCommand.continuous("Sorter Apply") {
        val limitPressed = false

        if (limitPressed) {
            motor.ePower = 0.0
            telemetry?.addData("sorter homing", true)
            return@continuous
        }

        error = angleUnwrapped - targetUnwrapped
        val pidOutput = pid.compute(error, 0.0)
        motor.ePower = (abs(pidOutput) + ffg * kG).withSign(pidOutput)

        telemetry?.addData("step", step)
        telemetry?.addData("ticks from encoder", hub.getEncoderTicks(3))
        telemetry?.addData("angle", angle)
        telemetry?.addData("target", target)
        telemetry?.addData("error", error)
        telemetry?.addData("power", motor.power)
        telemetry?.addData("intake slot color", getBallAtPosition(SlotPosition.Intake))
        telemetry?.addData("middle slot color", getBallAtPosition(SlotPosition.Middle))
        telemetry?.addData("output slot color", getBallAtPosition(SlotPosition.Output))
        telemetry?.addData(
            "transfer order (I>M>O)",
            "${colorShort(getBallAtPosition(SlotPosition.Intake))} > " +
                "${colorShort(getBallAtPosition(SlotPosition.Middle))} > " +
                colorShort(getBallAtPosition(SlotPosition.Output))
        )
    }

    fun outputPositionToggle() = SystemCommand.instant("toggle output positions") {
        isOutput = !isOutput
        move()
    }

    fun move() {
        target = when (step) {
            0 -> 0.0
            1 -> 120.0
            2 -> 240.0
            else -> 0.0
        }

        if (isOutput) {
            target -= offset
        }

        val shortestError = angleErrorDeg(target, angle)
        targetUnwrapped = angleUnwrapped - shortestError
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
        selectStepForColor(BallColor.Green)?.let { step = it }
    }

    fun targetPurple() = SystemCommand.instant("target purple") {
        selectStepForColor(BallColor.Purple)?.let { step = it }
    }

    fun openIntake() = SystemCommand.instant("open intake") {
        selectStepForOpen(desiredStep = 0)?.let { step = it }
    }

    fun openHuman() = SystemCommand.instant("open human") {
        selectStepForOpen(desiredStep = 2)?.let { step = it }
    }

    fun getBallAtPosition(position: SlotPosition): BallColor {
        val slotIndex = slotIndexForPosition(position)
        return colors[slotIndex]
    }

    fun hasBallAtPosition(position: SlotPosition): Boolean {
        return getBallAtPosition(position) != BallColor.None
    }

    fun getBallAtOffsetFromIntake(offset: Int): BallColor {
        val slotIndex = ((step + offset) % 3 + 3) % 3
        return colors[slotIndex]
    }

    fun hasBallAtOffsetFromIntake(offset: Int): Boolean {
        return getBallAtOffsetFromIntake(offset) != BallColor.None
    }

    fun hasBallAtIndex(slotIndex: Int): Boolean {
        val normalizedIndex = ((slotIndex % 3) + 3) % 3
        return colors[normalizedIndex] != BallColor.None
    }

    val occupiedBays: Int
        get() = colors.count { it != BallColor.None }

    private fun selectStepForColor(color: BallColor): Int? {
        val currentStep = step
        val desiredOutputStep = (currentStep + 2) % 3

        val colorSteps = colors.mapIndexedNotNull { index, ballColor ->
            index.takeIf { ballColor == color }
        }
        if (colorSteps.isEmpty()) return null

        val forwardDistances = colorSteps.map { targetIndex ->
            val forwardSteps = (targetIndex - currentStep + 3) % 3
            targetIndex to forwardSteps
        }

        val (nearestIndex, _) = forwardDistances.minBy { it.second }
        val deltaStepsToOutput = (desiredOutputStep - nearestIndex + 3) % 3
        return (currentStep + deltaStepsToOutput) % 3
    }

    private fun selectStepForOpen(desiredStep: Int): Int? {
        val currentStep = step

        val noneSteps = colors.mapIndexedNotNull { index, color ->
            index.takeIf { color == BallColor.None }
        }
        if (noneSteps.isEmpty()) return null

        val deltas = noneSteps.map { idx -> idx to (desiredStep - idx + 3) % 3 }
        val (_, minDelta) = deltas.minBy { it.second }
        return (currentStep + minDelta) % 3
    }

    private fun slotIndexForPosition(position: SlotPosition): Int {
        return when (position) {
            SlotPosition.Intake -> step
            SlotPosition.Middle -> (step + 1) % 3
            SlotPosition.Output -> (step + 2) % 3
        }
    }

    private fun colorShort(color: BallColor): String {
        return when (color) {
            BallColor.Green -> "G"
            BallColor.Purple -> "P"
            BallColor.None -> "_"
        }
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
