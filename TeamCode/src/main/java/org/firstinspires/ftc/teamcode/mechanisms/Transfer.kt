package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
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
import io.github.bionictigers.axiom.core.utils.Timer
import io.github.bionictigers.axiom.core.web.Editable
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.MotionProfile
import org.firstinspires.ftc.teamcode.control.MotionResult
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.control.generateMotionProfile
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.SensorColor
import org.firstinspires.ftc.teamcode.utils.degrees
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.abs
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.TimeMark
import kotlin.time.measureTime

class Transfer(
    hardwareMap: HardwareMap,
    val octoQuad: OctoQuad,
    telemetry: Telemetry? = null,
) : System(), Controllable<BaseProfile> {
    override val name: String = "Sorter"
    override val dependencies = listOf(octoQuad)

    enum class BallColor {
        Green,
        Purple,
        None
    }

    interface Schema : ControlSchema {
        val sort: Digital?
        val shoot: Digital?
    }

    private val balls = MutableList(3) { BallColor.None }
    val occupiedBays: Int
        get() = balls.count { it != BallColor.None }

    val shootingTime = 2000.milliseconds
    val intakeAngle = Angle.degrees(65.0)
    private val currentAngle: Angle
        get() = Angle.degrees((octoQuad.encoderData.position[4] / 1024.0 * 360.0))

    @Editable
    private val targetPid = PID(1.25, 0.0, 0.0, 0.0, -30.0, 360.0, -.45, .45)

    @Editable
    var reverseCorrectionWindowDegrees = 90.0

    @Editable
    var ballCaptureErrorDegrees = 12.0

    @Editable
    var greenMinColor = SensorColor(0.1, 0.75, 0.7)

    @Editable
    var greenMaxColor = SensorColor(0.5, 1.0, 0.85)

    @Editable
    var purpleMinColor = SensorColor(0.4, 0.5, 0.8)

    @Editable
    var purpleMaxColor = SensorColor(.65, 0.825, 1.0)

    private val greenReferenceColor = SensorColor(0.25, 1.0, 0.2)
    private val purpleReferenceColor = SensorColor(0.75, 0.2, 1.0)

    // Hardware
    private val motor = hardwareMap.getByName<DcMotorEx>("sorter")
    private val hub = ControlHub(hardwareMap, "Control Hub")
    private val colorSensor = hardwareMap.getByName<ColorSensor>("intakeColor")

    private val motionGenerator = MotionProfile(25000, 10000, 500)
    private var motionProfile: MotionResult? = null
    private var profiledStartedAt: TimeMark? = null

    var isShooting = false
    var shootingAt: TimeMark? = null
    var targetBay = 0
    var targetAngle = intakeAngle

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.REVERSE
    }

    val timer = Timer(.30.seconds)
    var color = SensorColor(0.0, 0.0, 0.0)
    var ballColor = BallColor.None
    override val update = SystemCommand.continuous("Sorter Telemetry") {
        if (motionProfile == null) {
            sort()
        }

        telemetry?.addData("Current Angle", currentAngle.degrees)
        telemetry?.addData("Target Angle", targetAngle.degrees)
        telemetry?.addData("Target Bay", targetBay)
        telemetry?.addData("Is Shooting", isShooting)
        telemetry?.addData("Power", motor.power)

        telemetry?.addData("Intake Color", color)
        telemetry?.addData("Detected Ball Color", ballColor)
        telemetry?.addData("Stored Balls", balls.joinToString())
        if (profiledStartedAt != null && motionProfile != null) {
            telemetry?.addData("Profiled Target Error", motionProfile!!.getPosition(profiledStartedAt!!.elapsedNow()))
            telemetry?.addData("Actual Error", getError(targetAngle, currentAngle))
        }

        timer.update(it).finished {
                color = SensorColor.fromSensor(colorSensor).normalized()
            val detectedBallColor = getBallColor(color)
            val isNewColor =
                detectedBallColor != BallColor.None && detectedBallColor != ballColor

            ballColor = detectedBallColor

            if (isNewColor && abs(
                    getError(
                        targetAngle,
                        currentAngle
                    )
                ) <= ballCaptureErrorDegrees
            ) {
                storeBallInCurrentBay(detectedBallColor)
                findNextFreeBay()?.let { nextFreeBay ->
                    moveToBay(nextFreeBay, it.lastExecutedAt)
                }
            }
        }
    }

    private fun getBallColor(color: SensorColor): BallColor {
        val isGreen = color.isInRange(greenMinColor, greenMaxColor, normalize = false)
        val isPurple = color.isInRange(purpleMinColor, purpleMaxColor, normalize = false)

        return when {
            isGreen && isPurple -> {
                if (color.distanceTo(greenReferenceColor) <= color.distanceTo(purpleReferenceColor)) {
                    BallColor.Green
                } else {
                    BallColor.Purple
                }
            }

            isGreen -> BallColor.Green
            isPurple -> BallColor.Purple
            else -> BallColor.None
        }
    }

    private fun storeBallInCurrentBay(ballColor: BallColor) {
        balls[targetBay] = ballColor
    }

    private fun findNextFreeBay(): Int? {
        for (offset in 1..balls.size) {
            val bay = (targetBay + offset) % balls.size
            if (balls[bay] == BallColor.None) return bay
        }

        return null
    }

    private fun moveToBay(bay: Int, executedAt: TimeMark?) {
        targetBay = bay
        targetAngle = getBayAngle(targetBay)
        motionProfile = motionGenerator.generate(getError(targetAngle, currentAngle), 0.0)
        profiledStartedAt = executedAt
    }

    private fun moveToAngleFromBay(angle: Angle, bay: Int, executedAt: TimeMark?) {
        targetBay = bay
        targetAngle = getBayAngle(targetBay) + angle
        motionProfile = motionGenerator.generate(getError(targetAngle, currentAngle), 0.0)
        profiledStartedAt = executedAt
    }

    private fun clearBalls() {
        for (index in balls.indices) {
            balls[index] = BallColor.None
        }
    }

    fun getError(target: Angle, current: Angle): Double {
        val forwardError = (target.degrees - current.degrees + 360.0) % 360.0
        val shortestSignedError = if (forwardError > 180.0) forwardError - 360.0 else forwardError
        return if (abs(shortestSignedError) <= reverseCorrectionWindowDegrees) shortestSignedError else forwardError
    }

    override val apply = SystemCommand.continuous("Sorter Control") {
        if (isShooting) {
            if (shootingAt != null && shootingAt!!.elapsedNow() > shootingTime) {
                isShooting = false
                sort()
            }

            motor.ePower = -1.0
        } else {
            val realTargetError = if (profiledStartedAt != null && motionProfile != null) {
                motionProfile!!.getPosition(profiledStartedAt!!.elapsedNow())
            } else {
                0.0
            }

            val currentError = getError(targetAngle, currentAngle)
            motor.ePower = -targetPid.compute(currentError, realTargetError)
        }
    }

    fun getBayAngle(bay: Int) = intakeAngle + Angle.degrees(bay * 120.0)

    fun sort() = SystemCommand.instant {
        isShooting = false
        moveToBay((targetBay + 1) % balls.size, it.lastExecutedAt)
    }

    val angleFromIntakeToPreShoot = Angle.degrees(-10.0)
    fun shootPrep() = SystemCommand.instant {
        isShooting = false
        moveToAngleFromBay(angleFromIntakeToPreShoot, targetBay, it.lastExecutedAt)
    }

    fun shootSingle() = SystemCommand.instant {
        isShooting = true
        shootingAt = it.lastExecutedAt
    }

    fun shoot() = SystemCommand.instant {
        isShooting = true
        shootingAt = it.lastExecutedAt
        clearBalls()
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.sorter) {
            if (!desiredGamepad.matches(gamepad)) return

            sort?.let { builder.register(it) { sort() } }
            shoot?.let { builder.register(it) { shoot() } }
        }
    }
}
