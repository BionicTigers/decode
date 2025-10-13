package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.BaseCommandState
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.commands.Scheduler
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Analog
import io.github.bionictigers.axiom.core.web.Server
import io.github.bionictigers.axiom.core.web.WebData
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.MotionProfile
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.motion.Odometry
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Matrix
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.compareTo
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.log2
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds
import kotlin.time.TimeMark
import kotlin.time.TimeSource
import kotlin.unaryMinus

class Drivetrain(hardwareMap: HardwareMap, val telemetry: Telemetry?, val odometry: Odometry? = null) : System(), Controllable<BaseProfile> {
    enum class DriveOrientation {
        /** Movement is relative to the robot */
        ROBOT,
        /** Movement is relative to the field, direction is constant */
        FIELD
    }

    interface Schema : ControlSchema {
        val orientation: DriveOrientation
        val x: Analog
        val y: Analog
        val rot: Analog
    }

    override val name = "Drivetrain"

    companion object {
        val xJerk = 6000.0
        val yJerk = 3000.0
        val angularJerk = Angle.degrees(900.0)
        val xMaxAcceleration = 3509.0
        val yMaxAcceleration = 3569.0/4.0
        val angularMaxAcceleration = Angle.degrees(900.0) //130
        val xMaxVelocity = 1003.0
        val yMaxVelocity = 1812.0
        val angularMaxVelocity = Angle.degrees(310.0)

        val xProfile = MotionProfile(xJerk, xMaxAcceleration, xMaxVelocity)
        val yProfile = MotionProfile(yJerk, yMaxAcceleration, yMaxVelocity)
        val angularProfile = MotionProfile(angularJerk.radians, angularMaxAcceleration.radians, angularMaxVelocity.radians)

        val K: Matrix = Matrix(
            arrayOf(
            arrayOf(0.15638948, 0.15638948, -0.05931547, 0.10198206, 0.10198206, 0.00310677),
            arrayOf(-0.15638948, 0.15638948, 0.05931547, -0.10198206, 0.10198206, -0.00310677),
            arrayOf(-0.15638948, 0.15638948, -0.05931547, -0.10198206, 0.10198206, 0.00310677),
            arrayOf(0.15638948, 0.15638948, 0.05931547, 0.10198206, 0.10198206, -0.00310677)
            )
        )

        fun calculatePowers(x: Double, y: Double, rot: Double): List<Double> {
//            val direction = atan2(y, x)

            val fL = y - x + rot * 4
            val fR = y + x - rot * 4
            val bL = y + x + rot * 4
            val bR = y - x - rot * 4

            return listOf(-fL, -bL, fR, bR)
        }

        fun fieldOriented(x: Double, y: Double, rot: Double) {

        }
    }

    private val motors = DriveMotors(hardwareMap)

    val data = DrivetrainData()

    init {
        motors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        Server.OnDrivetrainUpdate = { x, y, r ->
            println("$x, $y, $r")
            Scheduler.schedule(setXControl(x), setYControl(y), setRotControl(r))
        }
    }

    override val dependencies: List<System> = listOfNotNull(odometry)

    private val headingPID = PID(8.0, 0.0, 0.0, 0, -PI, PI, -1.0, 1.0)
    private var targetHeading = 0.0
    private var timeLetGo = TimeSource.Monotonic.markNow()
    private var rotation = 0.0

    override val beforeRun = SystemCommand.continuous("Motor Power Calculation", data) {
        if (it.isInTeleop) {
            if (odometry != null) {
                if (it.rotControl.absoluteValue > .05) {
                    rotation = it.rotControl
                    timeLetGo = TimeSource.Monotonic.markNow()
                    targetHeading = odometry.position.radians % (2 * PI)
                } else {
                    println("stop")
                    rotation = 0.0
//                } else {
//                    val shortestPath = atan2(
//                        sin(odometry.position.radians - targetHeading),
//                        cos(odometry.position.radians - targetHeading)
//                    )
//                    rotation = headingPID.compute(0.0, shortestPath) / PI
//                    println(rotation)
//                }
                }
            }
        }
    }

    var errorState: Matrix? = null

    override val afterRun = SystemCommand.continuous("Drivetrain Update", data) {
        if (it.isInTeleop) {
            val powers = calculatePowers(-it.xControl, -it.yControl, rotation)
            motors.setPower(powers)
        } else {
            if (errorState != null) {
                val controlMatrix = -K * errorState!!
                motors.setPower(controlMatrix)
            } else {
                stop()
            }
        }
    }

    fun moveToPosition(targetPose: Pose) = Command.create("Move to Position") {
        var xResult: MotionProfile.MotionResult? = null
        var yResult: MotionProfile.MotionResult? = null
        var angularResult: MotionProfile.MotionResult? = null
        enter {
            val currentPose = odometry!!.position
            xResult = xProfile.generate(currentPose.x, targetPose.x)
            yResult = yProfile.generate(currentPose.y, targetPose.y)
            angularResult = angularProfile.generate(currentPose.radians, targetPose.radians)
        }

        action {
            val currentPose = odometry!!.position
            val currentVelocity = odometry.velocity
            val time = it.enteredAt?.elapsedNow() ?: Duration.ZERO

            val x = currentPose.x - (xResult?.getPosition(time) ?: 0.0)
            val y = currentPose.y - (yResult?.getPosition(time) ?: 0.0)
            val rot = currentPose.radians - (angularResult?.getPosition(time) ?: 0.0)
            val vx = currentVelocity.x - (xResult?.getVelocity(time) ?: 0.0)
            val vy = currentVelocity.y - (yResult?.getVelocity(time) ?: 0.0)
            val w = currentVelocity.radians - (angularResult?.getVelocity(time) ?: 0.0)

            errorState = Matrix(
                arrayOf(
                    arrayOf(x, y, rot, vx, vy, w)
                )
            )

            if (currentPose.within(targetPose, Pose(10,10,5))) {
                stop()
            }

            telemetry?.addData("Target Pose", targetPose)
        }

        exit {
            errorState = null
        }
    }

    fun mtpNoProfile(targetPose: Pose) = Command.create("Move to Position") {
        action {
            val currentPose = odometry!!.position
            val error = currentPose - targetPose

            val x = error.x
            val y = error.y
            val rot = error.radians
            val vx = 0.0
            val vy = 0.0
            val w = 0.0

            errorState = Matrix(
                arrayOf(
                    arrayOf(x, y, rot, vx, vy, w)
                )
            )

            if (currentPose.within(targetPose, Pose(10, 10, 5))) {
                stop()
            }
        }

        exit {
            errorState = null
        }
    }

    fun calculatePowers(x: Double, y: Double, rotation: Double): List<Double> {
        val frontLeft = y - x + rotation
        val frontRight = y + x - rotation
        val backLeft = y + x + rotation
        val backRight = y - x - rotation

        val powers = listOf(-frontLeft, backLeft, frontRight, backRight)

        return powers.map { it }
    }

    fun stop() = motors.setPower(0.0, 0.0, 0.0, 0.0)

    fun setXControl(x: Double): Command<DrivetrainData> = SystemCommand.instant("Set X Control", data) { it.xControl = -x }

    fun setYControl(y: Double): Command<DrivetrainData> = SystemCommand.instant("Set Y Control", data) { it.yControl = -y }

    private val rotMulti: Double
        get() = 1 - (data.xControl.absoluteValue.coerceAtLeast(data.yControl.absoluteValue) * .6)

    fun setRotControl(rot: Double): Command<DrivetrainData> = SystemCommand.instant("Set Rot Control", data) { it.rotControl = -rot * rotMulti }

    override fun bindControls(
        profile: BaseProfile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ): Unit =
        with(profile.drivetrain) {
            if (!gamepad.matches(desiredGamepad)) return
            data.driveOrientation = orientation

            builder.register(x) { setXControl(it * x.modifier) }
            builder.register(y) { setYControl(it * y.modifier) }
            builder.register(rot) { setRotControl(abs(it).pow(1.5) * sign(it)) }
        }

    private data class DriveMotors(
        val frontLeft: DcMotorEx,
        val frontRight: DcMotorEx,
        val backLeft: DcMotorEx,
        val backRight: DcMotorEx
    ): Iterable<DcMotorEx> {
        constructor(hardwareMap: HardwareMap) : this(
            hardwareMap.getByName<DcMotorEx>("frontLeft"),
            hardwareMap.getByName<DcMotorEx>("frontRight"),
            hardwareMap.getByName<DcMotorEx>("backLeft"),
            hardwareMap.getByName<DcMotorEx>("backRight")
        )

        fun setPower(frontLeft: Double, backLeft: Double, frontRight: Double, backRight: Double) {
            val ratio = maxOf(abs(frontLeft), abs(backLeft), abs(frontRight), abs(backRight), 1.0)

            if (frontLeft != this.frontLeft.power) this.frontLeft.power = frontLeft / ratio
            if (backLeft != this.backLeft.power) this.backLeft.power = backLeft / ratio
            if (frontRight != this.frontRight.power) this.frontRight.power = frontRight / ratio
            if (backRight != this.backRight.power) this.backRight.power = backRight / ratio

            WebData.setDrivetrain(frontLeft, -frontRight, backLeft, -backRight)
        }

        fun setPower(powers: List<Double>) {
            setPower(powers[0], powers[1], powers[2], powers[3])
        }

        fun setPower(matrix: Matrix) {
            setPower(matrix[0, 0], matrix[0, 1], matrix[0, 2], matrix[0, 3])
        }

        override operator fun iterator() = listOf(frontLeft, frontRight, backLeft, backRight).iterator()
    }

    data class DrivetrainData(
        /* We only need to set these if we are using driver control */
        var driveOrientation: DriveOrientation? = null,
        var xControl: Double = 0.0,
        var yControl: Double = 0.0,
        var rotControl: Double = 0.0,
    ) : BaseCommandState() { val isInTeleop get() = driveOrientation != null }
}