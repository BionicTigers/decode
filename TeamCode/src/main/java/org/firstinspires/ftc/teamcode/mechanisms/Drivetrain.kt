package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
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
import io.github.bionictigers.axiom.core.web.Server.start
import io.github.bionictigers.axiom.core.web.WebData
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.MotionProfile
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.motion.Odometry
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Matrix
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.compareTo
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.floor
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
//        val xJerk = 60//00.0
//        val yJerk = 30//00.0
//        val angularJerk = Angle.degrees(50.0)//ngle.degrees(150.0)//Angle.degrees(900.0)
//        val xMaxAcceleration = 35//09.0
//        val yMaxAcceleration = 35//69.0/4.0
//        val angularMaxAcceleration = Angle.degrees(90.0) //900 //130
//        val xMaxVelocity = 10//03.0
//        val yMaxVelocity = 18//12.0
//        val angularMaxVelocity = Angle.degrees(30.0)//390
//
        val xJerk = 6000.0
        val yJerk = 3000.0
        val angularJerk = Angle.degrees(9000.0)//Angle.degrees(900.0)
        val xMaxAcceleration = 3509.0
        val yMaxAcceleration = 3569.0/4.0
        val angularMaxAcceleration = Angle.degrees(900.0) //900 //130
        val xMaxVelocity = 1003.0
        val yMaxVelocity = 1812.0
        val angularMaxVelocity = Angle.degrees(390.0)//390


        val xProfile = MotionProfile(xJerk, xMaxAcceleration, xMaxVelocity)
        val yProfile = MotionProfile(yJerk, yMaxAcceleration, yMaxVelocity)
        val angularProfile = MotionProfile(angularJerk.radians, angularMaxAcceleration.radians, angularMaxVelocity.radians)

        val K: Matrix = Matrix(
            arrayOf(
                arrayOf(0.011174535377565482, 0.011174535377562932, -0.045738378172064, 0.00982304939088537, 0.009823049390882863, -0.010866834518862686),
                arrayOf(-0.0111745353775661, 0.011174535377564292, 0.04573837817206454, -0.009823049390885843, 0.009823049390884474, 0.010866834518862698),
                arrayOf(-0.011174535377569088, 0.01117453537756327, -0.045738378172065046, -0.00982304939088793, 0.009823049390883335, -0.010866834518862717),
                arrayOf(0.011174535377568462, 0.011174535377563957, 0.04573837817206561, 0.009823049390887451, 0.009823049390884004, 0.010866834518862736),
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

    val motors = DriveMotors(hardwareMap)

    val data = DrivetrainData()

    init {
        motors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
//        motors.frontRight.direction = DcMotorSimple.Direction.REVERSE
//        motors.frontLeft.direction = DcMotorSimple.Direction.REVERSE
//        motors.backRight.direction = DcMotorSimple.Direction.REVERSE


//        Server.OnDrivetrainUpdate = { x, y, r ->
//            println("$x, $y, $r")
//            Scheduler.schedule(setXControl(x), setYControl(y), setRotControl(r))
//        }
    }

    override val dependencies: List<System> = listOfNotNull(odometry)

    private val headingPID = PID(8.0, 0.0, 0.0, 0, -PI, PI, -1.0, 1.0)
    private var targetHeading = 0.0
    private var timeLetGo = TimeSource.Monotonic.markNow()
    private var rotation = 0.0

    override val beforeRun = SystemCommand.create("Motor Power Calculation", data) {
        enter {
            if (it.isInTeleop) {
//                motors.frontLeft.direction = DcMotorSimple.Direction.REVERSE //these are for test bot
//                motors.frontRight.direction = DcMotorSimple.Direction.REVERSE
//                motors.backLeft.direction = DcMotorSimple.Direction.FORWARD
//                motors.backRight.direction = DcMotorSimple.Direction.REVERSE
            } else {
//                motors.frontLeft.direction = DcMotorSimple.Direction.REVERSE
//                motors.frontRight.direction = DcMotorSimple.Direction.FORWARD
//                motors.backLeft.direction = DcMotorSimple.Direction.REVERSE
//                motors.backRight.direction = DcMotorSimple.Direction.FORWARD
//                motors.frontLeft.direction = DcMotorSimple.Direction.REVERSE
//                motors.frontRight.direction = DcMotorSimple.Direction.FORWARD
//                motors.backLeft.direction = DcMotorSimple.Direction.FORWARD
//                motors.backRight.direction = DcMotorSimple.Direction.FORWARD
                motors.frontLeft.direction = DcMotorSimple.Direction.FORWARD
                motors.frontRight.direction = DcMotorSimple.Direction.REVERSE
                motors.backLeft.direction = DcMotorSimple.Direction.REVERSE
                motors.backRight.direction = DcMotorSimple.Direction.REVERSE
            }
        }

        action {
            if (it.isInTeleop) {
                if (odometry != null) {
                    if (it.rotControl.absoluteValue > .05) {
                        rotation = it.rotControl
                        timeLetGo = TimeSource.Monotonic.markNow()
                        targetHeading = odometry.position.radians % (2 * PI)
                    } else {
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
    }

    var errorState: Matrix? = null

    fun Double.toHundredths(): Double {
        return floor(this * 100) / 100
    }

    fun List<Double>.printSimple(): String {
        var string = ""
        this.forEach {
            string += it.toHundredths().toString() + ", "
        }
        return string
    }

    operator fun List<Double>.times(num: Double): List<Double> {
        val result = mutableListOf<Double>()
        this.forEach {
            result.add(it * num)
        }
        return result
    }

    override val afterRun = SystemCommand.continuous("Drivetrain Update", data) {
        if (it.isInTeleop) {
            val powers = calculatePowers(-it.xControl, -it.yControl, rotation)
            telemetry?.addData("Powers", powers.printSimple())
            motors.setPower(powers)

            println(odometry!!.position.degrees.toString())
            println(odometry.position.x.toString())
//            println(odometry.position.degrees.toString() + ", " + odometry.position.y.toString())
        } else {
            if (errorState != null) {
                val controlMatrix = (-K * errorState!!)
                telemetry?.addData("Error", errorState!!.printSimple())
                telemetry?.addData("Control Matrix", controlMatrix.printSimple())
                motors.setPower(controlMatrix.scalar(.4))
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
            // TODO: Denormalize target angle before generating profile
            angularResult = angularProfile.generate(currentPose.degrees, targetPose.degrees)
        }

        action {
            val currentPose = odometry!!.position
            val currentVelocity = odometry.velocity
            val time = it.enteredAt?.elapsedNow() ?: Duration.ZERO

            val x = currentPose.x - (xResult?.getPosition(time) ?: 0.0)
            val y = currentPose.y - (yResult?.getPosition(time) ?: 0.0)
            val rot = currentPose.degrees - (angularResult?.getPosition(time) ?: 0.0)
            val vx = currentVelocity.x - (xResult?.getVelocity(time) ?: 0.0)
            val vy = currentVelocity.y - (yResult?.getVelocity(time) ?: 0.0)
            val w = currentVelocity.degrees - (angularResult?.getVelocity(time) ?: 0.0)

            errorState = Matrix(
                arrayOf(
                    arrayOf(x), arrayOf(y), arrayOf(rot), arrayOf(vx), arrayOf(vy), arrayOf(w)
                )
            )

            if (currentPose.within(targetPose, Pose(1,1,5))) {
                stop()
            }

//            telemetry?.addData("Current Pose", currentPose)
//            telemetry?.addData("Target Pose", targetPose)
//            telemetry?.addData("Error State", errorState)
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
            val rot = error.degrees
            val vx = 0.0
            val vy = 0.0
            val w = 0.0

            errorState = Matrix(
                arrayOf(
                    arrayOf(x), arrayOf(y), arrayOf(rot), arrayOf(vx), arrayOf(vy), arrayOf(w)
                )
            )

            if (currentPose.within(targetPose, Pose(1, 1, 5))) {
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

    data class DriveMotors(
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

            if (frontLeft != this.frontLeft.power) this.frontLeft.ePower = frontLeft / ratio
            if (backLeft != this.backLeft.power) this.backLeft.ePower = backLeft / ratio
            if (frontRight != this.frontRight.power) this.frontRight.ePower = frontRight / ratio
            if (backRight != this.backRight.power) this.backRight.ePower = backRight / ratio

            WebData.setDrivetrain(frontLeft, -frontRight, backLeft, -backRight)
        }

        fun setPower(power: Double) {
            setPower(power, power, power, power)

        }

        fun setPower(powers: List<Double>) {
            setPower(powers[0], powers[1], powers[2], powers[3])
        }

        fun setPower(matrix: Matrix) {
            setPower(matrix[0, 0], matrix[1, 0], matrix[2, 0], matrix[3, 0])
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