package org.firstinspires.ftc.teamcode.mechanisms

import io.github.bionictigers.axiom.core.commands.System
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Analog
import io.github.bionictigers.axiom.core.web.WebData
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.MotionProfile
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Matrix
import org.firstinspires.ftc.teamcode.utils.RollingAverage
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin
import kotlin.time.Duration
import kotlin.time.TimeSource

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
        // TODO: test values and note units
        val xJerk = 6000.0
        val yJerk = 3000.0
        val angularJerk = Angle.degrees(9000.0)//Angle.degrees(900.0)
        val xMaxAcceleration = 3509.0
        val yMaxAcceleration = 3569.0/4.0
        val angularMaxAcceleration = Angle.degrees(900.0) //900 //130
        val xMaxVelocity = 1003.0
        val yMaxVelocity = 1812.0
        val angularMaxVelocity = Angle.degrees(390.0)//390

        // TODO: find real values,
        // in ticks/second
        val wheelMaxVel = 350.0 * 2

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

        val testBotDirections = listOf(DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD)
        val autoDirections = listOf(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE)
    }

    /* We only need to set these if we are using driver control */
    var driveOrientation: DriveOrientation? = null
    var xControl: Double = 0.0
    var yControl: Double = 0.0
    var rotControl: Double = 0.0

    val isInTeleop get() = driveOrientation != null

    val motors = DriveMotors(hardwareMap)
    val pids = MotorPIDs()

    var controllerCommand: List<Double> = listOf(0.0, 0.0, 0.0, 0.0)
    var timeChanged = TimeSource.Monotonic.markNow()
    var powers: MotorValues<Double?> = MotorValues(0.0, 0.0, 0.0, 0.0)

    override val dependencies: List<System> = listOfNotNull(odometry)

    var max = 0.0

    var errorState: Matrix? = null

    private val headingPID = PID(8.0, 0.0, 0.0, 0, -PI, PI, -1.0, 1.0)
    private var targetHeading = 0.0
    private var timeLetGo = TimeSource.Monotonic.markNow()
    private var rotation = 0.0

    init {
        motors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    override val update = SystemCommand.create("Motor Power Calculation") {
        enter {
            if (isInTeleop) {
                motors.forEachIndexed { index, motor -> motor.direction = testBotDirections[index] }
            } else {
                motors.forEachIndexed { index, motor -> motor.direction = autoDirections[index] }
            }
        }

        action {
            pids.updateVelocities()
            if (isInTeleop) {
                if (odometry != null) {
                    if (rotControl.absoluteValue > .05) {
                        rotation = rotControl
                        timeLetGo = TimeSource.Monotonic.markNow()
                        targetHeading = odometry.position.radians % (2 * PI)
                    } else {
                        // TODO: fix heading PID logic
                        val shortestPath = atan2(
                            sin((odometry.position.radians % (2 * PI)) - targetHeading),
                            cos((odometry.position.radians % (2 * PI)) - targetHeading)
                        )
                        rotation = headingPID.compute(0.0, shortestPath) / PI
                    }
                }
                        rotation = rotControl
//                val newCommand = calculatePowers(-it.xControl, -it.yControl, rotation)
//
//                if (largeChange(newCommand, controllerCommand)) {
//                    powers = calculatePowers(-it.xControl, -it.yControl, rotation)
//                    timeChanged = TimeSource.Monotonic.markNow()
//                } else if (TimeSource.Monotonic.markNow() - timeChanged < 250.milliseconds) {
//                    powers = calculatePowers(-it.xControl, -it.yControl, rotation)
//                } else {
//                    powers = pids.calculatePowers(-it.xControl, -it.yControl, rotation)
//                }
//
//                controllerCommand = newCommand

//                powers = pids.calculatePowers(-it.xControl, -it.yControl, rotation)

                if (calculateRawPowers(xControl, yControl, rotation).all { it == 0.0 } ) pids.reset()
                powers.set(pids.calculatePowers(xControl, yControl, rotation))

                telemetry?.addData("pid", pids)
                telemetry?.addData("powers", powers)
            } else {
                if (errorState != null) {
                    val target = (-K * errorState!!).scalar(.01)
                    telemetry?.addData("Error", errorState!!.printSimple())
                    telemetry?.addData("Target Velocities", target)
                    powers.set(pids.calculatePowers(target))
                } else {
                    powers.clear()
                }
            }
        }
    }

    fun largeChange(list:List<Double>, otherList: List<Double>): Boolean {
        list.forEachIndexed { index, num ->
            if (abs(num - otherList[index]) > .1){
                return true
            }
        }
        return false
    }

    fun findWheelMaxVelocity() {
        with(pids) {
            val newMax = velocities.min()
            max = maxOf(max, newMax)
            telemetry?.addData("Max Velocity", "$max, from ${motorNames[velocities.withIndex().minBy { it.value }.index]}")
        }
    }

    override val apply = SystemCommand.continuous("Drivetrain Update") {
        if (isInTeleop) {
            if (!powers.areNull) {
                motors.setPower(powers)
            }
        } else {
            if (!powers.areNull) {
                motors.setPower(powers)
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

            val targetAngle = denormalizeTargetAngle(targetPose.rotation)
            angularResult = angularProfile.generate(0.0, targetAngle.degrees)
        }

        action {
            val currentPose = odometry!!.position
            val currentVelocity = odometry.velocity
            val time = it.enteredAt.elapsedNow()

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

    fun calculateRawPowers(x: Double, y: Double, rotation: Double): List<Double> {
        val frontLeft = y - x + rotation
        val frontRight = y + x + rotation
        val backLeft = y + x - rotation
        val backRight = y - x - rotation

        val powers = listOf(frontLeft, frontRight, backLeft, backRight)

        return powers.map { it }
    }

    fun denormalizeTargetAngle(target: Angle) : Angle {
        return Angle.degrees(
            atan2(
                sin(odometry!!.position.degrees.normalizeDegrees() - target.degrees),
                cos(odometry.position.degrees.normalizeDegrees() - target.degrees)
            )
        )
    }

    fun stop() = motors.setPower(0.0, 0.0, 0.0, 0.0)

    fun setXControl(x: Double) = SystemCommand.instant("Set X Control") { xControl = -x }

    fun setYControl(y: Double) = SystemCommand.instant("Set Y Control") { yControl = -y }

    private val rotMulti: Double
        get() = 1 - (xControl.absoluteValue.coerceAtLeast(yControl.absoluteValue) * .6)

    fun setRotControl(rot: Double) = SystemCommand.instant("Set Rot Control") { rotControl = -rot * rotMulti }

    override fun bindControls(
        profile: BaseProfile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ): Unit =
        with(profile.drivetrain) {
            if (!gamepad.matches(desiredGamepad)) return
            driveOrientation = orientation

            builder.register(x) { setXControl(it * x.modifier) }
            builder.register(y) { setYControl(it * y.modifier) }
            builder.register(rot) { setRotControl(abs(it).pow(1.5) * sign(it)) }
        }

    inner class DriveMotors(
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

        fun setPower(frontLeft: Double, frontRight: Double, backLeft: Double, backRight: Double) {
            val ratio = maxOf(abs(frontLeft), abs(frontRight), abs(backLeft), abs(backRight), 1.0)

            if (frontLeft != this.frontLeft.power) this.frontLeft.ePower = frontLeft / ratio
            if (frontRight != this.frontRight.power) this.frontRight.ePower = frontRight / ratio
            if (backLeft != this.backLeft.power) this.backLeft.ePower = backLeft / ratio
            if (backRight != this.backRight.power) this.backRight.ePower = backRight / ratio

            WebData.setDrivetrain(frontLeft, frontRight, backLeft, backRight)
        }

        fun setPower(power: Double) { setPower(power, power, power, power) }

        fun setPower(powers: List<Double>) { setPower(powers[0], powers[1], powers[2], powers[3]) }

        fun setPower(matrix: Matrix) { setPower(matrix[0, 0], matrix[1, 0], matrix[2, 0], matrix[3, 0]) }

        fun setPower(powers: MotorValues<Double?>) {
            if (!powers.areNull) setPower(powers[0]!!, powers[1]!!, powers[2]!!, powers[3]!!)
        }

        override operator fun iterator() = listOf(frontLeft, frontRight, backLeft, backRight).iterator()

        operator fun get(index: Int) = listOf(frontLeft, frontRight, backLeft, backRight)[index]
    }

    inner class MotorPIDs() {
        // TODO: tune wheel velocity PIDs
        val pids = MotorValues(
            PID(1.0, 100.0, 0.0, 0, -wheelMaxVel, wheelMaxVel, -1.0, 1.0),
            PID(1.0, 100.0, 0.0, 0, -wheelMaxVel, wheelMaxVel, -1.0, 1.0),
            PID(1.0, 100.0, 0.0, 0, -wheelMaxVel, wheelMaxVel, -1.0, 1.0),
            PID(1.0, 100.0, 0.0, 0, -wheelMaxVel, wheelMaxVel, -1.0, 1.0)
        )

        val setPoints = MotorValues<Double?>(null, null, null, null)

        val velocities = MotorValues(0.0, 0.0, 0.0, 0.0)

        val rawPowers = MotorValues(0.0, 0.0, 0.0, 0.0)

        val modifiers = MotorValues(0.0, 0.0, 0.0, 0.0)

        val rollingAverages = MotorValues(
            RollingAverage(3),
            RollingAverage(3),
            RollingAverage(3),
            RollingAverage(3)
        )

        fun updateVelocities() {
            rollingAverages.forEachIndexed { index, it ->
                it += motors[index].velocity
                velocities[index] = it.average
            }
        }

        fun calculatePowers(x: Double, y: Double, rotation: Double) : List<Double> {
            rawPowers.set(calculateRawPowers(x, y, rotation))
            modifiers.set(calculateModifiers(x, y, rotation))
            return rawPowers.zip(modifiers) { power, mod -> power + mod }
        }

        fun calculatePowers(matrix: Matrix) : List<Double> {
            rawPowers.set(listOf(matrix[0, 0], matrix[1, 0], matrix[2, 0], matrix[3, 0]))
            modifiers.set(calculateModifiers(matrix))
            return rawPowers.zip(modifiers) { power, mod -> power + mod }
        }

        fun calculateModifiers(x: Double, y: Double, rotation: Double) : List<Double> {
            setTargets(x, y, rotation)
            return calculateModifiers()
        }

        fun calculateModifiers(matrix: Matrix) : List<Double> {
            setTargets(matrix)
            return calculateModifiers()
        }

        fun calculateModifiers() : List<Double> = pids.mapIndexed { index, pid -> pid.compute(velocities[index], setPoints[index]?: 0.0) }

        fun setTargets(matrix: Matrix) {
            val frontLeft = matrix[0, 0]
            val frontRight = matrix[1, 0]
            val backLeft = matrix[2, 0]
            val backRight = matrix[3, 0]

            val maxSP = maxOf(abs(frontLeft), abs(backLeft), abs(frontRight), abs(backRight))
            setPoints.set(
                if (maxSP > wheelMaxVel) {
                    listOf(
                        frontLeft / maxSP * wheelMaxVel,
                        frontRight / maxSP * wheelMaxVel,
                        backLeft / maxSP * wheelMaxVel,
                        backRight / maxSP * wheelMaxVel
                    )
                } else {
                    listOf(frontLeft, frontRight, backLeft, backRight)
                }
            )
        }

        fun setTargets(x: Double, y: Double, rotation: Double) {
            val frontLeft = y - x + rotation
            val frontRight = y + x + rotation
            val backLeft = y + x - rotation
            val backRight = y - x - rotation

            val ratio = maxOf(abs(frontLeft), abs(backLeft), abs(frontRight), abs(backRight), 1.0)
            setPoints.set(
                listOf(
                    frontLeft / ratio * wheelMaxVel,
                    frontRight / ratio * wheelMaxVel,
                    backLeft / ratio * wheelMaxVel,
                    backRight / ratio * wheelMaxVel
                )
            )
        }

        fun clear() {
            setPoints.clear()
            reset()
        }

        fun reset() { pids.forEach { it.reset() } }

        override fun toString(): String {
            fun Double.toHundredths(): Double = floor(this * 100) / 100
            var string = "\n"
            if (!setPoints.areNull) {
                this.setPoints.forEachIndexed { index, it ->
                    string += "   ${motorNames[index]} | Set Point: ${it?.toHundredths()}, Current: ${velocities[index].toHundredths()}, Raw Power ${rawPowers[index].toHundredths()}, Modifier, ${modifiers[index].toHundredths()}"
                    if (index < 3) {
                        string += "\n"
                    }
                }
                return string
            }
            return "Not set"
        }
    }

    val motorNames = listOf("FL", "FR", "BL", "BR")
    inner class MotorValues<T>(
        var frontLeft: T,
        var frontRight: T,
        var backLeft: T,
        var backRight: T
    ) : Iterable<T> {
        override fun iterator(): Iterator<T> = object : Iterator<T> {
            private var index = 0

            override fun hasNext(): Boolean = index < 4

            override fun next(): T = when (index++) {
                0 -> frontLeft
                1 -> frontRight
                2 -> backLeft
                3 -> backRight
                else -> throw NoSuchElementException()
            }
        }

        operator fun get(index: Int): T = when (index) {
            0 -> frontLeft
            1 -> frontRight
            2 -> backLeft
            3 -> backRight
            else -> throw IndexOutOfBoundsException("Motor index must be between 0 and 3")
        }

        operator fun set(index: Int, value: T) = when (index) {
            0 -> frontLeft = value
            1 -> frontRight = value
            2 -> backLeft = value
            3 -> backRight = value
            else -> throw IndexOutOfBoundsException("Motor index must be between 0 and 3")
        }

        fun set(list: List<T>) {
            list.forEachIndexed { index, it -> this[index] = it }
        }

        fun set(fl: T, fr: T, bl: T, br: T) {
            this[0] = fl
            this[1] = fr
            this[2] = bl
            this[3] = br
        }

        override fun toString(): String {
            fun Double.toHundredths(): Double = (floor(this * 100) / 100)
            if (this.isOfType<Double>()) {
                return this.mapIndexed { index, it -> "${motorNames[index]} | ${(it as Double).toHundredths()}" }.joinToString(", ")
            }
            return this.mapIndexed { index, it -> "${motorNames[index]} | $it" }.joinToString(", ")
        }

        inline fun <reified T> isOfType(): Boolean = this.any { it is T }
    }

    val <T> MotorValues<T?>.areNull: Boolean
        get() {
            return frontLeft == null && frontRight == null && backLeft == null && backRight == null
        }

    fun <T> MotorValues<Number>.equal(number: Number): Boolean {
        return this.any { it.toDouble() == number.toDouble() }
    }

    fun <T> MotorValues<T?>.clear() {
        frontLeft = null
        frontRight = null
        backLeft = null
        backRight = null
    }
}