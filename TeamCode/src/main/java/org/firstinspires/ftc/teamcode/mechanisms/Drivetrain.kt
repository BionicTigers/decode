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
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import io.github.bionictigers.axiom.core.web.Hidden
import io.github.bionictigers.axiom.core.web.WebData
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.MotionProfile
import org.firstinspires.ftc.teamcode.control.MotionResult
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.drivers.OctoQuadFWv3
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Matrix
import org.firstinspires.ftc.teamcode.utils.RollingAverage
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.emptyMatrix
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
import kotlin.math.sqrt
import kotlin.math.withSign
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.TimeMark
import kotlin.time.TimeSource

class Drivetrain(hardwareMap: HardwareMap, val telemetry: Telemetry? = null, val odometry: Odometry? = null, val octoQuad: OctoQuad) : System(), Controllable<BaseProfile> {
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

    @Hidden
    var speedMultiplier = 1.0
        set(value) {
            xProfile.setConstants(xJerk * value, xMaxAcceleration * value, xMaxVelocity * value)
            yProfile.setConstants(yJerk * value, yMaxAcceleration * value, yMaxVelocity * value)
            angularProfile.setConstants(
                angularJerk.radians * value,
                angularMaxAcceleration.radians * value,
                angularMaxVelocity.radians * value
            )
        }

    companion object {
        // TODO: test values and note units
        // values are in mm, need to swap to m for LQR calculations
        val xJerk = 6000.0/1000
        val yJerk = 3000.0/1000
        val angularJerk = Angle.degrees(9000.0)//Angle.degrees(900.0)
        val xMaxAcceleration = 3509.0/1000
        val yMaxAcceleration = 3569.0/4.0/1000
        val angularMaxAcceleration = Angle.degrees(900.0) //900 //130
        val xMaxVelocity = 1003.0/1000
        val yMaxVelocity = 1812.0/1000
        val angularMaxVelocity = Angle.degrees(390.0)//390

        // TODO: Remeasure with corrected ticks/rev conversion
        // in ticks/10ms
        val wheelMaxVel = 71.0
        // there are 384.5 ticks/rev for 435rpm 5203s https://www.gobilda.com/yellow-jacket-planetary-gear-motors?srsltid=AfmBOorR8B21Lt_Da5g7XxAyMjSV-vw9o19dorK7f_eDrLGrl9Zb0Orc
        val ticksPerRev = 384.5
        val xProfile = MotionProfile(xJerk, xMaxAcceleration, xMaxVelocity)
        val yProfile = MotionProfile(yJerk, yMaxAcceleration, yMaxVelocity)
        val angularProfile = MotionProfile(angularJerk.radians, angularMaxAcceleration.radians, angularMaxVelocity.radians)

        val wheelRadius = 0.096
        val lx = 0.371475
        val ly = 0.352425

        val K: Matrix = Matrix(
            arrayOf(
                arrayOf(0.085305766656032, 0.085305766656036, 0.016022179744041, 0.142498099853715, -0.000614655076519, 0.000614655076519, 0.001755158084200),
                arrayOf(-0.085305766656032, 0.085305766656037, -0.016022179744041, -0.000614655076519, 0.142498099853715, 0.001755158084200, 0.000614655076519),
                arrayOf(-0.085305766656032, 0.085305766656036, 0.016022179744041, 0.000614655076519, 0.001755158084200, 0.142498099853715, -0.000614655076519),
                arrayOf(0.085305766656032, 0.085305766656037, -0.016022179744041, 0.001755158084200, 0.000614655076519, -0.000614655076519, 0.142498099853715),
            )
        )

        val testBotDirections = listOf(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE)
        val autoDirections = listOf(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.REVERSE)
    }

    /* We only need to set these if we are using driver control */
    @Hidden
    var driveOrientation: DriveOrientation? = null
    @Hidden
    var xControl: Double = 0.0
    @Hidden
    var yControl: Double = 0.0
    @Hidden
    var rotControl: Double = 0.0

    @Hidden
    val inTeleop get() = driveOrientation != null

    @Hidden
    val motors = DriveMotors(hardwareMap)

//    val octoQuad = hardwareMap.getByName<OctoQuadFWv3>("octoQuad")
//    val encoderData = OctoQuadFWv3.EncoderDataBlock()

    @Hidden
    val velocities = MotorValues(0.0, 0.0, 0.0, 0.0)
    @Hidden
    val rollingAverages = MotorValues(
        RollingAverage(3),
        RollingAverage(3),
        RollingAverage(3),
        RollingAverage(3)
    )

    @Hidden
    val pids = MotorPIDs()

    @Hidden
    val staticFrictionConsts = MotorValues(0.03,0.03,0.03,0.03) // TODO: Update values to new bot
    @Hidden
    val kineticFrictionConsts = MotorValues(1.0/wheelMaxVel,1.0/wheelMaxVel,1.0/wheelMaxVel,1.0/wheelMaxVel) // TODO: Find realer values

//    var controllerCommand: List<Double> = listOf(0.0, 0.0, 0.0, 0.0)
//    var timeChanged = TimeSource.Monotonic.markNow()

    @Hidden
    var powers: MotorValues<Double> = MotorValues(0.0, 0.0, 0.0, 0.0)

    override val dependencies: List<System> = listOfNotNull(odometry, octoQuad)

    @Hidden
    var max = 0.0

    @Hidden
    var mtp: MoveVariables? = null

//    private val headingPID = PID(8.0, 0.0, 0.0, 0, -PI, PI, -1.0, 1.0)
//    private var targetHeading = 0.0
//    private var timeLetGo = TimeSource.Monotonic.markNow()
    private var rotation = 0.0

    init {
        motors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        octoQuad.octoQuad.setSingleEncoderDirection(0, OctoQuadFWv3.EncoderDirection.REVERSE)
        octoQuad.octoQuad.setSingleEncoderDirection(1, OctoQuadFWv3.EncoderDirection.FORWARD)
        octoQuad.octoQuad.setSingleEncoderDirection(2, OctoQuadFWv3.EncoderDirection.REVERSE)
        octoQuad.octoQuad.setSingleEncoderDirection(3, OctoQuadFWv3.EncoderDirection.FORWARD)

        octoQuad.octoQuad.setSingleVelocitySampleInterval(0, 10)
        octoQuad.octoQuad.setSingleVelocitySampleInterval(1, 10)
        octoQuad.octoQuad.setSingleVelocitySampleInterval(2, 10)
        octoQuad.octoQuad.setSingleVelocitySampleInterval(3, 10)
    }

    var stuck = false
    override val update = SystemCommand.create("Motor Power Calculation") {
        enter {
            if (inTeleop) {
                motors.forEachIndexed { index, motor ->
                    motor.direction = testBotDirections[index]
                }
            } else { // auto
                motors.forEachIndexed { index, motor -> motor.direction = autoDirections[index] }
            }
        }

        action {
            updateVelocities()
            if (inTeleop) {
                // rotation pid
//                if (odometry != null) {
//                    if (rotControl.absoluteValue > .05) {
//                        rotation = rotControl
//                        timeLetGo = TimeSource.Monotonic.markNow()
//                        targetHeading = odometry.position.radians % (2 * PI)
//                    } else {
//                        // TODO: fix heading PID logic
//                        val shortestPath = atan2(
//                            sin((odometry.position.radians % (2 * PI)) - targetHeading),
//                            cos((odometry.position.radians % (2 * PI)) - targetHeading)
//                        )
//                        rotation = headingPID.compute(0.0, shortestPath) / PI
//                    }
//                } else {
//                    rotation = rotControl
//                }

                rotation = rotControl // no rot pid

                // shouldn't need this if feedforward works
//                val newCommand = calculateControllerPowers(-xControl, -yControl, rotation)
//                if (largeChange(newCommand, controllerCommand)) {
//                    powers.set(calculateControllerPowers(-xControl, -yControl, rotation))
//                    timeChanged = TimeSource.Monotonic.markNow()
//                } else if (TimeSource.Monotonic.markNow() - timeChanged < 250.milliseconds) {
//                    powers.set(calculateControllerPowers(-xControl, -yControl, rotation))
//                } else {
//                    powers.set(pids.calculatePowers(-xControl, -yControl, rotation))
//                }
//                controllerCommand = newCommand

                // reset pids for drift at target of 0
                if (calculateTargetVels(xControl, yControl, rotation).all { it == 0.0 } ) pids.reset()

                powers.set(pids.calculatePowers(xControl, yControl, rotation)) // pid control
//                powers.set(calculateControllerPowers(-xControl, -yControl, rotation)) // non pid control

                telemetry?.addData("pid", pids)
                telemetry?.addData("powers", powers)

                // TODO: Maintain position when controller values are 0, maybe cancelable with a button press just in case

            } else { // auto
                if (mtp != null) {
                    with(mtp!!) {
                        println("${odometry!!.position} and ${finalPose}")
                        if (Pose(
                                odometry.position.x,
                                odometry.position.y,
                                odometry.position.rotation.degrees.normalizeDegrees()
                        ).within(finalPose, Pose(30,30,1.0))) {
                            println("position correct")
                            telemetry?.addLine("position correct")
                            mtp = null
                        } else if (odometry.velocity.within(Pose(), Pose(5, 5, 5))) {
                            stuck = true
                        } else {
                            stuck = false
                        }
                        telemetry?.addData("currentPose", odometry.position)
                        telemetry?.addData("targetPose", finalPose)
                    }
                } else {
                    // TODO: Maintain position after move to pos
                }
            }
            telemetry?.addData("Velocities", velocities.toString())
            telemetry?.addData("stuck", stuck)
        }
    }

    fun Double.withSignZero(other: Double) = if (other == 0.0) 0.0 else this.withSign(other)

    override val apply = SystemCommand.continuous("Drivetrain Update") {
        if (!inTeleop) {
            if (mtp != null) {
                with(mtp!!) {
//                    println("pid calculated, ${it.deltaTime} loop time")
                    powers.set(controlState)

                    if (stuck) {
                        powers.set(
                            powers[0] + (0.2.withSignZero(powers[0])),
                            powers[1] + (0.2.withSignZero(powers[1])),
                            powers[2] + (0.2.withSignZero(powers[2])),
                            powers[3] + (0.2.withSignZero(powers[3]))
                        )
                    }
                    // TODO: add heading pid modifier

                    telemetry?.addData("Error", errorState.printSimple())
                    telemetry?.addData("Control State", controlState.printSimple())
                    telemetry?.addData("powers", powers)
                }
            } else {
                powers.setAll(0.0) // TODO: figure out how to maintain position
            }
        }
        motors.setPower(powers)
        telemetry?.addData("powers", powers)
    }

    /**
     * @param speed percentage of normal speed, 0 to 1
     */
    fun moveToPosition(target: Pose, speed: Double = 1.0) = Command.create("Move to Position") {
        enter {
            println("mtp")
            speedMultiplier = speed
            val currentPose = Pose(odometry!!.position.x/1000, odometry.position.y/1000, odometry.position.rotation)
            val targetPose = Pose(target.x/1000, target.y/1000, Angle.radians(target.radians))
            println(currentPose)
            println(targetPose)
            val currentAngle = currentPose.rotation
            val targetAngle = denormalizeTargetAngle(currentAngle, target.rotation)
            println(targetAngle)

            mtp = MoveVariables(
                finalPose = target,
                targetPose = targetPose,
                xResult = xProfile.generate(currentPose.x, targetPose.x),
                yResult = yProfile.generate(currentPose.y, targetPose.y),
                angularResult = angularProfile.generate(currentAngle.radians, targetAngle.radians),
                startTime = it.enteredAt,
                errorState = emptyMatrix(),
                controlState = emptyMatrix()
            )

            with(mtp!!) {
                errorState = getErrorState(startTime.elapsedNow() + 20.milliseconds)
                controlState = (-K * errorState).scalar(30.0)
            }

            Scheduler.schedule(calculateLQR())
        }

        action {
            if (mtp == null)
                println("mtp stop")
            stop()
        }
    }

    var firstRun = true
    fun calculateLQR() = Command.create("LQR Calculation") {
        enter {
            println("LQR start")
            firstRun = true
        }

        action {
            if (mtp == null) {
//                println("LQR stop, previous dt: ${it.deltaTime}")
                stop()
            }

            if (firstRun) {
                firstRun = false
                return@action
            }

//            println("LQR, previous dt: ${it.deltaTime}")

            with(mtp!!) {
                errorState = getErrorState(startTime.elapsedNow() + 20.milliseconds)
                controlState = (-K * errorState).scalar(30.0)
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

    var xGlobal = 0.0
    var yGlobal = 0.0
    var rot = 0.0
    var theta = 0.0
    var x = 0.0
    var y = 0.0
    var vx = 0.0
    var vy = 0.0
    fun getErrorState(time: Duration): Matrix {
        val currentPose = odometry!!.position
        val currentVelocity = odometry.velocity

        with (mtp!!) { // odo values are in mm and mm/s, need to swap to m for LQR calculations
            xGlobal = currentPose.x/1000 - (xResult.getPosition(time))
            yGlobal = currentPose.y/1000 - (yResult.getPosition(time))
            rot = currentPose.radians - (angularResult.getPosition(time))

            // convert global deltas to local deltas so LQR dynamics stay linear

//            val h = sqrt(xGlobal.pow(2) + yGlobal.pow(2))
//            val a = Angle.degrees((Angle.radians(atan2(yGlobal, xGlobal)).degrees + 90).normalizeDegrees() + currentPose.degrees).radians
//
//            val x = h * sin(a)
//            val y = h * cos(a)

            theta = currentPose.radians

            x = xGlobal * cos(-theta) + yGlobal * sin(-theta)
            y = xGlobal * sin(theta) + yGlobal * cos(theta)

            telemetry?.addData("xGlobal", xGlobal)
            telemetry?.addData("yGlobal", xGlobal)

            telemetry?.addData("x", x)
            telemetry?.addData("y", y)

//            println(angularResult.getPosition(time))
//            val vx = currentVelocity.x/1000 - (xResult.getVelocity(time))
//            val vy = currentVelocity.y/1000 - (yResult.getVelocity(time))
//            val w = currentVelocity.radians - (angularResult.getVelocity(time))

            // get vel linear is in mm/s i want m, botToWheelVels gives rad/s
            vx = xResult.getVelocity(time)/1000 * cos(-theta) + yResult.getVelocity(time)/1000 * sin(-theta)
            vy = xResult.getVelocity(time)/1000 * sin(theta) + yResult.getVelocity(time)/1000* cos(theta)

            val targetW = botToWheelVels(Pose(vx, vy, (angularResult.getVelocity(time))))
            telemetry?.addData("targetW", targetW.toString())

            // velocities are in ticks/10ms :( I want rad/s!!!!!!!
            val conv = 1/ticksPerRev * (2*PI) * (1/.01)

            val wfl = (velocities.frontLeft*conv) - targetW.frontLeft
            val wfr = (velocities.frontRight*conv) - targetW.frontRight
            val wbl = (velocities.backLeft*conv) - targetW.backLeft
            val wbr = (velocities.backRight*conv) - targetW.backRight

            return Matrix(
                arrayOf(
                    arrayOf(x), arrayOf(y), arrayOf(rot), arrayOf(wfl), arrayOf(wfr), arrayOf(wbl), arrayOf(wbr)
                )
            )
        }
    }

    fun botToWheelVels(botVel: Pose) : MotorValues<Double> {
        return MotorValues(
            (1 / wheelRadius) * (botVel.y - botVel.x + (lx + ly) * botVel.radians),
            (1 / wheelRadius) * (botVel.y + botVel.x - (lx + ly) * botVel.radians),
            (1 / wheelRadius) * (botVel.y + botVel.x + (lx + ly) * botVel.radians),
            (1 / wheelRadius) * (botVel.y - botVel.x - (lx + ly) * botVel.radians),
            ) // gives rad/s
    }

//    fun mtpNoProfile(targetPose: Pose) = Command.create("Move to Position") {
//        action {
//            val currentPose = odometry!!.position
//            val error = currentPose - targetPose
//
//            val x = error.x
//            val y = error.y
//            val rot = error.degrees
//            val vx = 0.0
//            val vy = 0.0
//            val w = 0.0
//
//            errorState = Matrix(
//                arrayOf(
//                    arrayOf(x), arrayOf(y), arrayOf(rot), arrayOf(vx), arrayOf(vy), arrayOf(w)
//                )
//            )
//
//            if (currentPose.within(targetPose, Pose(1, 1, 5))) {
//                stop()
//            }
//        }
//
//        exit {
//            errorState = null
//        }
//    }

    fun updateVelocities() {
        rollingAverages.forEachIndexed { index, it ->
//            it += octoQuad.encoderData.velocity[index].toDouble() // ticks/10ms (raw from OctoQuad)
            velocities[index] = it.average
        }
    }

    fun calculateControllerPowers(x: Double, y: Double, rotation: Double): List<Double> {
        val frontLeft = y - x - rotation
        val frontRight = y + x + rotation
        val backLeft = y + x - rotation
        val backRight = y - x + rotation

        val ratio = maxOf(abs(frontLeft), abs(backLeft), abs(frontRight), abs(backRight), 1.0)

        return listOf(
            frontLeft / ratio,
            frontRight / ratio,
            backLeft / ratio,
            backRight / ratio
        )
    }

    fun calculateTargetVels(x: Double, y: Double, rotation: Double): List<Double> {
        val frontLeft = y - x - rotation
        val frontRight = y + x + rotation
        val backLeft = y + x - rotation
        val backRight = y - x + rotation

        val ratio = maxOf(abs(frontLeft), abs(backLeft), abs(frontRight), abs(backRight), 1.0)

        return listOf(
            frontLeft / ratio * wheelMaxVel,
            frontRight / ratio * wheelMaxVel,
            backLeft / ratio * wheelMaxVel,
            backRight / ratio * wheelMaxVel
        )
    }

    /** matrix values must be from 200ms in the future and in m/s^2 */
    fun calculateTargetVels(matrix: Matrix) : List<Double> {
        val frontLeft = matrix[0, 0] * .2
        val frontRight = matrix[1, 0] * .2
        val backLeft = matrix[2, 0] * .2
        val backRight = matrix[3, 0] * .2

        val maxSP = maxOf(abs(frontLeft), abs(backLeft), abs(frontRight), abs(backRight))

        return if (maxSP > wheelMaxVel) {
            listOf(
                frontLeft / maxSP * wheelMaxVel,
                frontRight / maxSP * wheelMaxVel,
                backLeft / maxSP * wheelMaxVel,
                backRight / maxSP * wheelMaxVel
            )
        } else {
            listOf(frontLeft, frontRight, backLeft, backRight)
        }
    }

    fun denormalizeTargetAngle(current: Angle, target: Angle) : Angle {
        // Pick the equivalent target angle closest to current so the profile
        // takes the shortest rotation path (no long-way-around).
        val shortestDelta = atan2(
            sin(target.radians - current.radians),
            cos(target.radians - current.radians)
        )
        return Angle.radians(current.radians + shortestDelta)
    }

    fun stopMotors() = motors.setPower(0.0, 0.0, 0.0, 0.0)

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

        fun setPower(powers: MotorValues<Double>) { setPower(powers[0], powers[1], powers[2], powers[3]) }

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
        val feedFrwrd = MotorValues(0.0, 0.0, 0.0, 0.0)
        val modifiers = MotorValues(0.0, 0.0, 0.0, 0.0)

        fun calculatePowers(x: Double, y: Double, rotation: Double) : List<Double> {
            setPoints.set(calculateTargetVels(x, y, rotation))
            feedFrwrd.set(calculateFFPowers())
            modifiers.set(calculateModifiers())
            return feedFrwrd.zip(modifiers) { power, mod -> power + mod }
        }

//        fun calculatePowers(matrix: Matrix) : List<Double> {
//            setPoints.set(calculateTargetVels(matrix))
//            feedFrwrd.set(calculateFFPowers())
//            modifiers.set(calculateModifiers())
//            return feedFrwrd.zip(modifiers) { power, mod -> power + mod }
//        }

        private fun calculateFFPowers(): List<Double> {
            return setPoints.mapIndexed { index, velocity ->
                staticFrictionConsts[index] * sign(velocity!!) + kineticFrictionConsts[index] * velocity
            }
        }

        private fun calculateModifiers() : List<Double> = pids.mapIndexed { index, pid -> pid.compute(velocities[index], setPoints[index]?: 0.0) }

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
                    string += "   ${motorNames[index]} | Target Vel: ${it?.toHundredths()}, Current: ${velocities[index].toHundredths()},\n" +
                              "       Raw Power ${feedFrwrd[index].toHundredths()}, Modifier, ${modifiers[index].toHundredths()}"
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

        fun setAll(value: T) {
            this.forEachIndexed { index, _ -> this[index] = value }
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

    fun <T> MotorValues<T?>.setNull() {
        frontLeft = null
        frontRight = null
        backLeft = null
        backRight = null
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

    fun MotorValues<Double>.set(matrix: Matrix) {
        listOf(matrix[0,0], matrix[1,0], matrix[2,0], matrix[3,0]).forEachIndexed{ index, it -> this[index] = it }
    }

    data class MoveVariables(
        var finalPose: Pose,
        var targetPose: Pose,
        var xResult: MotionResult,
        var yResult: MotionResult,
        var angularResult: MotionResult,
        var startTime: TimeMark,
        var errorState: Matrix,
        var controlState: Matrix,
    )
}