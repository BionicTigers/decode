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
        val yMaxAcceleration = 3569.0
        val angularMaxAcceleration = Angle.degrees(900.0) //900 //130
        val xMaxVelocity = 1003.0
        val yMaxVelocity = 1812.0
        val angularMaxVelocity = Angle.degrees(390.0)


        val xProfile = MotionProfile(xJerk, xMaxAcceleration, xMaxVelocity)
        val yProfile = MotionProfile(yJerk, yMaxAcceleration, yMaxVelocity)
        val angularProfile = MotionProfile(angularJerk.radians, angularMaxAcceleration.radians, angularMaxVelocity.radians)

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
    
    // PID controllers for autonomous movement
    private val xPID = PID(6.0, 0.0, 0, 0.0, -1000.0, 4000.0, -1.0, 1.0)
    private val yPID = PID(6.0, 0.0, 0, 0.0, -1000.0, 4000.0, -1.0, 1.0)
    private val rotPID = PID(12.0, 0.0, 0, 0.0, -360.0, 360.0, -1.0, 1.0)
    
    // Feedforward gains (kV for velocity feedforward, kA for acceleration feedforward)
    private val xkV = 0.001 // Velocity feedforward gain for x
    private val xkA = 0.0001 // Acceleration feedforward gain for x
    private val ykV = 0.001 // Velocity feedforward gain for y
    private val ykA = 0.0001 // Acceleration feedforward gain for y
    private val rotkV = 0.001 // Velocity feedforward gain for rotation
    private val rotkA = 0.0001 // Acceleration feedforward gain for rotation

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

    // Motion profile results for current movement
    private var xProfileResult: MotionProfile.MotionResult? = null
    private var yProfileResult: MotionProfile.MotionResult? = null
    private var angularProfileResult: MotionProfile.MotionResult? = null
    private var profileStartTime: TimeMark? = null
    
    // Simple target pose for PID-only control (no motion profile)
    private var simpleTargetPose: Pose? = null

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

    override val afterRun = SystemCommand.continuous("Drivetrain Update", data) {
        if (it.isInTeleop) {
            val powers = calculatePowers(-it.xControl, -it.yControl, rotation)
            telemetry?.addData("Powers", powers.printSimple())
            motors.setPower(powers)

//            println(odometry!!.position.degrees.toString())
//            println(odometry.position.x.toString())
//            println(odometry.position.degrees.toString() + ", " + odometry.position.y.toString())
        } else {
            // Autonomous mode - PID + feedforward control
            val currentPose = odometry!!.position
            
            if (xProfileResult != null && yProfileResult != null && angularProfileResult != null && profileStartTime != null) {
                // Motion profile mode - PID + feedforward with motion profiles
                val currentVelocity = odometry.velocity
                val elapsedTime = profileStartTime!!.elapsedNow()
                
                // Get target position, velocity, and acceleration from motion profiles
                val targetX = xProfileResult!!.getPosition(elapsedTime)
                val targetY = yProfileResult!!.getPosition(elapsedTime)
                val targetRot = angularProfileResult!!.getPosition(elapsedTime)
                
                val targetVx = xProfileResult!!.getVelocity(elapsedTime)
                val targetVy = yProfileResult!!.getVelocity(elapsedTime)
                val targetVRot = angularProfileResult!!.getVelocity(elapsedTime)
                
                val targetAx = xProfileResult!!.getAcceleration(elapsedTime)
                val targetAy = yProfileResult!!.getAcceleration(elapsedTime)
                val targetARot = angularProfileResult!!.getAcceleration(elapsedTime)
                
                // Calculate errors
                val xError = currentPose.x - targetX
                val yError = currentPose.y - targetY
                // Calculate shortest angular error (handles wrap-around)
                val rotError = -Math.toDegrees(
                    atan2(
                        sin(Math.toRadians(targetRot - currentPose.degrees)),
                        cos(Math.toRadians(targetRot - currentPose.degrees))
                    )
                )
                
                // PID control outputs
                val xPIDOutput = xPID.compute(currentPose.x, targetX)
                val yPIDOutput = yPID.compute(currentPose.y, targetY)
                val rotPIDOutput = rotPID.compute(0.0, rotError)
                
                // Feedforward terms (velocity and acceleration feedforward)
                val xFF = xkV * targetVx + xkA * targetAx
                val yFF = ykV * targetVy + ykA * targetAy
                val rotFF = rotkV * targetVRot + rotkA * targetARot
                
                // Combined PID + feedforward
                val xControl = xPIDOutput + xFF
                val yControl = yPIDOutput + yFF
                val rotControl = rotPIDOutput + rotFF
                
                // Convert to motor powers
                val powers = calculatePowers(xControl, yControl, rotControl)
                
                telemetry?.addData("X Error", xError)
                telemetry?.addData("Y Error", yError)
                telemetry?.addData("Rot Error", rotError)
                telemetry?.addData("X PID", xPIDOutput)
                telemetry?.addData("Y PID", yPIDOutput)
                telemetry?.addData("Rot PID", rotPIDOutput)
                telemetry?.addData("X FF", xFF)
                telemetry?.addData("Y FF", yFF)
                telemetry?.addData("Rot FF", rotFF)
                
                motors.setPower(powers)
            } else if (simpleTargetPose != null) {
                // Simple PID-only mode (no motion profile, no feedforward)
                val target = simpleTargetPose!!
                
                // Calculate shortest angular error (handles wrap-around)
                val rotError = -Math.toDegrees(
                    atan2(
                        sin(Math.toRadians(target.degrees - currentPose.degrees)),
                        cos(Math.toRadians(target.degrees - currentPose.degrees))
                    )
                )
                
                // PID control outputs only
                val xControl = xPID.compute(currentPose.x, target.x)
                val yControl = yPID.compute(currentPose.y, target.y)
                val rotControl = rotPID.compute(rotError, 0.0)
                
                // Convert to motor powers
                val powers = calculatePowers(xControl, yControl, rotControl)
                
                telemetry?.addData("X Error", xControl)
                telemetry?.addData("Y Error", yControl)
                telemetry?.addData("Rot Error", rotControl)
                telemetry?.addData("Powers", powers.printSimple())

                motors.setPower(powers)
            } else {
                stop()
            }
        }
    }

    fun moveToPosition(targetPose: Pose) = Command.create("Move to Position") {
        enter {
            val currentPose = odometry!!.position
            val currentVelocity = odometry.velocity
            
            // Generate motion profiles with current velocity as starting velocity
            xProfileResult = xProfile.generate(currentPose.x, targetPose.x, currentVelocity.x)
            yProfileResult = yProfile.generate(currentPose.y, targetPose.y, currentVelocity.y)
            angularProfileResult = angularProfile.generate(currentPose.degrees, targetPose.degrees, currentVelocity.degrees)
            
            profileStartTime = TimeSource.Monotonic.markNow()
            
            // Reset PID controllers
            xPID.reset()
            yPID.reset()
            rotPID.reset()
        }

        action {
            val currentPose = odometry!!.position
            
            // Check if we've reached the target
            if (currentPose.within(targetPose, Pose(1, 1, 5))) {
                stop()
            }
        }

        exit {
            xProfileResult = null
            yProfileResult = null
            angularProfileResult = null
            profileStartTime = null
        }
    }

    fun mtpNoProfile(targetPose: Pose) = Command.create("Move to Position No Profile") {
        enter {
            // Set simple target pose for PID-only control
            simpleTargetPose = targetPose
            
            // Reset PID controllers
            xPID.reset()
            yPID.reset()
            rotPID.reset()
        }
        
        action {
            val currentPose = odometry!!.position
            
            // Check if we've reached the target
            if (currentPose.within(targetPose, Pose(1, 1, 5))) {
                stop()
            }
        }

        exit {
            simpleTargetPose = null
        }
    }

    // fun calculatePowers(x: Double, y: Double, rotation: Double): List<Double> {
    //     val fL = y - x + rotation
    //     val fR = y + x - rotation
    //     val bL = y + x + rotation
    //     val bR = y - x - rotation

    //     return listOf(-fL, -bL, fR, bR)
    // }

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
