package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Digital
import io.github.bionictigers.axiom.core.web.Editable
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.InterpolatedMap
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RollingAverage
import org.firstinspires.ftc.teamcode.utils.Vector2
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.eq
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.interpolatedMapOf
import org.firstinspires.ftc.teamcode.utils.milliseconds
import org.firstinspires.ftc.teamcode.utils.seconds
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

class Output(hardwareMap: HardwareMap, kicker: Kicker? = null, sorter: Sorter? = null, val telemetry: Telemetry? = null, val odometry: Odometry?, octoQuad: OctoQuad, val isRed: Boolean): System(), Controllable<BaseProfile> {
    override val name: String = "Output"
    override val dependencies = listOf(octoQuad)
    // encoder on octoQuad 4

    var active: Boolean = false
    var velocity: RollingAverage = RollingAverage(5)
    var targetVelocity: Double = 0.0
    @Editable
    val pid: PID = PID(2.0, 2.0, 0.0, 1.0, 0.0, 2700.0, 0.0, 1.0)

    interface Schema : ControlSchema {
        val shoot: Digital?
        val stop: Digital?
        val toggle: Digital?
        val toggleSlow: Digital?
        val incVel: Digital?
        val decVel: Digital?
        val smallIncVel: Digital?
        val smallDecVel: Digital?
        val aimLeft: Digital?
        val aimRight: Digital?
        val resetOdometry: Digital?
    }

    val motor = hardwareMap.getByName<DcMotorEx>("output")
    var angle = Angle.ZERO

    var currentVel = 0.0
    var targetAngle = angle.degrees - 1

    val indcLight = hardwareMap.getByName<Servo>("indcLight")
    val turret = hardwareMap.getByName<Servo>("turret")
    val hub = ControlHub(hardwareMap, "Control Hub")

    val deltaTime = RollingAverage(5)

    var junkTicks = octoQuad.encoderData.position[4]

    val rampAngle = Angle.degrees(45.7) // output angle of the ball
    val shooterHeight = 255.2 // must be in mm

    // at the corners of the field
    val redGoalPos = Pose(3657.6,0.0, 0.0)
    val blueGoalPos = Pose(3657.6,3657.6, 0.0)
    // distance from red/blueGoalPos to the wall of the goal at their closest point, in mm
    val k = 464.5

    // val ballToWheelVel = interpolatedMapOf(
    //     1894.845639 to 500.0,
    //     3406.043328 to 1000.0,
    //     3914.481296 to 1100.0,
    //     4414.634545 to 1200.0,
    //     4769.74576 to 1300.0,
    //     5301.269619 to 1400.0,
    //     5378.976959 to 1500.0,
    //     5468.872693 to 1600.0,
    //     5873.160028 to 1700.0
    // )

    val velocityMap = interpolatedMapOf(
        3600.0 to 2230.0,
        2042.0 to 1900.0
    )
//    val velocityMap2 = interpolatedMapOf(
//        3044.0 to 2060.0, // 12.1 V
//        3044.0 to 1950.0,

//        )

    @Editable
    var farTarget = 1625.0//1980.0
    @Editable
    var closeTarget = 1430.0//1580.0

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER
        deltaTime += 20.0
    }

    var lastDist = 0.0
    var lastvel = 0.0
    var lastvoltage = 0.0

    override val update = SystemCommand.continuous("Output Data") {
//        val lVel = hub.getEncoderTicks(0) / it.deltaTime.seconds
        val lVel = (octoQuad.encoderData.position[4] - junkTicks) / it.deltaTime.seconds
        if (lVel.isFinite()) {
            velocity.plusAssign(lVel)
        }

//        if (odometry != null) {
//            val autoVelocity = calculateShooterVel()
//            telemetry?.addData("calculation", autoVelocity)
//            if (autoVelocity != null) {
//                targetVelocity = autoVelocity
//            } else {
//                // indicator light?
//            }
//        }

        currentVel = velocity.average
//        maxVelocity = max(velocity.average, maxVelocity)
//        telemetry?.addData("maxVelocity", maxVelocity)
        telemetry?.addData("target", targetVelocity)
        telemetry?.addData("velocity", velocity.average)
        telemetry?.addData("power", motor.ePower)
        telemetry?.addLine("---------------------")


        if (active) {
            val currentPos = odometry!!.position
            val goalPos = if (isRed) redGoalPos else blueGoalPos
//
//                val goalPointA = if (isRed) redGoalPos.position + Vector2(0, 720) else blueGoalPos.position + Vector2(720, 0)
//                val goalPointB = if (isRed) redGoalPos.position + Vector2(720, 0) else blueGoalPos.position + Vector2(0, 720)
//                val closestPoint = closestPointOnLineSegment(
//                    goalPointA,
//                    goalPointB,
//                    currentPos.position
//                )
//
//                val distToGoalWall = (currentPos.position - closestPoint).magnitude()
            val distToGoalWall = (goalPos.position - currentPos.position).magnitude()
            targetVelocity = velocityMap[distToGoalWall]

            val error = targetVelocity - velocity.average
            val rampPower = if (error > 400) 1.0 else 0.0
            motor.ePower = pid.compute(velocity.average, targetVelocity) + rampPower
            println(pid.compute(velocity.average, targetVelocity) + rampPower)
            if (kicker?.reset ?: false) {
                lastDist = distToGoalWall
                lastvel = velocity.average
                lastvoltage = hub.getVoltage()
            }
//            } else {
//                val error = targetVelocity - velocity.average
//                val rampPower = if (error > 400) 1.0 else 0.0
//                motor.ePower = pid.compute(velocity.average, targetVelocity) + rampPower
//            }
        } else {
            motor.ePower = 0.0
        }

        telemetry?.addData("distToGoalWall", lastDist)
        telemetry?.addData("last shot vel", lastvel)
        telemetry?.addData("voltage", lastvoltage)
        telemetry?.addLine("---------------------")

        if (active && velocity.average.eq(targetVelocity, 45.0))
            indcLight.position = .5
        else if (active)
            indcLight.position = .28
        else
            indcLight.position = .63


        junkTicks = octoQuad.encoderData.position[4]
    }

    override val apply = SystemCommand.continuous("Turret Auto Aim") {
        if (odometry == null) {
            return@continuous
        }
//        // Static target position in mm
        val targetX = if (isRed) redGoalPos.x else blueGoalPos.x
        val targetY = if (isRed) redGoalPos.y else blueGoalPos.y

        // Calculate angle from robot to target in field coordinates
        // Using atan2(deltaX, deltaY) because robot heading 0° = positive Y, 90° = positive X
        val deltaX = targetX - odometry.position.x
        val deltaY = targetY - odometry.position.y
        val angleToTarget = atan2(deltaX, deltaY) // radians

        // Get robot's current heading in radians
        val robotHeading = odometry.position.radians

        // Calculate relative angle (how much turret needs to turn from robot's forward direction)
        var relativeAngle = angleToTarget - robotHeading

        // Normalize to [-π, π] so turret takes the shortest path
        while (relativeAngle > PI) relativeAngle -= 2 * PI
        while (relativeAngle < -PI) relativeAngle += 2 * PI

        // Convert to servo position
        // At 0.5, turret is aligned with robot's forward direction (0 relative angle)
        // 90° servo range: -45° (servo 0.0) to +45° (servo 1.0)
        val servoPosition = (0.5 - ((relativeAngle) / (PI / 3))).coerceIn(0.1, 0.9)

        telemetry?.addData("Angle", Math.toDegrees(relativeAngle))
        telemetry?.addData("Servo Position", servoPosition)

        if (isRed)
            turret.position = servoPosition
        else
            turret.position = servoPosition + .02
//        turret.position = 0.5
    }

    fun vectorToSegment(t: Double, a: Vector2, b: Vector2, p: Vector2): Vector2 {
        return Vector2(
            (1 - t) * a.x + t * b.x - p.x,
            (1 - t) * a.y + t * b.y - p.y
        )
    }

    fun closestPointOnLineSegment(a: Vector2, b: Vector2, p: Vector2): Vector2 {
        val v = b - a
        val u = a - p
        val vu = v.dot(u)
        val vv = v.x.pow(2) + v.y.pow(2)
        val t = -vu / vv
        if (t in 0.0..1.0) return vectorToSegment(t, a, b, Vector2())
        val g0 = vectorToSegment(0.0, a, b, p).diag()
        val g1 = vectorToSegment(1.0, a, b, p).diag()
        return if (g0 <= g1) a else b
    }

//    fun calculateShooterVel() : Double? {
//        val currentPos = odometry!!.position
//        val goalPos = if (isRed) redGoalPos else blueGoalPos
//        val kDrag = 0.168 // stupid ball is slowed down by air too much, this was calculated using the ratio of the old far and close shot values
//        val p = 1.05
//
////        val distFromGoal = sqrt((currentPos.x - goalPos.x).pow(2) + (currentPos.y - goalPos.y).pow(2))
////        // account for differences in goal distance at different angles, since the goal is triangular
////        val distToGoalWall = distFromGoal - ( k / ( cos(PI/4 - atan2(currentPos.y - goalPos.y, currentPos.x - goalPos.x)) ) )
////        telemetry?.addData("thing", ( k / ( cos(PI/4 - atan2(currentPos.y - goalPos.y, currentPos.x - goalPos.x)) ) ))
//
//        val goalPointA = if (isRed) redGoalPos.position + Vector2(0, 720) else blueGoalPos.position + Vector2(720, 0)
//        val goalPointB = if (isRed) redGoalPos.position + Vector2(720, 0) else blueGoalPos.position + Vector2(0, 720)
//        val closestPoint = closestPointOnLineSegment(
//            goalPointA,
//            goalPointB,
//            currentPos.position
//        )
//
//        val distToGoalWall = (currentPos.position - closestPoint).magnitude()
//        telemetry?.addData("distToGoalWall", distToGoalWall)
//        telemetry?.addData("closestPoint", closestPoint.toString())
//
//
//        // the desired height of the ball when it passes over the goal wall, height of goal + 5in, in mm
//        val heightAtWall = 984.5 + 127
//
//        val ballVelocity = if (distToGoalWall * tan(rampAngle.radians) + shooterHeight - kDrag * distToGoalWall > heightAtWall) {
//            // all units must be in mm , including gravity
//            sqrt( (distToGoalWall.pow(2) * 9800) /
//                    ( 2 * cos(rampAngle.radians).pow(2) * (distToGoalWall * tan(rampAngle.radians) + shooterHeight - heightAtWall - kDrag * distToGoalWall.pow(p)) ) )
//        } else {
//            return null // if the statement is false, the shot is impossible at the current ramp angle and field position
//        }
//        telemetry?.addData("ball vel", ballVelocity)
//        // ball velocity is in 3d, we need to project it to 2d x and y to adjust for robot movement before converting back
//        val projBallVel = ballVelocity * cos(rampAngle.radians)
//
//        val shotAngle = atan2(goalPos.y - currentPos.y, goalPos.x - currentPos.x)
//        val adjusted2dBallVel =
//            sqrt((projBallVel * cos(shotAngle) - odometry.velocity.x).pow(2)
//                    + (projBallVel * sin(shotAngle) - odometry.velocity.y).pow(2))
//        // convert back to 3d
//        val adjustedBallVel = sqrt( adjusted2dBallVel.pow(2) + (ballVelocity * sin(rampAngle.radians)).pow(2) )
//
//        telemetry?.addData("adjustedBallVel", adjustedBallVel)
//        telemetry?.addData("friction", ballToWheelVel[adjustedBallVel])
//
//        return ballToWheelVel[adjustedBallVel]
//    }

    fun shoot() = SystemCommand.instant("Output Enable Far Target") {
//        motor.power = .83
        active = true
    }

    fun shootClose() = SystemCommand.instant("Output Enable Close Target") {
//        motor.power = .73
        active = true
    }

    fun incVel() = SystemCommand.instant("Output Enable") {
        targetVelocity += 100
        active = true
    }

    fun decVel() = SystemCommand.instant("Output Enable") {
        if (targetVelocity > 100) {
            targetVelocity -= 100
        } else {
            targetVelocity = 0.0
            active = false
        }
    }

    fun smallIncVel() = SystemCommand.instant("Output Enable") {
        targetVelocity += 10
        active = true
    }

    fun smallDecVel() = SystemCommand.instant("Output Enable") {
        if (targetVelocity > 10) {
            targetVelocity -= 10
        } else {
            targetVelocity = 0.0
            active = false
        }
    }

    fun stop() = SystemCommand.instant("Output Disable") {
//        motor.power = 0.0
        targetVelocity = 0.0
        active = false
    }

    fun turnLeft() = SystemCommand.instant("Output Aim Left") {
        turret.position += .1
    }

    fun turnRight() = SystemCommand.instant("Output Aim Right") {
        turret.position += -.1
    }

    fun resetOdometry() = SystemCommand.instant("Reset Odometry") {
        odometry?.setPose(Pose(609.6 * 6 - Distance.inch(9).mm / 2, 609.6 * 3, 270))
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.output) {
            if (!desiredGamepad.matches(gamepad)) return

            shoot?.let { builder.register(it) { shoot() } }
            stop?.let { builder.register(it) { stop() } }
            toggle?.let { builder.register(it) { if (active && targetVelocity == farTarget) stop() else shoot() } }
            toggleSlow?.let { builder.register(it) { if (active && targetVelocity == closeTarget) stop() else shootClose() } }
            incVel?.let { builder.register(it) { incVel() } }
            decVel?.let { builder.register(it) { decVel() } }
            smallIncVel?.let { builder.register(it) { smallIncVel() } }
            smallDecVel?.let { builder.register(it) { smallDecVel() } }
            aimLeft?.let { builder.register(it) { turnLeft() } }
            aimRight?.let { builder.register(it) { turnRight() } }
            resetOdometry?.let { builder.register(it) { resetOdometry() } }
        }
    }
}
