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
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.eq
import org.firstinspires.ftc.teamcode.utils.getByName
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

class Output(hardwareMap: HardwareMap, kicker: Kicker? = null, sorter: Sorter? = null, telemetry: Telemetry? = null, val odometry: Odometry, octoQuad: OctoQuad, val isRed: Boolean): System(), Controllable<BaseProfile> {
    override val name: String = "Output"
    override val dependencies = listOf(octoQuad)
    // encoder on octoQuad 4

    var active: Boolean = false
    var velocity: RollingAverage = RollingAverage(5)
    var targetVelocity: Double = 0.0
    @Editable
    val pid: PID = PID(2.0, 2.0, 0.0, 1.0, 0.0, 2700.0, 0.0, 1.0)
    var maxVelocity: Double = 0.0

    interface Schema : ControlSchema {
        val shoot: Digital?
        val stop: Digital?
        val toggle: Digital?
        val toggleSlow: Digital?
        val aimLeft: Digital
        val aimRight: Digital
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
    val redGoalPos = Pose(0.0,3657.6, 0.0)
    val blueGoalPos = Pose(3657.6,3657.6, 0.0)
    // distance from red/blueGoalPos to the wall of the goal at their closest point, in mm
    val k = 464.5

    val ballToWheelVel = InterpolatedMap(
        // TODO
    )

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

    override val update = SystemCommand.continuous("Output Data") {
//        val lVel = hub.getEncoderTicks(0) / it.deltaTime.seconds
        val lVel = (octoQuad.encoderData.position[4] - junkTicks) / it.deltaTime.seconds
        if (lVel.isFinite()) {
            velocity.plusAssign(lVel)
        }

        val autoVelocity = calculateShooterVel()
        if (autoVelocity != null) {
            targetVelocity = autoVelocity
        } else {
            // indicator light?
        }

        currentVel = velocity.average
        maxVelocity = max(velocity.average, maxVelocity)
        telemetry?.addData("maxVelocity", maxVelocity)
        telemetry?.addData("target", targetVelocity)
        telemetry?.addData("velocity", velocity.average)
        telemetry?.addData("power", motor.ePower)

        if (active) {
            if (kicker?.reset ?: false)
                motor.ePower = 1.0
            else {
                val error = targetVelocity - velocity.average
                val rampPower = if (error > 100) 1.0 else 0.0
                motor.ePower = pid.compute(velocity.average, targetVelocity) + rampPower
            }
        } else {
            motor.ePower = 0.0
        }

        if (active && velocity.average.eq(targetVelocity, 45.0))
            indcLight.position = .5
        else if (active)
            indcLight.position = .28
        else
            indcLight.position = .63


        junkTicks = octoQuad.encoderData.position[4]
    }

    override val apply = SystemCommand.continuous("Turret Auto Aim") {
        deltaTime += it.deltaTime.milliseconds

        // Static target position in mm
        val targetX = 3352.8 + 300
        val targetY = 0

        // predicted position
        val predictedPose = Pose(odometry.position.x + odometry.velocity.x * deltaTime.average,
            odometry.position.y + odometry.velocity.y * deltaTime.average,
            odometry.position.radians + odometry.velocity.radians * deltaTime.average)

        // Calculate angle from robot to target in field coordinates
        // Using atan2(deltaX, deltaY) because robot heading 0° = positive Y, 90° = positive X
        val deltaX = targetX - predictedPose.x
        val deltaY = targetY - predictedPose.y
        val angleToTarget = atan2(deltaX, deltaY) // radians

        // Get robot's current heading in radians
        val robotHeading = predictedPose.radians

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

        turret.position = servoPosition
//        turret.position = 0.5
    }

    fun calculateShooterVel() : Double? {
        val currentPos = odometry.position
        val goalPos = if (isRed) redGoalPos else blueGoalPos
        val kDrag = 0.168 // stupid ball is slowed down by air too much, this was calculated using the ratio of the old far and close shot values
        val p = 1.05

        val distFromGoal = sqrt((currentPos.x - goalPos.x).pow(2) + (currentPos.y - goalPos.y).pow(2))
        // account for differences in goal distance at different angles, since the goal is triangular
        val distToGoalWall = distFromGoal - ( k / ( cos(PI/4 - atan2(currentPos.y - goalPos.y, currentPos.x - goalPos.x)) ) )

        // the desired height of the ball when it passes over the goal wall, height of goal + 5in, in mm
        val heightAtWall = 984.5 + 127

        val ballVelocity = if (distToGoalWall * tan(rampAngle.radians) + shooterHeight - kDrag * distToGoalWall > heightAtWall) {
            // all units must be in mm , including gravity
            sqrt( (distToGoalWall.pow(2) * 9800) /
                    ( 2 * cos(rampAngle.radians).pow(2) * (distToGoalWall * tan(rampAngle.radians) + shooterHeight - heightAtWall - kDrag * distToGoalWall.pow(p)) ) )
        } else {
            return null // if the statement is false, the shot is impossible at the current ramp angle and field position
        }
        // ball velocity is in 3d, we need to project it to 2d x and y to adjust for robot movement before converting back
        val projBallVel = ballVelocity * cos(rampAngle.radians)

        val shotAngle = atan2(goalPos.y - currentPos.y, goalPos.x - currentPos.x)
        val adjusted2dBallVel =
            sqrt((projBallVel * cos(shotAngle) - odometry.velocity.x).pow(2)
                    + (projBallVel * sin(shotAngle) - odometry.velocity.y).pow(2))
        // convert back to 3d
        val adjustedBallVel = sqrt( adjusted2dBallVel.pow(2) + (ballVelocity * sin(rampAngle.radians)).pow(2) )

        return ballToWheelVel[adjustedBallVel]
    }

    fun shoot() = SystemCommand.instant("Output Enable") {
//        motor.power = .83
        targetVelocity = farTarget
        active = true
    }

    fun shootClose() = SystemCommand.instant("Output Enable Slow") {
//        motor.power = .73
        targetVelocity = closeTarget
        active = true
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
        odometry.setPose(Pose(609.6 * 6 - Distance.inch(9).mm / 2, 609.6 * 3, 270))
    }

    override fun bindControls(profile: BaseProfile, gamepad: Gamepads, builder: Controls.Builder) {
        with(profile.output) {
            if (!desiredGamepad.matches(gamepad)) return

            shoot?.let { builder.register(it) { shoot() } }
            stop?.let { builder.register(it) { stop() } }
            toggle?.let { builder.register(it) { if (active && targetVelocity == farTarget) stop() else shoot() } }
            toggleSlow?.let { builder.register(it) { if (active && targetVelocity == closeTarget) stop() else shootClose() } }
            aimLeft?.let { builder.register(it) { turnLeft() } }
            aimRight?.let { builder.register(it) { turnRight() } }
            resetOdometry?.let { builder.register(it) { resetOdometry() } }
        }
    }
}
