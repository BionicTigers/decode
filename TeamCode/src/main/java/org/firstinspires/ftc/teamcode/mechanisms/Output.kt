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
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RollingAverage
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.eq
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.seconds
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sqrt

class Output(hardwareMap: HardwareMap, kicker: Kicker? = null, sorter: Sorter? = null, telemetry: Telemetry? = null, val odometry: Odometry, octoQuad: OctoQuad): System(), Controllable<BaseProfile> {
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
    var distance: Double = 0.0
    var finalVeriticalVelocity: Double  = 0.0
    var initialVerticalVelocity: Double = 0.0
    val verticalAcceleration: Double = 9.81
    var time: Double = 0.0
    var initialHorizontalVelocity: Double = 0.0
    var finalHorizontalVelocity: Double = 0.0
    var wheelVelocity: Double = 0.0
    var wheelSpeedVar: Double = 0.0
    var c =


    val motor = hardwareMap.getByName<DcMotorEx>("output")
    var angle = Angle.ZERO

    var currentVel = 0.0
    var targetAngle = angle.degrees - 1

    val indcLight = hardwareMap.getByName<Servo>("indcLight")
    val turret = hardwareMap.getByName<Servo>("turret")
    val hub = ControlHub(hardwareMap, "Control Hub")

    var junkTicks = octoQuad.encoderData.position[4]

    @Editable
    var farTarget = 1625.0//1980.0
    @Editable
    var closeTarget = 1430.0//1580.0

    init {
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor.mode == DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override val update = SystemCommand.continuous("Output Data") {
        distance = sqrt((3654.0 - odometry.position.x).pow(2)+(0 - odometry.position.y).pow(2))
        finalVeriticalVelocity = sqrt(initialVerticalVelocity.pow(2) + 2 * verticalAcceleration * 15)
        time = (finalVeriticalVelocity - initialVerticalVelocity) / verticalAcceleration

        initialHorizontalVelocity = distance/time
        initialHorizontalVelocity =  finalHorizontalVelocity

        wheelVelocity = sqrt(finalHorizontalVelocity.pow(2)+ finalVeriticalVelocity.pow(2))
        wheelSpeedVar = 2 * kotlin.math.PI * ((9.72 / 2.0) + 5/(9.72 / 2.0)) * wheelVelocity

        rpm = (wheelSpeedVar / c) * 60/6000
//        val lVel = hub.getEncoderTicks(0) / it.deltaTime.seconds
        val lVel = (octoQuad.encoderData.position[4] - junkTicks) / it.deltaTime.seconds
        if (lVel.isFinite()) {
            velocity.plusAssign(lVel)
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
//        // Static target position in mm
        val targetX = 3352.8 + 300
        val targetY = 0

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

        turret.position = servoPosition
//        turret.position = 0.5
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
