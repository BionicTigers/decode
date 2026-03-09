package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.input.ControlSchema
import io.github.bionictigers.axiom.core.input.Controllable
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.input.matches
import io.github.bionictigers.axiom.core.input.types.Digital
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.control.PID
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.RollingAverage
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.interpolatedMapOf
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.time.TimeMark
import kotlin.time.TimeSource

class Shooter(hardware: HardwareMap, val odometry: Odometry?, val limeLight: LimeLight?, val telemetry: Telemetry, val isRed: Boolean) : System(), Controllable<BaseProfile> {
    override val name = "Shooter"

    override val dependencies = listOfNotNull(odometry, limeLight)

    interface Schema : ControlSchema {
        val togglePower: Digital?
        val aimLeft: Digital?
        val aimRight: Digital?
        val hoodUp: Digital?
        val hoodDown: Digital?
    }

    val redGoalPos = Pose(3657.6 - 50,0.0, 0.0)
    val blueGoalPos = Pose(3627.6 - 50,3627.6, 0.0)

    val ticksPerRev = 384.5 // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/

    val flywheel = hardware.getByName<DcMotorEx>("shooter")
    val turret = hardware.getByName<DcMotorEx>("shooterAim")
    val hoodServo = hardware.getByName<Servo>("hood")

    val turretPID = PID(0.1, 0.0, 0.0, 0.0, -180.0, 180.0, -.5, .5)
    val flywheelPID: PID = PID(2.0, 2.0, 0.0, 1.0, 0.0, 2700.0, 0.0, 1.0)

    val flywheelVel = RollingAverage(3)

    val flywheelDistToVel = interpolatedMapOf(
        3600.0 to 2100.0,
        2042.0 to 1730.0
    )

    val hoodDistToAngle = interpolatedMapOf(
        0.0 to 0.0,
        2042.0 to .3
    )

    var aimedAt: TimeMark? = null

    override val apply = SystemCommand.continuous {
        val currentAngle = (turret.currentPosition / ticksPerRev * 360.0).normalizeDegrees180()
        //Static target position in mm
        val targetX = if (isRed) redGoalPos.x else blueGoalPos.x
        val targetY = if (isRed) redGoalPos.y else blueGoalPos.y

        // Calculate angle from robot to target in field coordinates
        // Using atan2(deltaX, deltaY) because robot heading 0° = positive Y, 90° = positive X
        val deltaX = targetX - odometry!!.position.x
        val deltaY = targetY - odometry.position.y
        val angleToTarget = atan2(deltaX, deltaY) // radians

        // Get robot's current heading in radians
        val robotHeading = odometry.position.radians

        // Calculate relative angle (how much turret needs to turn from robot's forward direction)
        var targetAngle = Angle.radians(angleToTarget - robotHeading)

        // Normalize to [-π, π] so turret takes the shortest path
        while (targetAngle.radians > PI) targetAngle.radians -= 2 * PI
        while (targetAngle.radians < -PI) targetAngle.radians += 2 * PI
        targetAngle = Angle.degrees(targetAngle.degrees.coerceIn(-170.0, 170.0))

        val tolerance = 0.0
        if ((currentAngle - tolerance) < targetAngle.degrees && targetAngle.degrees < (currentAngle + tolerance)) {
//            targetAngle = Angle.degrees(limeLight!!.getAngle())
        }

        val error = (targetAngle.degrees - currentAngle).normalizeDegrees180()
        val correctedTarget = currentAngle + error

        turret.ePower = turretPID.compute(currentAngle, correctedTarget)

        flywheelVel += flywheel.velocity

        val currentPos = odometry.position
        val goalPos = if (isRed) redGoalPos else blueGoalPos
        val distToGoalWall = (goalPos.position - currentPos.position).magnitude()

        val targetVelocity = flywheelDistToVel[distToGoalWall]
        flywheel.ePower = flywheelPID.compute(flywheelVel.average, targetVelocity)

        val hoodAngle = hoodDistToAngle[distToGoalWall]
        hoodServo.position = hoodAngle
    }

    fun togglePower() = SystemCommand.instant("Toggle Shooter Power") {
        if (flywheel.power == 0.0) {
            flywheel.power = 1.0
        } else {
            flywheel.power = 0.0
        }
    }

    fun aimLeft() = SystemCommand.instant("Aim Left") {
        turret.power = -1.0
        aimedAt = TimeSource.Monotonic.markNow()
    }

    fun aimRight() = SystemCommand.instant("Aim Right") {
        turret.power = 1.0
        aimedAt = TimeSource.Monotonic.markNow()
    }

    fun hoodUp() = SystemCommand.instant("Hood Up") {
        hoodServo.position += .1
    }

    fun hoodDown() = SystemCommand.instant("Hood Down") {
        hoodServo.position -= .1
    }

    override fun bindControls(
        profile: BaseProfile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ) {
        with(profile.shooter) {
            if (!desiredGamepad.matches(gamepad)) return

            togglePower?.let { builder.register(it) { togglePower() } }
            aimLeft?.let { builder.register(it) { aimLeft() } }
            aimRight?.let { builder.register(it) { aimRight() } }
            hoodUp?.let { builder.register(it) { hoodUp() } }
            hoodDown?.let { builder.register(it) { hoodDown() } }
        }
    }

    fun Double.normalizeDegrees180(): Double {
        var angle = this % 360
        if (angle > 180) angle -= 360
        if (angle < -180) angle += 360
        return angle
    }

}