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
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.withSign
import kotlin.time.TimeMark
import kotlin.time.TimeSource

class Shooter(hardware: HardwareMap, val odometry: Odometry?, val limeLight: LimeLight?, val telemetry: Telemetry, val isRed: Boolean) : System(), Controllable<BaseProfile> {
    override val name = "Shooter"

    override val dependencies = listOfNotNull(odometry, limeLight)

    interface Schema : ControlSchema {
        val togglePower: Digital?
        val flywlInc: Digital?
        val flywlDec: Digital?
        val aimLeft: Digital?
        val aimRight: Digital?
        val hoodUp: Digital?
        val hoodDown: Digital?
    }

    val redGoalPos = Pose(3657.6 - 50,0.0, 0.0)
    val blueGoalPos = Pose(3627.6 - 50,3627.6, 0.0)

    val ticksPerRev = (4259.0 - 2762.0) // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/

    val flywheel = hardware.getByName<DcMotorEx>("shooter")
    val turret = hardware.getByName<DcMotorEx>("shooterAim")
    val hoodServo = hardware.getByName<Servo>("hood")

    val turretPID = PID(5.0, 0.0, 0.0, 0.0, -180.0, 180.0, -.5, .5)
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

    var flywlMod = 0.0
    var aimMod = 0.0
    var hoodMod = 0.0

    var isActive = false

    var junkTicks = 0

    var aimedAt: TimeMark? = null

    init {
        junkTicks = turret.currentPosition
    }

    override val apply = SystemCommand.continuous {
        //turret

        val currentAngle = (((turret.currentPosition - junkTicks) / ticksPerRev) * 360.0).normalizeDegrees180()
        //Static target position in mm
        val targetX = if (isRed) redGoalPos.x else blueGoalPos.x
        val targetY = if (isRed) redGoalPos.y else blueGoalPos.y

        val futurePos = odometry!!.position + (odometry.velocity * .02)

        // Calculate angle from robot to target in field coordinates
        // Using atan2(deltaX, deltaY) because robot heading 0° = positive Y, 90° = positive X
        val deltaX = targetX - futurePos.x
        val deltaY = targetY - futurePos.y
        val angleToTarget = atan2(deltaX, deltaY) // radians

        // Get robot's current heading in radians
        val robotHeading = futurePos.radians

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

        telemetry.addData(" -- shooter futurePose --", futurePos)
        telemetry.addLine("--- aim ---")
        telemetry.addData("currentAngle",currentAngle)

//        telemetry.addData("correctedTarget",correctedTarget)

        val aimTarget = (targetAngle.degrees + aimMod).coerceIn(-170.0, 170.0)

        telemetry.addData("-aimMod",aimMod)
        telemetry.addData("-aimTarget",aimTarget)

        val ff = if(abs(aimTarget - currentAngle) > 2) .2 else 0.0
        val pidPow = turretPID.compute(currentAngle, aimTarget)
//        turret.ePower = (pidPow + ff.withSign(pidPow))

        telemetry.addData("aim powa", pidPow)

        telemetry.addData("ticks", turret.currentPosition)

        //flywheel
        telemetry.addLine("--- flywheel ---")

        flywheelVel += flywheel.velocity

        //val currentPos = odometry.position
        val goalPos = if (isRed) redGoalPos else blueGoalPos
        val distToGoalWall = (goalPos.position - futurePos.position).magnitude()

        val targetVelocity = (flywheelDistToVel[distToGoalWall] + flywlMod).coerceIn(0.0..3000.0)
        flywheel.ePower =  if (isActive) flywheelPID.compute(flywheelVel.average, targetVelocity) else 0.0
        telemetry.addData("flywheelVel", flywheelVel.average)
        telemetry.addData("-flywlMod",flywlMod)
        telemetry.addData("-targetVelocity", targetVelocity)
        telemetry.addData("flywheel power", if (isActive) flywheelPID.compute(flywheelVel.average, targetVelocity) else 0.0)

        // hood
        telemetry.addLine("--- hood ---")

        val hoodAngle = (hoodDistToAngle[distToGoalWall] + hoodMod).coerceIn(0.0..0.3)
        telemetry.addData("-hoodMod",hoodMod)
        telemetry.addData("-hoodAngle", hoodAngle)

        hoodServo.position = hoodAngle
    }

    fun togglePower() = SystemCommand.instant("Toggle Shooter Power") {
        isActive = !isActive
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

    fun flywheelManualInc() = SystemCommand.instant("Flywheel Manual Inc") {
        flywlMod += 100
    }

    fun flywheelManualDec() = SystemCommand.instant("Flywheel Manual Dec") {
        flywlMod -= 100
    }

    fun aimManualRight() = SystemCommand.instant("Flywheel Manual Inc") {
        aimMod += 5
    }

    fun aimManualLeft() = SystemCommand.instant("Flywheel Manual Dec") {
        aimMod -= 5
    }

    fun hoodManualInc() = SystemCommand.instant("Flywheel Manual Inc") {
        flywlMod += .05
    }

    fun hoodManualDec() = SystemCommand.instant("Flywheel Manual Dec") {
        hoodMod -= .05
    }

    override fun bindControls(
        profile: BaseProfile,
        gamepad: Gamepads,
        builder: Controls.Builder
    ) {
        with(profile.shooter) {
            if (!desiredGamepad.matches(gamepad)) return

            togglePower?.let { builder.register(it) { togglePower() } }
            aimLeft?.let { builder.register(it) { aimManualLeft() } }
            aimLeft?.let { builder.register(it) { aimManualLeft() } }
            aimRight?.let { builder.register(it) { aimManualRight() } }
            hoodUp?.let { builder.register(it) { hoodManualInc() } }
            hoodDown?.let { builder.register(it) { hoodManualDec() } }
        }
    }

    fun Double.normalizeDegrees180(): Double {
        var angle = this % 360
        if (angle > 180) angle -= 360
        if (angle < -180) angle += 360
        return angle
    }

}