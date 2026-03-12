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
import org.firstinspires.ftc.teamcode.utils.ePosition
import org.firstinspires.ftc.teamcode.utils.ePower
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.interpolatedMapOf
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.withSign
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.TimeMark
import kotlin.time.TimeSource
import kotlin.time.measureTime

class Shooter(hardware: HardwareMap, val odometry: Odometry?, val limeLight: Limelight?, val telemetry: Telemetry? = null, val isRed: Boolean) : System(), Controllable<BaseProfile> {
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

    val ticksPerRev = 384.5 * 2.825 * 2 // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/

    val flywheel = hardware.getByName<DcMotorEx>("shooter")
    val turret = hardware.getByName<DcMotorEx>("shooterAim")
    val hood = hardware.getByName<Servo>("hood")
//    val wheelLight = hardware.getByName<Servo>("wheelLight")
    val aimLight = hardware.getByName<Servo>("aimLight")

    private val aimLowerBound = -79.0
    private val aimUpperBound = 105.0
    private val visionMeasurementTimeout = 250.milliseconds
    private val maxVisionTrimDegrees = 20.0
    private val visionBlend = 0.65
    private val visionDecay = 0.8
    private val maxVisionSettleErrorDegrees = 12.0
    private val maxVisionTurretVelocity = 10.0
    private val limelightLeftOffsetMm = 100.0
    private val limelightMountYawBiasDegrees = 0.0

    val turretPID = PID(5.0, 5.0, 0.0, 0.0, aimLowerBound, aimUpperBound, -.4, .4)
    val flywheelPID: PID = PID(2.0, 2.0, 0.0, 1.0, 0.0, 2700.0, 0.0, 1.0)

    val flywheelVel = RollingAverage(3)

//    val flywheelDistToVel = interpolatedMapOf(
//        3600.0 to 2300.0,
//        2200.0 to 2000.0,
//        1700.0 to 1860.0,
//        1400.0 to 1740.0,
//        900.0 to 1540.0,
//        400.0 to 1680.0
//    )

    val flywheelDistToVel = interpolatedMapOf(
        3600.0 to 2300.0,
        2200.0 to 1570.0,
        1700.0 to 1420.0,
        1400.0 to 1340.0,
        900.0 to 1140.0,
        400.0 to 1080.0
    )

    val hoodDistToAngle = interpolatedMapOf(
        500.0 to 0.0,
        800.0 to 0.0,
        1400.0 to 1.0,
        1700.0 to 1.0,
        3000.0 to 0.8
    )

    var flywlMod = 0.0
    var aimMod = 0.0
    var hoodMod = 0.0
    var visionTrim = 0.0
    var visionRawTx = 0.0
    var visionExpectedTx = 0.0
    var visionAimError = 0.0

    var isActive = false

    var junkTicks = turret.currentPosition
    var ticks = turret.currentPosition
    var flywheelTicks = flywheel.currentPosition
    var currentAngle = 0.0

    var aimedAt: TimeMark? = null
    var lastFlywheelSampleAt: TimeMark? = null

    init {
        junkTicks = turret.currentPosition
    }

    private fun getFuturePose(): Pose? = odometry?.let { it.position + (it.velocity * .02) }

    private fun getGoalPose() = if (isRed) redGoalPos else blueGoalPos

    private fun getDistanceToGoal(futurePos: Pose): Double {
        return (getGoalPose().position - futurePos.position).magnitude()
    }

    private fun getDistanceToGoalWall(futurePos: Pose): Double {
        return getDistanceToGoal(futurePos) - 700.0
    }

    val currentVelocity: Double
        get() = flywheelVel.average

    val targetVelocity: Double
        get() {
            val futurePos = getFuturePose() ?: return 0.0
            return (flywheelDistToVel[getDistanceToGoalWall(futurePos)] + flywlMod)
        }

    private fun hasFreshVisionMeasurement(measurement: Limelight.AimMeasurement?): Boolean {
        return measurement?.valid == true &&
            measurement.capturedAt?.elapsedNow()?.let { it <= visionMeasurementTimeout } == true
    }

    private fun getExpectedVisionTx(distanceToGoalMm: Double): Double {
        val clampedDistance = distanceToGoalMm.coerceAtLeast(1.0)
        val parallaxTx = Math.toDegrees(atan2(limelightLeftOffsetMm, clampedDistance))
        return parallaxTx + limelightMountYawBiasDegrees
    }

    private fun updateVisionTrim(odometryTarget: Double, distanceToGoalMm: Double) {
        val measurement = limeLight?.aimMeasurement
        val hasFreshMeasurement = hasFreshVisionMeasurement(measurement)
        visionExpectedTx = getExpectedVisionTx(distanceToGoalMm)
        visionRawTx = measurement?.txDegrees ?: 0.0
        visionAimError = visionRawTx - visionExpectedTx
        val canConsumeMeasurement = measurement?.isNewSinceLastConsume == true &&
            hasFreshMeasurement &&
            abs(odometryTarget - currentAngle) <= maxVisionSettleErrorDegrees &&
            abs(turret.velocity) <= maxVisionTurretVelocity

        if (canConsumeMeasurement) {
            limeLight?.consumeAimMeasurement()?.let { freshMeasurement ->
                visionRawTx = freshMeasurement.txDegrees
                visionAimError = visionRawTx - visionExpectedTx
                val boundedTrim = visionAimError.coerceIn(-maxVisionTrimDegrees, maxVisionTrimDegrees)
                visionTrim = (visionTrim * (1.0 - visionBlend)) + (boundedTrim * visionBlend)
            }
            return
        }

        if (!hasFreshMeasurement) {
            visionTrim *= visionDecay
            if (abs(visionTrim) < 0.05) {
                visionTrim = 0.0
            }
        }
    }

    override val apply = SystemCommand.continuous("Shooter Control") {
//        telemetry?.addData("ticks", ticks)

        val futurePos = getFuturePose() ?: return@continuous

        //turret
        val newTicks = turret.currentPosition
        var deltaTicks = newTicks - ticks
        var deltaAngle = (deltaTicks / ticksPerRev) * 360.0
        currentAngle += deltaAngle
        ticks = newTicks

        //Static target position in mm
        val targetX = if (isRed) redGoalPos.x else blueGoalPos.x
        val targetY = if (isRed) redGoalPos.y else blueGoalPos.y

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
        targetAngle = Angle.degrees(targetAngle.degrees.coerceIn(aimLowerBound, aimUpperBound))

        val distanceToGoal = getDistanceToGoal(futurePos)
        updateVisionTrim(targetAngle.degrees, distanceToGoal)
        val aimMeasurement = limeLight?.aimMeasurement
        val hasFreshVision = hasFreshVisionMeasurement(aimMeasurement)
//        if (lmlError.degrees < tolerance) {
//            aimLight.position = .7 // probably add the hood to the condition if we end up using the encoder
//        } else
//            aimLight.position = .4

        telemetry?.addData(" -- shooter futurePose --", futurePos)
        telemetry?.addData(" -- shooter distance to goal --", distanceToGoal)
        telemetry?.addLine("--- aim ---")
        telemetry?.addData("currentAngle", currentAngle)
        telemetry?.addData("visionTrim", visionTrim)
        telemetry?.addData("visionFresh", hasFreshVision)
        telemetry?.addData("visionNew", aimMeasurement?.isNewSinceLastConsume ?: false)
        telemetry?.addData("visionDistance", distanceToGoal)
        telemetry?.addData("visionRawTx", visionRawTx)
        telemetry?.addData("visionExpectedTx", visionExpectedTx)
        telemetry?.addData("visionAimError", visionAimError)

//        telemetry.addData("correctedTarget",correctedTarget)

        val aimTarget = (targetAngle.degrees + visionTrim + aimMod).coerceIn(aimLowerBound, aimUpperBound)

        telemetry?.addData("-aimMod",aimMod)
        telemetry?.addData("-visionTrim",visionTrim)
        telemetry?.addData("-aimTarget",aimTarget)

        val ff = if(abs(aimTarget - currentAngle) > 2) .2 else 0.0
        val pidPow = turretPID.compute(currentAngle, aimTarget)
        turret.ePower = (pidPow + ff.withSign(pidPow))

        telemetry?.addData("aim powa", pidPow)

        telemetry?.addData("ticks", turret.currentPosition)

        //flywheel
//        telemetry?.addLine("--- flywheel ---")

        val currentFlywheelTicks = flywheel.currentPosition
        val flywheelDeltaTicks = currentFlywheelTicks - flywheelTicks
        val flywheelDeltaTime = lastFlywheelSampleAt?.elapsedNow()?.inWholeNanoseconds
        if (flywheelDeltaTime != null && flywheelDeltaTime > 0) {
            flywheelVel += flywheelDeltaTicks * 1_000_000_000.0 / flywheelDeltaTime
        }
        flywheelTicks = currentFlywheelTicks
        lastFlywheelSampleAt = TimeSource.Monotonic.markNow()

        val distToGoalWall = getDistanceToGoalWall(futurePos)
        val desiredVelocity = targetVelocity
        flywheel.ePower =  if (isActive) flywheelPID.compute(flywheelVel.average, desiredVelocity) else 0.0
        // flywheel.ePower = if (isActive) 1.0 else 0.0
        telemetry?.addData("flywheelVel", flywheelVel.average)
        telemetry?.addData("-flywlMod",flywlMod)
        telemetry?.addData("-targetVelocity", desiredVelocity)
        telemetry?.addData("flywheel power", if (isActive) flywheelPID.compute(flywheelVel.average, desiredVelocity) else 0.0)

        // hood
//        telemetry?.addLine("--- hood ---")
//
        val hoodAngle = (hoodDistToAngle[distToGoalWall] + hoodMod)
        telemetry?.addData("-hoodMod",hoodMod)
        telemetry?.addData("-distToGoalWall",distToGoalWall)
        telemetry?.addData("-hoodAngle", hoodDistToAngle[distToGoalWall] + hoodMod)

        hood.ePosition = hoodAngle
    }

    fun togglePower() = SystemCommand.instant("Toggle Shooter Power") {
        isActive = !isActive
    }

    fun start() = SystemCommand.instant("Start Shooter") {
        isActive = true
    }

    fun stop() = SystemCommand.instant("Stop Shooter") {
        isActive = false
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
        hood.position += .1
    }

    fun hoodDown() = SystemCommand.instant("Hood Down") {
        hood.position -= .1
    }

    fun flywheelManualInc() = SystemCommand.instant("Flywheel Manual Inc") {
        flywlMod += 100
    }

    fun flywheelManualDec() = SystemCommand.instant("Flywheel Manual Dec") {
        flywlMod -= 100
    }

    fun aimManualRight() = SystemCommand.instant("Aim Manual Right") {
        aimMod += 5
    }

    fun aimManualLeft() = SystemCommand.instant("Aim Manual Left") {
        aimMod -= 5
    }

    fun hoodManualInc() = SystemCommand.instant("Flywheel Manual Inc") {
        hoodMod += .05
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