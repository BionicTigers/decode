package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import io.github.bionictigers.axiom.core.commands.Command
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
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.ControlHub
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.TimeMark
import kotlin.time.TimeSource

class Shooter(hardware: HardwareMap, val octoQuad: OctoQuad, val odometry: Odometry?, val telemetry: Telemetry, val isRed: Boolean) : System(), Controllable<BaseProfile> {
    override val name = "Shooter"

    interface Schema : ControlSchema {
        val togglePower: Digital?
        val aimLeft: Digital?
        val aimRight: Digital?
        val hoodUp: Digital?
        val hoodDown: Digital?
    }

    val redGoalPos = Pose(3657.6 - 50,0.0, 0.0)
    val blueGoalPos = Pose(3627.6 - 50,3627.6, 0.0)

    val ticksPerRev = 1.0

    val flywheelMotor = hardware.getByName<DcMotorEx>("shooter")
    val aimMotor = hardware.getByName<DcMotorEx>("shooterAim")
    val hoodServo = hardware.getByName<Servo>("hood")

    var aimedAt: TimeMark? = null

//    override val apply = SystemCommand.continuous {
//        val aimAngle = aimMotor.currentPosition / ticksPerRev * 360.0
//        //Static target position in mm
//        val targetX = if (isRed) redGoalPos.x else blueGoalPos.x
//        val targetY = if (isRed) redGoalPos.y else blueGoalPos.y
//
//        // Calculate angle from robot to target in field coordinates
//        // Using atan2(deltaX, deltaY) because robot heading 0° = positive Y, 90° = positive X
//        val deltaX = targetX - odometry!!.position.x
//        val deltaY = targetY - odometry.position.y
//        val angleToTarget = atan2(deltaX, deltaY) // radians
//
//        // Get robot's current heading in radians
//        val robotHeading = odometry.position.radians
//
//        // Calculate relative angle (how much turret needs to turn from robot's forward direction)
//        var relativeAngle = angleToTarget - robotHeading
//
//        // Normalize to [-π, π] so turret takes the shortest path
//        while (relativeAngle > PI) relativeAngle -= 2 * PI
//        while (relativeAngle < -PI) relativeAngle += 2 * PI
//
//
//
//        telemetry?.addData("Angle", Math.toDegrees(relativeAngle))
//        telemetry?.addData("Servo Position", servoPosition)
//        telemetry?.addData("servo position", servoPositionTx)
//
////        if (isRed)
////            turret.position = servoPosition
////        else
////            turret.position = servoPosition + .02
////        turret.position = 0.5
//
//        if (aimedAt == null || aimedAt!!.elapsedNow() > 75.milliseconds) {
//            aimMotor.power = 0.0
//        }
//    }

    fun togglePower() = SystemCommand.instant("Toggle Shooter Power") {
        if (flywheelMotor.power == 0.0) {
            flywheelMotor.power = 1.0
        } else {
            flywheelMotor.power = 0.0
        }
    }



    fun aimLeft() = SystemCommand.instant("Aim Left") {
        aimMotor.power = -1.0
        aimedAt = TimeSource.Monotonic.markNow()
    }

    fun aimRight() = SystemCommand.instant("Aim Right") {
        aimMotor.power = 1.0
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


}