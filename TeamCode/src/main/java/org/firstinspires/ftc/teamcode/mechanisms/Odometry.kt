package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.System
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.Matrix
import org.firstinspires.ftc.teamcode.utils.getByName

interface RobotConfig {
    /**
     * How far sideways from the center the forward (X) odometry pod is.
     * Left of center is positive, right is negative.
     */
    val forwardOffset: Distance

    /**
     * How far forwards from the center the strafe (Y) odometry pod is.
     * Forward of center is positive, backwards is negative.
     */
    val strafeOffset: Distance
}

object Configs {
    object Test: RobotConfig {
        override val forwardOffset: Distance = Distance.inch((6.9))
        override val strafeOffset: Distance = Distance.inch((-3.75))
    }

    object Main: RobotConfig {
        override val forwardOffset: Distance = Distance.inch(6.3125)
        override val strafeOffset: Distance = Distance.inch(-2.5)
    }
}

class Odometry(
    hardwareMap: HardwareMap,
    val telemetry: Telemetry? = null,
    val startPose: Pose = Pose(0.0, 0.0, Angle.radians(0.0)),
    config: RobotConfig = Configs.Main,
    override val name: String = "Odometry"
) : System() {
    private val pinpoint = hardwareMap.getByName<GoBildaPinpointDriver>("pinpoint")

    var position = startPose
    var velocity = Pose(0.0, 0.0, 0.0)
    var state = Matrix(
        arrayOf(
            arrayOf(position.x),
            arrayOf(position.y),
            arrayOf(position.radians),
            arrayOf(velocity.x),
            arrayOf(velocity.y),
            arrayOf(velocity.radians)
        )
    )

    init {
        pinpoint.setOffsets(config.forwardOffset.mm, config.strafeOffset.mm, DistanceUnit.MM)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD)
        pinpoint.recalibrateIMU()
        pinpoint.setPosition(Pose2D(DistanceUnit.MM,startPose.y, -startPose.x, AngleUnit.DEGREES, -startPose.degrees))
    }

    /**
     * Takes .25 seconds, robot must be stationary for the whole .25 seconds.
     * It is recommended that the imu is recalibrated before each use (at the start of an auto or teleop).
     */
    val recalibrate = SystemCommand.instant("Odometry Recalibrate") {
        pinpoint.recalibrateIMU()
    }

    override val update = SystemCommand.continuous("Odometry Update") {
        pinpoint.update()

        position = Pose(
            -pinpoint.getPosY(DistanceUnit.MM),
            pinpoint.getPosX(DistanceUnit.MM),
            -Angle.radians(pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS))
        )

        velocity = Pose( // values are global
            -pinpoint.getVelY(DistanceUnit.MM),
            pinpoint.getVelX(DistanceUnit.MM),
            -Angle.radians(pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS))
        )
        log()
    }

    fun setPose(newPose: Pose) {
        pinpoint.setPosition(Pose2D(DistanceUnit.MM,newPose.y, -newPose.x, AngleUnit.DEGREES, -newPose.degrees))
    }

    fun log() {
        telemetry?.addData("x", position.x)
        telemetry?.addData("y", position.y)
        telemetry?.addData("rot", position.rotation.degrees.normalizeDegrees())
//        telemetry?.addData("x vel", velocity.x)
//        telemetry?.addData("y vel", velocity.y)
//        telemetry?.addData("rot vel", velocity.rotation.degrees.normalizeDegrees())
    }

    private var loopTimeTooLowWarning: Telemetry.Line? = null
    private var loopTimeTooHighWarning: Telemetry.Line? = null

//    override val apply = SystemCommand.continuous("Odometry Check") {
//        if (pinpoint.loopTime < 500)
//            if (loopTimeTooLowWarning == null)
//                loopTimeTooLowWarning = telemetry?.addLine("Loop time is lower than 500ms, odometry may not be accurate/something may be wrong with pods according to the GoBilda documentation.")
//            else
//                loopTimeTooLowWarning?.let {
//                    telemetry?.removeLine(loopTimeTooLowWarning)
//                    loopTimeTooLowWarning = null
//                }
//
//        if (pinpoint.loopTime > 1100)
//            if (loopTimeTooHighWarning == null)
//                loopTimeTooHighWarning = telemetry?.addLine("Loop time is higher than 1100ms, odometry may not be accurate/something may be wrong with pods according to the GoBilda documentation.")
//            else
//                loopTimeTooHighWarning?.let {
//                    telemetry?.removeLine(it)
//                    loopTimeTooHighWarning = null
//                }
//    }
}

fun Double.normalizeDegrees() : Double {
    var angle = this % 360
    if (angle < 0) {
        angle += 360.0
    }
    return angle
}