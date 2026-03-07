package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
import org.firstinspires.ftc.teamcode.utils.getByName
import org.firstinspires.ftc.teamcode.utils.seconds
import kotlin.math.max
import kotlin.time.TimeSource

@TeleOp
class SorterTest : LinearOpMode() {
    override fun runOpMode() {
        val sorterMotor = hardwareMap.getByName<DcMotorEx>("sorter")
        val octoQuad = OctoQuad(hardwareMap, telemetry)
        val TICKS_PER_REV = 8192.0

        Scheduler.schedule(octoQuad)
        waitForStart()

        var lastTick = octoQuad.encoderData.position[5]
        var lastAngle = 0.0
        var lastVelocity = 0.0
        var maxVelocity = 0.0
        var maxAcceleration = 0.0
        var lastTime = TimeSource.Monotonic.markNow()

        while (opModeIsActive()) {
            Scheduler.tick()
            if (gamepad1.circle) {
                sorterMotor.power = 1.0
            } else if (gamepad1.cross) {
                sorterMotor.power = 0.0
            }

            val dt = lastTime.elapsedNow()
            val tick = octoQuad.encoderData.position[5].toDouble()
            val angle = (tick - lastTick) / 8192 * 360
            val velocity = (angle - lastAngle) / dt.seconds
            val acceleration = (velocity - lastVelocity) / dt.seconds
            lastVelocity = velocity
            maxVelocity = max(velocity, maxVelocity)
            maxAcceleration = max(acceleration, maxAcceleration)
            lastAngle = angle
            lastTick = tick.toInt()
            lastTime = TimeSource.Monotonic.markNow()

            telemetry.addData("maxVelocity", maxVelocity)
            telemetry.addData("maxAcceleration", maxAcceleration)
            telemetry.update()
        }
    }
}