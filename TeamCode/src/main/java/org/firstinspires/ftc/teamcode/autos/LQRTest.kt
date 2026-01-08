package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.utils.Pose

@Autonomous(name = "LQR Test", group = "autonomous")
class LQRTest : LinearOpMode() {
    override fun runOpMode() {
        val odometry = Odometry(hardwareMap, telemetry, Pose(0.0,0.0,0.0))
        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry)
        val pose = Pose(0.0, 600.0, 0.0)
        Scheduler.schedule(drivetrain, odometry)

        waitForStart()
        Scheduler.schedule(drivetrain.moveToPosition(pose))
        while (opModeIsActive()) {
            Scheduler.tick()
            telemetry.update()
        }

        drivetrain.pids.clear()
        Scheduler.clear()
    }
}