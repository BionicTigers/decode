package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.core.commands.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.motion.Odometry
import org.firstinspires.ftc.teamcode.utils.Pose

@Autonomous(name = "LQR Test", group = "autonomous")
class LQRTest : LinearOpMode() {
    override fun runOpMode() {
        val odometry = Odometry(hardwareMap, telemetry)
        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry)
        val pose = Pose(200.0, 200.0, 0.0)

        waitForStart()
        Scheduler.schedule(drivetrain.moveToPosition(pose))
        while (opModeIsActive()) {
            Scheduler.update()
            telemetry.update()
        }
    }
}