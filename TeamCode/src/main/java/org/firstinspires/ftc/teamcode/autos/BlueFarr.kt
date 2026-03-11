package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Pose

@Autonomous
class BlueFarr: LinearOpMode() {
    val startPose = Pose(0, 0, 90)
    val endPose = Pose(600, 600, Angle.degrees(0))

    override fun runOpMode() {
        val odometry = Odometry(hardwareMap, telemetry, startPose)
        val octoQuad = OctoQuad(hardwareMap, null)

        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry, octoQuad)
//        val intake = Intake(hardwareMap, octoQuad)
//        val kicker = Kicker(hardwareMap)
//        val sorter = Sorter(hardwareMap, kicker, telemetry, octoQuad)
//        val output = Output(hardwareMap, kicker, sorter, telemetry, odometry, octoQuad, false)

//        Scheduler.schedule(odometry, drivetrain, intake, kicker, sorter, output)
        Scheduler.schedule(odometry, drivetrain)

        waitForStart()

        Scheduler.schedule(drivetrain.moveToPosition(endPose))

//        sorter.move()

        while (opModeIsActive()) {
            Scheduler.tick()
            telemetry.update()
        }

        Scheduler.clear()
    }
}
