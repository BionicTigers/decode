package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.Pose

class BlueFarNoTree: LinearOpMode() {
    private val startPose = Pose(Distance.inch(9).mm, 609.6 * 3.5, 90)
    private val shootClosePose = Pose(609.6 * 3.5, 609.6 * 3.5, 90 + 45)
    private val queueClosePose = Pose(609.6 * 1.5, 609.6 * 4, 0)
    private val grabClosePose = Pose(609.6 * 1.5, 609.6 * 5.5, 0)
    private val queueMiddlePose = Pose(1211.37, 2125.95, 0)
    private val grabMiddlePose = Pose(390.60, 2128.79, 0)
    private val queueFarPose = Pose(989.85, 1509.67, 0)
    private val grabFarPose = Pose(384.92, 1523.87, 0)

    override fun runOpMode() {
        val octoQuad = OctoQuad(hardwareMap, telemetry)
        val odometry = Odometry(hardwareMap, telemetry, startPose)
        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry, octoQuad)
        val intake = Intake(hardwareMap, octoQuad)
        val kicker = Kicker(hardwareMap, telemetry)
        val sorter = Sorter(hardwareMap, kicker, telemetry, octoQuad)
        val output = Output(hardwareMap, kicker, sorter, telemetry, odometry, octoQuad, false)

        kicker.servo.position = 0.5

        Scheduler.telemetry = telemetry
        Scheduler.schedule(octoQuad, odometry, drivetrain, intake, kicker, sorter, output)


    }
}