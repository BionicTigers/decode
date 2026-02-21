package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.LimeLight
import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.milliseconds
import kotlin.time.measureTime

@Autonomous
class BlueFarSimple : LinearOpMode() {

    override fun runOpMode() {
        val odometry = Odometry(hardwareMap, telemetry, Pose(0, 0, 0))
        val octoquad = OctoQuad(hardwareMap, telemetry)
        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry, octoquad)
        val intake = Intake(hardwareMap, octoquad)
        val kicker = Kicker(hardwareMap)
        val sorter = Sorter(hardwareMap, kicker, null, octoquad)
        val limeLight = LimeLight(hardwareMap, telemetry )
        val output = Output(hardwareMap, kicker, sorter, null, odometry, octoquad, false, limeLight)

        Scheduler.telemetry = telemetry
        Scheduler.schedule(odometry, drivetrain, intake, kicker, sorter, output)

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.tick()
            telemetry.update()
        }

        Scheduler.clear()

    }
}