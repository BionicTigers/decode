package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.milliseconds
import kotlin.time.TimeSource
import kotlin.time.measureTime

@TeleOp(name = "Main Control")
class MainControl : LinearOpMode() {
    override fun runOpMode() {
        val b = TimeSource.Monotonic.markNow()
        val odometry = Odometry(hardwareMap, null, Pose(0, 0, 0))
        val octoquad = OctoQuad(hardwareMap, telemetry)
        val drivetrain = Drivetrain(hardwareMap, null, odometry, octoquad)
        val intake = Intake(hardwareMap, octoquad)
        val kicker = Kicker(hardwareMap)
        val sorter = Sorter(hardwareMap, kicker, null, octoquad)
        val output = Output(hardwareMap, kicker, sorter, telemetry, odometry, octoquad)
        val controls = Controls(gamepad1, gamepad2, BaseProfile.default, BaseProfile.default,
            listOf(drivetrain, intake, output, sorter, kicker)
        )

        Scheduler.telemetry = telemetry
        Scheduler.schedule(drivetrain, controls, intake, sorter, output, kicker, octoquad, odometry)

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Time Taken", measureTime {
                Scheduler.tick()
            }.milliseconds)
            telemetry.update()
        }

        Scheduler.clear()
    }
}