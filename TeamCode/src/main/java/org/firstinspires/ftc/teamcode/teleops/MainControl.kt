package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Pose
import kotlin.time.TimeSource

@TeleOp(name = "Main Control")
class MainControl : LinearOpMode() {
    override fun runOpMode() {
        val b = TimeSource.Monotonic.markNow()
        val odometry = Odometry(hardwareMap, telemetry, Pose(0, 0, 0))
        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry)
        val intake = Intake(hardwareMap, drivetrain)
        val kicker = Kicker(hardwareMap)
        val sorter = Sorter(hardwareMap, kicker, telemetry)
        val output = Output(hardwareMap, kicker, telemetry, odometry)
        val controls = Controls(gamepad1, gamepad2, BaseProfile.default, BaseProfile.default,
            listOf(drivetrain, intake, output, sorter, kicker)
        )

        Scheduler.telemetry = telemetry
        Scheduler.schedule(drivetrain, controls, intake, sorter, output, kicker)

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.tick()
            telemetry.update()
        }

        Scheduler.clear()
    }
}