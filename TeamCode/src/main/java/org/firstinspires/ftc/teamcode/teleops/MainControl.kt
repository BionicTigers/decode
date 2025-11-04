package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.core.commands.Scheduler
import io.github.bionictigers.axiom.core.input.Controls
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.motion.Odometry
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import kotlin.time.TimeMark
import kotlin.time.TimeSource
import kotlin.time.measureTime

@TeleOp(name = "Main Control")
class MainControl : LinearOpMode() {
    override fun runOpMode() {
            Scheduler.reset()

//        val odometry = Odometry(hardwareMap, telemetry)
        val b = TimeSource.Monotonic.markNow()
        val drivetrain = Drivetrain(hardwareMap, telemetry)
        val intake = Intake(hardwareMap, drivetrain)
        val kicker = Kicker(hardwareMap)
        val sorter = Sorter(hardwareMap, kicker, telemetry)
        val output = Output(hardwareMap)
        val controls = Controls(gamepad1, gamepad2, BaseProfile.default, BaseProfile.default,
            listOf(drivetrain, intake, output, sorter, kicker)
        )

        Scheduler.telemetry = telemetry
        Scheduler.schedule(drivetrain, controls, intake, sorter, output, kicker)

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            telemetry.update()
        }

        Scheduler.reset()
    }
}