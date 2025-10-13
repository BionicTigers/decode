package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.core.commands.Scheduler
import io.github.bionictigers.axiom.core.input.Controls
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.motion.Odometry
import org.firstinspires.ftc.teamcode.profiles.BaseProfile

@TeleOp(name = "Anything please")
class MainControl : LinearOpMode() {
    override fun runOpMode() {
        val odometry = Odometry(hardwareMap, telemetry)
        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry)
        val intake = Intake(hardwareMap)
        val output = Output(hardwareMap)
        val controls = Controls(gamepad1, gamepad2, BaseProfile.default, BaseProfile.default,
            listOf(drivetrain, intake, output)
        )

        Scheduler.telemetry = telemetry
        Scheduler.schedule(drivetrain, controls, odometry, intake, output)

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            telemetry.update()
        }
    }
}