package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Angle
import org.firstinspires.ftc.teamcode.utils.Pose

@TeleOp(name = "Drive")
class Drive : LinearOpMode() {
    override fun runOpMode() {
        val odometry = Odometry(hardwareMap, telemetry, Pose(2023.872,517.65,Angle.degrees(0)))
        val output = Output(hardwareMap, kicker = null, telemetry, odometry)
        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry)
        val controls = Controls(gamepad1, gamepad2, BaseProfile.default, BaseProfile.default,
            listOf(drivetrain, output)
        )

        Scheduler.telemetry = telemetry
        Scheduler.schedule(drivetrain, controls, odometry, output)

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.tick()
            telemetry.update()
        }
        Scheduler.clear()
    }
}