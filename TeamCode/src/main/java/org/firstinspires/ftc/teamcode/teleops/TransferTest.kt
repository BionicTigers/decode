package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.profiles.BaseProfile

@TeleOp(name = "Test")
class Test : LinearOpMode() {
    override fun runOpMode() {
        val intake = Intake(hardwareMap)
        val sorter = Sorter(hardwareMap, telemetry = telemetry)
        val controls = Controls(gamepad1, gamepad1, BaseProfile.default, BaseProfile.default,
            listOf(intake, sorter)
        )

        Scheduler.telemetry = telemetry
        Scheduler.schedule(controls, intake, sorter)

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.tick()
            telemetry.update()
        }

        Scheduler.clear()
    }
}