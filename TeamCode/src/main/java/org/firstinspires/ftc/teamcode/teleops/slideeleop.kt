package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.core.commands.Scheduler
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import org.firstinspires.ftc.teamcode.mechanisms.MilesIntake
import org.firstinspires.ftc.teamcode.mechanisms.MilesSlides
import org.firstinspires.ftc.teamcode.profiles.BaseProfile

@TeleOp(name = "miles slidesw")
class slideeleop(): LinearOpMode() {

    override fun runOpMode() {
        val milesSlides = MilesSlides(hardwareMap)
        val milesIntake = MilesIntake(hardwareMap)
        val controls = Controls(gamepad1, gamepad2, BaseProfile.default, BaseProfile.default,
            listOf(milesSlides,milesIntake)
        )
re
        Scheduler.schedule(controls,milesSlides,milesIntake)
        waitForStart()
        while (opModeIsActive()) {
            Scheduler.update()
        }
    }
}