package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.core.commands.Command
import io.github.bionictigers.axiom.core.commands.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Sorter

@TeleOp
class VisionTest : LinearOpMode() {
    override fun runOpMode() {
        Scheduler.telemetry = telemetry

        val vision = Sorter(hardwareMap, telemetry)
        Scheduler.schedule(vision)
        Scheduler.schedule(Command.continuous {
            vision.seeApril()
            telemetry.addData("detection", vision.obeliskCode.name)
        })

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            telemetry.update()
        }
        Scheduler.reset()
    }
}




