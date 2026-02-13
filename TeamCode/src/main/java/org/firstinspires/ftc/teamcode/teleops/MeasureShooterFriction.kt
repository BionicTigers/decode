package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.profiles.OutputTune
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.Pose
import org.firstinspires.ftc.teamcode.utils.milliseconds
import kotlin.time.measureTime

@TeleOp(name = "Shooter Friction")
class MeasureShooterFriction: LinearOpMode() {
    override fun runOpMode() {
        val odometry = Odometry(hardwareMap, null, Pose(Distance.inch(9).mm, 609.6 * 3.5, 90))
        val kicker = Kicker(hardwareMap)
        val octoquad = OctoQuad(hardwareMap, null)
        val sorter = Sorter(hardwareMap, kicker, null, octoquad)
        val output = Output(hardwareMap, kicker, sorter, telemetry, odometry, octoquad, false)
        val controls = Controls(gamepad1, gamepad2, OutputTune, BaseProfile.default,
            listOf(output, kicker, sorter)
        )

        Scheduler.telemetry = telemetry
        Scheduler.schedule(controls, output, kicker, sorter, octoquad)

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