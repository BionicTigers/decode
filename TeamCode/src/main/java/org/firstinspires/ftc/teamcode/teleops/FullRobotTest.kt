package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.mechanisms.Transfer
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.time.measureTime

@TeleOp()
class FullRobotTest : LinearOpMode() {
    override fun runOpMode() {
        val octoQuad = OctoQuad(hardwareMap, null)
        val transfer = Transfer(hardwareMap, octoQuad, null)
        val intake = Intake(hardwareMap)
        val odometry = Odometry(hardwareMap, telemetry)
        val drivetrain = Drivetrain(hardwareMap, null, odometry, octoQuad)
        val shooter = Shooter(hardwareMap, odometry, null, telemetry, true)
        val controls = Controls(gamepad1, gamepad2, BaseProfile.default, BaseProfile.default, listOf(transfer, intake, drivetrain, shooter))

        Scheduler.schedule(octoQuad, transfer, controls, drivetrain, intake, shooter, odometry)

        waitForStart()

        while (opModeIsActive()) {
            val time = measureTime {
                Scheduler.tick()
            }
            telemetry.addData("Loop Time", time)
            telemetry.update()
        }
    }
}