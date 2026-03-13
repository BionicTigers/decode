package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import io.github.bionictigers.axiom.core.input.Controls
import io.github.bionictigers.axiom.core.input.Gamepads
import io.github.bionictigers.axiom.core.scheduler.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Limelight
import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
import org.firstinspires.ftc.teamcode.mechanisms.Odometry
import org.firstinspires.ftc.teamcode.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.mechanisms.Transfer
import org.firstinspires.ftc.teamcode.profiles.BaseProfile
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.Pose
import kotlin.time.measureTime

@TeleOp(name = "Robot Stuff")
class FullRobotTest : LinearOpMode() {
    override fun runOpMode() {
        val octoQuad = OctoQuad(hardwareMap, null)
        val transfer = Transfer(hardwareMap, octoQuad, null)
        val intake = Intake(hardwareMap)
        val limelight = Limelight(hardwareMap, telemetry, false)
        val odometry = Odometry(hardwareMap, telemetry,
            Pose(Distance.inch(14/2).mm, 609.6 * 4 - 609.6 * .5, 90.0)
        )

        val drivetrain = Drivetrain(hardwareMap, null, odometry, octoQuad)
        val shooter = Shooter(hardwareMap, odometry, limelight, telemetry, false)
        val controls = Controls(gamepad1, gamepad2, BaseProfile.default, BaseProfile.default, listOf(transfer, intake, drivetrain, shooter))
        val shooterRumbleGamepad: Gamepad = when (BaseProfile.default.shooter.desiredGamepad) {
            Gamepads.GAMEPAD_2 -> gamepad2
            else -> gamepad1
        }

        Scheduler.schedule(octoQuad, transfer, controls, drivetrain, intake, shooter, odometry, limelight)

        waitForStart()
        var wasShotReady = false

        while (opModeIsActive()) {
            val time = measureTime {
                Scheduler.tick()
            }
            if (shooter.shotReady && !wasShotReady) {
                shooterRumbleGamepad.rumble(250)
            }
            wasShotReady = shooter.shotReady
            telemetry.addData("Loop Time", time)
            telemetry.update()
        }
    }
}