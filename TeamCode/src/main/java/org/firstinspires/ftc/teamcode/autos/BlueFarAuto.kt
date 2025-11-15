package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.core.commands.Scheduler
import io.github.bionictigers.axiom.core.commands.groups.concurrent
import io.github.bionictigers.axiom.core.commands.groups.sequential
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.motion.Odometry
import org.firstinspires.ftc.teamcode.utils.Pose
import kotlin.time.Duration.Companion.seconds

@Autonomous(name = "BlueFar")
class BlueFarAuto : LinearOpMode() {
    override fun runOpMode() {
        val odometry = Odometry(hardwareMap, telemetry, Pose(0, 0, 0))
        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry)
        val intake = Intake(hardwareMap, drivetrain)
        val kicker = Kicker(hardwareMap)
        val sorter = Sorter(hardwareMap, kicker, telemetry)
        val output = Output(hardwareMap, telemetry)

        val backUp = sequential {
            concurrent {
                add(kicker.down())
                add(intake.intake())
                add(output.shoot())
            }
            wait(7.seconds)
            add(kicker.kick())
            wait(1.seconds)
            add(kicker.kick())
            wait(1.seconds)
            add(sorter.moveBackward())
            wait(1.seconds)
            add(kicker.kick())
            wait(1.seconds)
            add(kicker.kick())
            wait(1.seconds)
            concurrent {
                add(output.stop())
                add(drivetrain.mtpNoProfile(Pose(500, 400, 0.0)))
            }
        }

        Scheduler.telemetry = telemetry
        Scheduler.schedule(odometry, drivetrain, intake, kicker, sorter, output)
        Scheduler.schedule(backUp )

        waitForStart()

        while (opModeIsActive()) {
            Scheduler.update()
            telemetry.update()
        }

        Scheduler.reset()
    }

}