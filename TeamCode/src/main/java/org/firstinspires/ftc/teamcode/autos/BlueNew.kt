package org.firstinspires.ftc.teamcode.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import io.github.bionictigers.axiom.core.commands.Scheduler
import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
import org.firstinspires.ftc.teamcode.mechanisms.Intake
import org.firstinspires.ftc.teamcode.mechanisms.Kicker
import org.firstinspires.ftc.teamcode.mechanisms.Output
import org.firstinspires.ftc.teamcode.mechanisms.Sorter
import org.firstinspires.ftc.teamcode.motion.Odometry
import org.firstinspires.ftc.teamcode.utils.Pose

//@Autonomous(name = "BlueFar")
//class BlueFarAuto : LinearOpMode() {
//    override fun runOpMode() {
//        val odometry = Odometry(hardwareMap, telemetry, Pose(0, 0, 0))
//        val drivetrain = Drivetrain(hardwareMap, telemetry, odometry)
//        val intake = Intake(hardwareMap, drivetrain)
//        val kicker = Kicker(hardwareMap)
//        val sorter = Sorter(hardwareMap, kicker, telemetry)
//        val output = Output(hardwareMap, telemetry)
//
//        Scheduler.schedule(odometry, drivetrain, intake, kicker, sorter, output)
//
//
//    }
//}