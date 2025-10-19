//package org.firstinspires.ftc.teamcode.teleops
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import io.github.bionictigers.axiom.core.commands.Scheduler
//import io.github.bionictigers.axiom.core.input.Controls
//import io.github.bionictigers.axiom.core.input.Gamepads
//import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
//import org.firstinspires.ftc.teamcode.mechanisms.Intake
//import org.firstinspires.ftc.teamcode.mechanisms.Output
//import org.firstinspires.ftc.teamcode.mechanisms.Sorter
//import org.firstinspires.ftc.teamcode.motion.Odometry
//import org.firstinspires.ftc.teamcode.profiles.BaseProfile
//
//@TeleOp
//class Main: LinearOpMode() {
//    override fun runOpMode() {
//        val odometry = Odometry(hardwareMap, telemetry)
//        val drivetrain = Drivetrain(hardwareMap, telemetry)
//        val output = Output(hardwareMap)
//        val sorter = Sorter(hardwareMap)
//        val intake = Intake(hardwareMap)
//
//        val controls = Controls(gamepad1, gamepad2, BaseProfile.default, BaseProfile.default,
//            listOf(drivetrain, intake, output)
//        )
//        Scheduler.schedule(odometry, drivetrain, output, sorter, intake, controls)
//
//        waitForStart()
//
//        while (opModeIsActive()) {
//            Scheduler.update()
//        }
//
//        Scheduler.reset()
//    }
//}