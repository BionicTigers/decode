//package org.firstinspires.ftc.teamcode.teleops
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import io.github.bionictigers.axiom.core.input.Controls
//import io.github.bionictigers.axiom.core.scheduler.Scheduler
//import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
//import org.firstinspires.ftc.teamcode.mechanisms.Intake
//import org.firstinspires.ftc.teamcode.mechanisms.Kicker
//import org.firstinspires.ftc.teamcode.mechanisms.MotorRun
//import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
//import org.firstinspires.ftc.teamcode.mechanisms.Odometry
//import org.firstinspires.ftc.teamcode.mechanisms.Output
//import org.firstinspires.ftc.teamcode.mechanisms.Persistents
//import org.firstinspires.ftc.teamcode.mechanisms.Sorter
//import org.firstinspires.ftc.teamcode.profiles.BaseProfile
//import org.firstinspires.ftc.teamcode.utils.milliseconds
//import kotlin.time.TimeSource
//import kotlin.time.measureTime
//
//@TeleOp(name = "mtor run")
//class Motor : LinearOpMode()  {
//    override fun runOpMode() {
//        val motor = MotorRun(hardwareMap)
//        val controls = Controls(gamepad1, gamepad2, BaseProfile.default, BaseProfile.default,
//            listOf(motor)
//        )
//
//        Scheduler.telemetry = telemetry
//        Scheduler.schedule(motor, controls)
//
//        waitForStart()
//        while (opModeIsActive()) {
//            telemetry.addData("Time Taken", measureTime {
//                Scheduler.tick()
//            }.milliseconds)
//            telemetry.update()
//        }
//
//        Scheduler.clear()
//    }
//}