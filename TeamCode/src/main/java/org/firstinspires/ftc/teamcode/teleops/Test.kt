//package org.firstinspires.ftc.teamcode.teleops
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import io.github.bionictigers.axiom.core.input.Controls
//import io.github.bionictigers.axiom.core.scheduler.Scheduler
//import org.firstinspires.ftc.teamcode.mechanisms.Drivetrain
//import org.firstinspires.ftc.teamcode.mechanisms.Intake
//import org.firstinspires.ftc.teamcode.mechanisms.LimeLight
//import org.firstinspires.ftc.teamcode.mechanisms.OctoQuad
//import org.firstinspires.ftc.teamcode.mechanisms.Output
//import org.firstinspires.ftc.teamcode.mechanisms.Sorter
//import org.firstinspires.ftc.teamcode.mechanisms.Odometry
//import org.firstinspires.ftc.teamcode.mechanisms.Persistents
//import org.firstinspires.ftc.teamcode.profiles.BaseProfile
//import org.firstinspires.ftc.teamcode.utils.milliseconds
//import kotlin.time.TimeSource
//import kotlin.time.measureTime
//
//@TeleOp(name = "TestUrServo")
//class Test: LinearOpMode() {
//    override fun runOpMode() {
//        val octoquad = OctoQuad(hardwareMap, null)
//        val limeLight = LimeLight(hardwareMap, telemetry )
//
//        val output = Output(hardwareMap, null, null, null, null, octoquad, true, limeLight)
//        val controls = Controls(gamepad1, gamepad2, BaseProfile.default, BaseProfile.default,
//            listOf(output)
//        )
//        Scheduler.telemetry = telemetry
//        Scheduler.schedule( controls,output, octoquad, limeLight)
//        waitForStart()
//
//        while (opModeIsActive()) {
//            Scheduler.tick
//            telemetry.update()
//        }
//
//        Scheduler.clear()
//    }
//}
//
//
