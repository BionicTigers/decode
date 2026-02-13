//package org.firstinspires.ftc.teamcode.teleops
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import io.github.bionictigers.axiom.core.scheduler.Scheduler
//import org.firstinspires.ftc.teamcode.mechanisms.LimeLight
//
//@TeleOp(name = "Limeop")
//class LimeOp: LinearOpMode() {
//    override fun runOpMode() {
//        val limeLight = LimeLight(hardwareMap, telemetry)
//        Scheduler.telemetry = telemetry
//        Scheduler.schedule(limeLight)
//
//        waitForStart()
//
//        while (opModeIsActive()) {
//            Scheduler.tick()
//            telemetry.update()
//        }
//        Scheduler.clear()
//    }
//
//}