//package org.firstinspires.ftc.teamcode.teleops
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import io.github.bionictigers.axiom.core.commands.System
//import io.github.bionictigers.axiom.core.scheduler.Scheduler
//import io.github.bionictigers.axiom.core.web.Editable
//import io.github.bionictigers.axiom.core.web.Server
//
//private class TestSystem() : System() {
//    override val name: String = "Test System"
//
//    var testValue = 0
//    @Editable
//    var message = "Hello, Axiom!"
//
//    override val update = SystemCommand.continuous("Test Update") {
//        testValue++
//    }
//}
//
//@TeleOp
//class AxiomTest : LinearOpMode() {
//    override fun runOpMode() {
//        val testSystem = TestSystem()
//
//        Scheduler.schedule(testSystem)
//
//        waitForStart()
//
//        while (opModeIsActive()) {
//            Scheduler.tick()
//            telemetry.addData("Test Value", testSystem.testValue)
//            telemetry.addData("Message", testSystem.message)
//            telemetry.update()
//        }
//
//        Scheduler.clear()
//    }
//}