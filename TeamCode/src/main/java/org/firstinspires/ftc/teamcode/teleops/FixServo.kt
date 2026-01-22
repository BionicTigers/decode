package org.firstinspires.ftc.teamcode.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.utils.getByName

@TeleOp
class FixServo : LinearOpMode() {
    override fun runOpMode() {
        val turret = hardwareMap.getByName<Servo>("turret")
        waitForStart()
        while (opModeIsActive())
            turret.position = 0.5
    }
}