package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.System

class Drivetrain(hardwareMap: HardwareMap) : System() {
    override val name = "Drivetrain"

    private data class DriveMotors(
        val frontLeft: DcMotorEx,
        val frontRight: DcMotorEx,
        val backLeft: DcMotorEx,
        val backRight: DcMotorEx
    ) {

    }
}