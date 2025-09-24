package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorImplEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType

class Motor(
    controller: DcMotorController,
    portNumber: Int,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
    motorType: MotorConfigurationType = MotorConfigurationType.getUnspecifiedMotorType()
) : DcMotorImplEx(controller, portNumber, direction, motorType) {
    var lastSetPower: Double = 0.0
        private set

    override fun setPower(power: Double) {
        if (lastSetPower != power)
            super.setPower(power)
        lastSetPower = power
    }
}