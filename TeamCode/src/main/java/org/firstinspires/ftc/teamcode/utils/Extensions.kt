package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs

inline fun <reified T: HardwareDevice> HardwareMap.getByName(name: String): T {
    return this.get(T::class.java, name)
}

fun Double.eq(other: Double, epsilon: Double) = abs(this - other) < epsilon

var DcMotorEx.ePower
    get() = this.power
    set(value) {
        if (!this.power.eq(value, .001)
            || value == 0.0 && this.power != 0.0
            || value == 1.0 && this.power != 1.0
            || value == -1.0 && this.power != -1.0
        ) {
            this.power = value
        }
    }