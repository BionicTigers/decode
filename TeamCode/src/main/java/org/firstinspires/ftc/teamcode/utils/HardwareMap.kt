package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap

inline fun <reified T: HardwareDevice> HardwareMap.getByName(name: String): T {
    return this.get(T::class.java, name)
}