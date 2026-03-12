package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import java.util.WeakHashMap
import kotlin.math.abs

inline fun <reified T: HardwareDevice> HardwareMap.getByName(name: String): T {
    return this.get(T::class.java, name)
}

fun Double.eq(other: Double, epsilon: Double) = abs(this - other) < epsilon

private fun shouldWriteHardwareValue(
    current: Double,
    target: Double,
    epsilon: Double,
    forceExactTarget: Boolean
): Boolean {
    if (current == target) return false
    return forceExactTarget || !current.eq(target, epsilon)
}

private val motorPowerCache = WeakHashMap<DcMotorEx, Double>()
private val servoPositionCache = WeakHashMap<Servo, Double>()

var DcMotorEx.ePower
    get() = motorPowerCache[this] ?: this.power.also { motorPowerCache[this] = it }
    set(value) {
        val current = motorPowerCache[this] ?: this.power.also { motorPowerCache[this] = it }
        val forceExactTarget = value == 0.0 || value == 1.0 || value == -1.0
        if (shouldWriteHardwareValue(current, value, 0.01, forceExactTarget)) {
            this.power = value
            motorPowerCache[this] = value
        }
    }

var Servo.ePosition
    get() = servoPositionCache[this] ?: this.position.also { servoPositionCache[this] = it }
    set(value) {
        val current = servoPositionCache[this] ?: this.position.also { servoPositionCache[this] = it }
        val forceExactTarget = value == 0.0 || value == 1.0
        if (shouldWriteHardwareValue(current, value, 0.001, forceExactTarget)) {
            this.position = value
            servoPositionCache[this] = value
        }
    }