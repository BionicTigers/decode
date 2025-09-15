package org.firstinspires.ftc.teamcode.control

import kotlin.math.sign

object Feedforward {
    fun basic(kS: Double, kV: Double, kA: Double): (Double, Double) -> Double {
        return { velocity: Double, acceleration: Double ->
            kS * sign(velocity) + kV * velocity + kA * acceleration
        }
    }
}