package org.firstinspires.ftc.teamcode.control

import io.github.bionictigers.axiom.core.commands.Scheduler
import io.github.bionictigers.axiom.core.web.Display
import java.util.SortedMap
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds

data class GainSchedule(
    val kP: Map<Double, Number>,
    val tI: Map<Double, Number>,
    val tD: Map<Double, Number>
) {
    operator fun get(index: Number): Triple<Double, Double, Double> {
        val num = index.toDouble()

        return Triple(
            interpolate(kP, num),
            interpolate(tI, num),
            interpolate(tD, num)
        )
    }
}

private fun interpolate(map: Map<Double, Number>, num: Double): Double {
    val exact = map[num]?.toDouble()
    if (exact != null) return exact

    val keys = map.keys.toList()
    val lower = keys.filter { it < num }.maxOrNull()
    val upper = keys.filter { it > num }.minOrNull()

    if (lower == null && upper == null) return 0.0

    val lowerValue = map[lower]?.toDouble() ?: return map[upper]!!.toDouble()
    val upperValue = map[upper]?.toDouble() ?: return lowerValue

    val slope = (upperValue - lowerValue) / (upper!! - lower!!)
    return lowerValue + slope * (num - lower)
}

class DynamicPID(
    val schedule: GainSchedule,
    kAW: Double = if (schedule[0.0].second == 0.0) 0.0 else 1.0 / schedule[0.0].second,
    pvMin: Double,
    pvMax: Double,
    cvMin: Double = -1.0,
    cvMax: Double = 1.0,
    sampleTime: Duration = 20.milliseconds,
    val indexProvider: (error: Double) -> Number,
    feedforward: (setpoint: Double, setpointRate: Double) -> Double = { _, _ -> 0.0 }
) : PID(
    schedule[indexProvider(0.0)].first,
    schedule[indexProvider(0.0)].second,
    schedule[indexProvider(0.0)].third,
    kAW,
    pvMin,
    pvMax,
    cvMin,
    cvMax,
    sampleTime,
    feedforward
) {
    @Display()
    val index: Double
        get() = indexProvider(error).toDouble()

    fun updateGains() {
        val (kP, tI, tD) = schedule[index]
        this.kP = kP
        this.tI = tI
        this.tD = tD
    }

    override fun compute(processValue: Double, setpoint: Double): Double {
        updateGains()
        return super.compute(processValue, setpoint)
    }
}