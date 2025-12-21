package org.firstinspires.ftc.teamcode.control

import io.github.bionictigers.axiom.core.web.Display
import io.github.bionictigers.axiom.core.web.Editable
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.DurationUnit
import kotlin.time.TimeMark
import kotlin.time.TimeSource

open class PID(
    @Display(priority = 13) @Editable var kP: Double,
    @Display(priority = 12) @Editable var tI: Double,
    @Display(priority = 11) @Editable var tD: Double,
    @Display(priority = 10) @Editable var kAW: Double = if (tI == 0.0) 0.0 else 1.0 / tI,
    val pvMin: Double,
    val pvMax: Double,
    val cvMin: Double = -1.0,
    val cvMax: Double = 1.0,
    private val sampleTime: Duration = 20.milliseconds,
    val feedforward: (setpoint: Double, setpointRate: Double) -> Double = { _, _ -> 0.0 }
) {
    constructor(
        kP: Number, tI: Number, tD: Number,
        kAW: Number = if (tI.toDouble() == 0.0) 0.0 else 1.0 / tI.toDouble(),
        pvMin: Number, pvMax: Number,
        cvMin: Number = -1, cvMax: Number = 1,
        sampleTime: Duration = 20.milliseconds
    ) : this(
        kP.toDouble(), tI.toDouble(), tD.toDouble(), kAW.toDouble(),
        pvMin.toDouble(), pvMax.toDouble(),
        cvMin.toDouble(), cvMax.toDouble(),
        sampleTime
    )

    private var lastCompute: TimeMark? = null

    @Display(priority = 9) var processValue: Double = 0.0
        private set(value) { field = value.coerceIn(pvMin, pvMax) }
    @Display(priority = 8) var setPoint: Double = 0.0
        private set(value) { field = value.coerceIn(pvMin, pvMax) }

    @Display(priority = 7) var cv: Double = 0.0
        private set
    @Display(priority = 6) var error: Double = 0.0
        private set
    @Display(priority = 5) var errorPercent: Double = 0.0
        private set
    @Display(priority = 3) var p: Double = 0.0
        private set
    @Display(priority = 2) var i: Double = 0.0
        private set
    @Display(priority = 1) var d: Double = 0.0
        private set

    private val kI get() = if (tI == 0.0) 0.0 else kP / tI
    private val kD get() = kP * tD

    private val span get() = pvMax - pvMin

    open fun compute(processValue: Double, setPoint: Double): Double {
        val dt = lastCompute?.elapsedNow() ?: sampleTime
        if (dt < sampleTime) return cv

        val dts = dt.toDouble(DurationUnit.SECONDS)

        error = setPoint - processValue
        errorPercent = if (pvMax - pvMin == 0.0) 0.0 else error / span

        p = kP * errorPercent
        d = kD * -(processValue - this.processValue) / span / dts

        val cvRaw = p + i + d + feedforward(setPoint, (setPoint - this.setPoint) / dts)
        cv = cvRaw.coerceIn(cvMin, cvMax)

        i += (kI * errorPercent + kAW * (cv - cvRaw)) * dts

        this.processValue = processValue
        this.setPoint = setPoint

        lastCompute = TimeSource.Monotonic.markNow()
        return cv
    }

    fun reset() {
        cv = 0.0
        i = 0.0
        lastCompute = null
    }
}