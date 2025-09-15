package org.firstinspires.ftc.teamcode.control

import io.github.bionictigers.axiom.core.web.Display
import io.github.bionictigers.axiom.core.web.Editable
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.TimeMark

open class PID(
    @Display(priority = 12) @Editable var kP: Double,
    @Display(priority = 11) @Editable var tI: Double,
    @Display(priority = 10) @Editable var tD: Double,
    val pvMin: Double,
    val pvMax: Double,
    val cvMin: Double = -1.0,
    val cvMax: Double = 1.0,
    private val sampleTime: Duration = 20.milliseconds,
    val feedforward: (setpoint: Double, setpointRate: Double) -> Double = { _, _ -> 0.0 }
) {
    constructor(
        kP: Number, tI: Number, tD: Number,
        pvMin: Number, pvMax: Number,
        cvMin: Number = -1, cvMax: Number = 1,
        sampleTime: Duration = 20.milliseconds
    ) : this(
        kP.toDouble(), tI.toDouble(), tD.toDouble(),
        pvMin.toDouble(), pvMax.toDouble(),
        cvMin.toDouble(), cvMax.toDouble(),
        sampleTime
    )

    private val lastCompute: TimeMark? = null

    @Display(priority = 9) var processValue: Double = 0.0
        private set(value) {
            field = value.coerceIn(pvMin, pvMax)
        }
    @Display(priority = 8) var setpoint: Double = 0.0
        private set(value) {
            field = value.coerceIn(pvMin, pvMax)
        }
    @Display(priority = 7) var cv: Double = 0.0
        private set
    @Display(priority = 6) var error: Double = 0.0
        private set
    @Display(priority = 3) var p: Double = 0.0
        private set
    @Display(priority = 2) var i: Double = 0.0
        private set
    @Display(priority = 1) var d: Double = 0.0
        private set

    fun compute(processValue: Double, setpoint: Double) {

    }
}