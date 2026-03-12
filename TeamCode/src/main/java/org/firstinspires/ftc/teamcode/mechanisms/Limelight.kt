package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap

import io.github.bionictigers.axiom.core.commands.System
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.TimeMark
import kotlin.time.TimeSource


class Limelight(hardwareMap: HardwareMap, val telemetry: Telemetry? = null, val isRed: Boolean): System() {
    override val name = "Vision"

    data class AimMeasurement(
        val txDegrees: Double = 0.0,
        val valid: Boolean = false,
        val capturedAt: TimeMark? = null,
        val sequence: Long = -1L,
        val isNewSinceLastConsume: Boolean = false
    )

    enum class Obelisk {
        PPG,
        PGP,
        GPP,
        NO_DETECTION
    }

    val ticksToAngle = 0.03937
    private val measurementPublishInterval = 75.milliseconds
    private var aimMeasurementSequence = 0L
    private var consumedAimMeasurementSequence = -1L
    private var lastPublishedAimMeasurementAt: TimeMark? = null
    private var storedAimMeasurement = AimMeasurement()

    val aimMeasurement: AimMeasurement
        get() = storedAimMeasurement.copy(
            isNewSinceLastConsume = storedAimMeasurement.valid &&
                storedAimMeasurement.sequence != consumedAimMeasurementSequence
        )

    val limeLight = hardwareMap.getByName<Limelight3A>("limeLight")
    var obeliskCode = Obelisk.NO_DETECTION
        private set

    init {
        limeLight.setPollRateHz(10)
        limeLight.pipelineSwitch(0)
        limeLight.start()
    }

//    fun getObeliskCode(): Obelisk {
//        limeLight.pipelineSwitch(0)
//        val result = limeLight.latestResult
//        val fiducials = result.fiducialResults
//        fiducials?.forEach {
//            obeliskCode = when (it?.fiducialId) {
//                21 -> Obelisk.GPP
//                22 -> Obelisk.PGP
//                23 -> Obelisk.PPG
//                else -> Obelisk.NO_DETECTION
//            }
//        }
//        telemetry?.addData("ID", fiducials.map { it.fiducialId }.toString())
//
//        return obeliskCode
//    }

    fun getAngle(): Double = aimMeasurement.txDegrees

    fun consumeAimMeasurement(): AimMeasurement? {
        val measurement = aimMeasurement
        if (!measurement.valid || !measurement.isNewSinceLastConsume) return null

        consumedAimMeasurementSequence = measurement.sequence
        return measurement
    }

    private fun updateAimMeasurement() {
        val result = limeLight.latestResult
        val fiducials = result.fiducialResults.orEmpty()
        val targetId = if (isRed) 24 else 20
        val now = TimeSource.Monotonic.markNow()
        val seesTarget = fiducials.any { it?.fiducialId == targetId }
        val visibleIds = fiducials.mapNotNull { it?.fiducialId }

        telemetry?.addData("ll targetIds", visibleIds.toString())
        telemetry?.addData("ll targetVisible", seesTarget)

        if (!seesTarget) {
            storedAimMeasurement = AimMeasurement()
            telemetry?.addData("ll tx", "n/a")
            telemetry?.addData("ll aimError", "n/a")
            return
        }

        val canPublish = lastPublishedAimMeasurementAt?.elapsedNow()?.let { it >= measurementPublishInterval } != false
        val tx = result.tx

        telemetry?.addData("ll tx", tx)
        telemetry?.addData("ll measurementReady", canPublish)

        if (!canPublish) return

        storedAimMeasurement = AimMeasurement(
            txDegrees = tx,
            valid = true,
            capturedAt = now,
            sequence = ++aimMeasurementSequence
        )
        lastPublishedAimMeasurementAt = now
    }

    override val apply = SystemCommand.continuous("Limelight Aim") {
        updateAimMeasurement()
    }

}