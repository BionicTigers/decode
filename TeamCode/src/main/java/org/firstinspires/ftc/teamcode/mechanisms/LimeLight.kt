package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.Scheduler.telemetry
import io.github.bionictigers.axiom.core.commands.System
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.atan2


class LimeLight(hardwareMap: HardwareMap, telemetry: Telemetry): System() {


    override val name = "Vision"
    enum class Obelisk {
        PPG,
        PGP,
        GPP,

        RED,

        BLUE,
        NO_DETECTION
    }
    val limeLight = hardwareMap.getByName<Limelight3A>("limeLight")
    var obeliskCode = Obelisk.NO_DETECTION
        private set

    init {
        limeLight.setPollRateHz(100)
        limeLight.start()
    }


    var colorSeen: Boolean = false
    fun getAngleFin(): Double? {
        limeLight.pipelineSwitch(0)
        val result = limeLight.latestResult
        val fiducials = result.fiducialResults
        fiducials?.forEach {
            obeliskCode = when (it?.fiducialId) {

                21 -> Obelisk.GPP
                22 -> Obelisk.PGP
                23 -> Obelisk.PPG


                else -> Obelisk.NO_DETECTION
            }


        }

        val blue = fiducials.find { it.fiducialId == 20 }
        val red = fiducials.find { it.fiducialId == 24 }

        telemetry?.addData("ID", fiducials.map { it.fiducialId }.toString())
        var angle: Double = 0.0
        if (red != null) {
            val x = red.targetXDegrees
            val y = red.targetYDegrees
            angle = atan2(y,x)

            if (blue != null) {
                val x = blue.targetXDegrees
                val y = blue.targetYDegrees
                angle = atan2(y,x)

            }

            var tx = result.getTx()
            var ty = result.getTy()

            val targetOffsetAngle_Vertical: Double = ty.toDouble()
            val mountDegrees: Double = -10.0
            val limeLightHeight = 10.0
            val goalHeight = 29.0

            var angleDegrees = mountDegrees + targetOffsetAngle_Vertical
            var angleRadians = angleDegrees * (3.14159 / 180.0)

            val distanceFromGoal = (goalHeight - limeLightHeight) / Math.atan(angleRadians)
            var rotation = atan2(tx,distanceFromGoal)
            return rotation
        }

        if (red != null || blue != null) {
            var colorSeen = true
        }

        return null
    }
}





