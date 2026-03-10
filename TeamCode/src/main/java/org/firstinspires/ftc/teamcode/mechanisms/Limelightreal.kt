package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import io.github.bionictigers.axiom.core.commands.Command

import io.github.bionictigers.axiom.core.commands.System
import io.github.bionictigers.axiom.core.scheduler.Scheduler.telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.Distance
import org.firstinspires.ftc.teamcode.utils.getByName
import kotlin.math.atan2


class LimeLight(hardwareMap: HardwareMap, telemetry: Telemetry, isRed: Boolean): System() {
    override val name = "Vision"
    enum class Obelisk {
        PPG,
        PGP,
        GPP,
        NO_DETECTION
    }

    val ticksToAngle = 0.03937

    val offsetBlue = 0.0
    val offsetRed = 0.0

    val limeLight = hardwareMap.getByName<Limelight3A>("limeLight")
    var obeliskCode = Obelisk.NO_DETECTION
        private set

    init {
        limeLight.setPollRateHz(100)
        limeLight.start()
    }

    fun getAngleFin(): Obelisk {
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
        telemetry?.addData("ID", fiducials.map { it.fiducialId }.toString())

        return obeliskCode
    }

    fun getAngle(): Double {
        limeLight.pipelineSwitch(0)
        val result = limeLight.latestResult
        val fiducials = result.fiducialResults
        var angleToTurn = 0.0

        fiducials?.forEach {
            if (it?.fiducialId == 20) { // blue
                val tx = result.tx
                angleToTurn = tx - offsetBlue
            } else if (it?.fiducialId == 24) { // red
                val tx = result.tx
                angleToTurn = tx - offsetRed
            }
        }

        telemetry?.addData("ID", fiducials.map { it.fiducialId }.toString())

        return angleToTurn
    }

    override val apply = SystemCommand.continuous {
        getAngleFin()
    }

}