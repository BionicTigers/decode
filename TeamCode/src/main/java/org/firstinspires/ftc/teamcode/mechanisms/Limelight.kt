package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap

import io.github.bionictigers.axiom.core.commands.System
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.getByName


class Limelight(hardwareMap: HardwareMap, val telemetry: Telemetry, val isRed: Boolean): System() {
    override val name = "Vision"
    enum class Obelisk {
        PPG,
        PGP,
        GPP,
        NO_DETECTION
    }

    val ticksToAngle = 0.03937

    val offsetBlue = 4.0
    val offsetRed = 1.1

    var lmlAimError = 0.0

    val limeLight = hardwareMap.getByName<Limelight3A>("limeLight")
    var obeliskCode = Obelisk.NO_DETECTION
        private set

    init {
        limeLight.setPollRateHz(100)
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

    fun getAngle(): Double {
        telemetry.addLine("doing angle")
        limeLight.pipelineSwitch(0)
        val result = limeLight.latestResult
        val fiducials = result.fiducialResults
        var angleToTurn = 0.0

        fiducials?.forEach {
            telemetry.addData("ID", it.fiducialId)
            print(it?.fiducialId)
            if (it?.fiducialId == 20) { // blue
                telemetry.addLine("blue")
                if (!isRed) {
                    val ty = result.ty
                    angleToTurn = ty - offsetBlue

                    telemetry.addData("tx blue", ty)
                }
            } else if (it?.fiducialId == 24) { // red
                telemetry.addLine("red")
                if (isRed) {
                    val ty = result.ty
                    angleToTurn = ty - offsetRed
                    telemetry.addData("tx red", ty)
                }
            }
        }

        telemetry.addData("ID", fiducials.map { it.fiducialId }.toString())

        return angleToTurn
    }

    override val apply = SystemCommand.continuous {
        lmlAimError = getAngle()
    }

}